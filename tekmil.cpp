#include <iio.h>
#include <liquid/liquid.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>
#include <algorithm>

#define URI_DEV "usb:1.11.5"
#define CHUNK_SIZE 1024

static constexpr uint8_t CMD_VIDEO = 0xA1;
static constexpr uint8_t CMD_PHOTO = 0xB2;

static constexpr long long CMD_RX_FREQ   = 900000000;
static constexpr long long VIDEO_TX_FREQ = 915000000;
static constexpr long long SAMPLE_RATE   = 2083333;
static constexpr long long RF_BW         = 1500000;

enum Mode {
    MODE_VIDEO = 0,
    MODE_PHOTO = 1
};

static std::atomic<int> g_mode{MODE_VIDEO};
static std::atomic<bool> g_run{true};

static void die(const char* msg) {
    std::perror(msg);
    std::exit(1);
}

static void interleave(uint8_t* data, size_t len) {
    std::vector<uint8_t> temp(len);
    const int cols = 32;
    const int rows = static_cast<int>(len / cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            temp[c * rows + r] = data[r * cols + c];
        }
    }

    std::memcpy(data, temp.data(), len);
}

// Header: 8 byte
// [0..3] frame_id  (uint32_t)
// [4..5] chunk_idx (uint16_t)
// [6..7] total     (uint16_t)
static int cmd_callback(unsigned char* hdr,
                        int h_valid,
                        unsigned char* pay,
                        unsigned int pay_len,
                        int p_valid,
                        framesyncstats_s stats,
                        void* userdata) {
    (void)hdr;
    (void)h_valid;
    (void)stats;
    (void)userdata;

    if (!p_valid || !pay || pay_len < 1) return 0;

    if (pay[0] == CMD_VIDEO) {
        g_mode = MODE_VIDEO;
        std::printf("\n[OTURUCU] Komanda alindi: VIDEO\n");
    } else if (pay[0] == CMD_PHOTO) {
        g_mode = MODE_PHOTO;
        std::printf("\n[OTURUCU] Komanda alindi: FOTO\n");
    } else {
        std::printf("\n[OTURUCU] Namelum komanda: 0x%02X\n", pay[0]);
    }

    return 0;
}

static void rx_worker(iio_buffer* rx_buf, flexframesync fs) {
    const size_t max_samps = 1 << 20;
    std::vector<liquid_float_complex> iq(max_samps);

    while (g_run) {
        const ssize_t nbytes = iio_buffer_refill(rx_buf);
        if (nbytes < 0) continue;

        char* p = (char*)iio_buffer_start(rx_buf);
        const ptrdiff_t step = iio_buffer_step(rx_buf);
        if (!p || step <= 0) continue;

        size_t count = (size_t)(nbytes / step);
        if (count > max_samps) count = max_samps;

        for (size_t i = 0; i < count; ++i) {
            int16_t* s = (int16_t*)p;
            iq[i].real = (float)s[0] / 32768.0f;
            iq[i].imag = (float)s[1] / 32768.0f;
            p += step;
        }

        flexframesync_execute(fs, iq.data(), count);
    }
}

int main() {
    int v_fd = open("/dev/video0", O_RDWR);
    if (v_fd < 0) die("open /dev/video0");

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 320;
    fmt.fmt.pix.height = 240;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    if (ioctl(v_fd, VIDIOC_S_FMT, &fmt) < 0) die("VIDIOC_S_FMT");

    v4l2_requestbuffers req{};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(v_fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS");

    v4l2_buffer vbuf{};
    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_MMAP;
    vbuf.index = 0;
    if (ioctl(v_fd, VIDIOC_QUERYBUF, &vbuf) < 0) die("VIDIOC_QUERYBUF");

    void* cam_mem = mmap(nullptr, vbuf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v_fd, vbuf.m.offset);
    if (cam_mem == MAP_FAILED) die("mmap");

    if (ioctl(v_fd, VIDIOC_QBUF, &vbuf) < 0) die("VIDIOC_QBUF(init)");

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v_fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON");

    iio_context* ctx = iio_create_context_from_uri(URI_DEV);
    if (!ctx) die("iio_create_context_from_uri");

    iio_device* phy   = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    iio_device* rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!phy || !txdev || !rxdev) {
        std::fprintf(stderr, "XETA: Pluto device tapilmadi\n");
        return 1;
    }

    iio_channel* rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel* rx_lo   = iio_device_find_channel(phy, "altvoltage0", true);

    iio_channel* tx_ctrl = iio_device_find_channel(phy, "voltage0", true);
    iio_channel* tx_lo   = iio_device_find_channel(phy, "altvoltage1", true);

    if (!rx_ctrl || !rx_lo || !tx_ctrl || !tx_lo) {
        std::fprintf(stderr, "XETA: ctrl/LO tapilmadi\n");
        return 1;
    }

    iio_channel_attr_write(rx_ctrl, "rf_port_select", "A_BALANCED");
    iio_channel_attr_write(rx_ctrl, "gain_control_mode", "manual");
    iio_channel_attr_write_longlong(rx_ctrl, "hardwaregain", 60);
    iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(rx_ctrl, "rf_bandwidth", RF_BW);
    iio_channel_attr_write_longlong(rx_lo, "frequency", CMD_RX_FREQ);

    iio_channel_attr_write(tx_ctrl, "rf_port_select", "B");
    iio_channel_attr_write_longlong(tx_ctrl, "hardwaregain", -10);
    iio_channel_attr_write_longlong(tx_ctrl, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(tx_ctrl, "rf_bandwidth", RF_BW);
    iio_channel_attr_write_longlong(tx_lo, "frequency", VIDEO_TX_FREQ);

    iio_channel* rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel* rx_q = iio_device_find_channel(rxdev, "voltage1", false);

    iio_channel* tx_i = iio_device_find_channel(txdev, "voltage0", true);
    iio_channel* tx_q = iio_device_find_channel(txdev, "voltage1", true);

    if (!rx_i || !rx_q || !tx_i || !tx_q) {
        std::fprintf(stderr, "XETA: IQ stream channel tapilmadi\n");
        return 1;
    }

    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);
    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);

    iio_buffer* rx_buf = iio_device_create_buffer(rxdev, 1 << 20, false);
    iio_buffer* tx_buf = iio_device_create_buffer(txdev, 1 << 20, false);
    if (!rx_buf || !tx_buf) {
        std::fprintf(stderr, "XETA: RX/TX buffer yaradilmaadi\n");
        return 1;
    }

    flexframesync fs = flexframesync_create(cmd_callback, nullptr);
    flexframegen fg  = flexframegen_create(nullptr);
    if (!fs || !fg) {
        std::fprintf(stderr, "XETA: liquid obyektleri yaradilmaadi\n");
        return 1;
    }

    std::thread t_cmd(rx_worker, rx_buf, fs);

    std::printf("[OTURUCU] Basladi | RX(A)=900MHz komanda | TX(B)=915MHz video\n");

    uint32_t frame_id = 0;

    while (g_run) {
        if (ioctl(v_fd, VIDIOC_DQBUF, &vbuf) < 0) die("VIDIOC_DQBUF");

        const uint32_t bytes_used = vbuf.bytesused;
        if (bytes_used == 0) {
            if (ioctl(v_fd, VIDIOC_QBUF, &vbuf) < 0) die("VIDIOC_QBUF(empty)");
            continue;
        }

        const uint16_t total_chunks = (uint16_t)((bytes_used + CHUNK_SIZE - 1) / CHUNK_SIZE);
        ++frame_id;

        std::printf("[OTURUCU] Frame=%u bytes=%u chunks=%u mode=%s\n",
                    frame_id, bytes_used, total_chunks,
                    (g_mode.load() == MODE_VIDEO ? "VIDEO" : "FOTO"));

        int16_t* out = (int16_t*)iio_buffer_start(tx_buf);
        const ptrdiff_t step = iio_buffer_step(tx_buf);
        if (!out || step <= 0) {
            std::fprintf(stderr, "XETA: TX buffer start/step\n");
            break;
        }

        size_t written = 0;

        for (uint16_t chk = 0; chk < total_chunks; ++chk) {
            uint8_t hdr[8] = {0};
            std::memcpy(hdr + 0, &frame_id, 4);
            std::memcpy(hdr + 4, &chk, 2);
            std::memcpy(hdr + 6, &total_chunks, 2);

            const size_t off = (size_t)chk * CHUNK_SIZE;
            const size_t remain = (bytes_used > off) ? (bytes_used - off) : 0;
            const size_t part_len = std::min<size_t>(CHUNK_SIZE, remain);

            std::vector<uint8_t> payload(CHUNK_SIZE, 0xFF);
            if (part_len > 0) {
                std::memcpy(payload.data(), (uint8_t*)cam_mem + off, part_len);
            }

            interleave(payload.data(), CHUNK_SIZE);
            for (size_t i = 0; i < CHUNK_SIZE; ++i) payload[i] ^= 0xAC;

            flexframegen_reset(fg);
            flexframegen_assemble(fg, hdr, payload.data(), CHUNK_SIZE);

            while (true) {
                liquid_float_complex s;
                int done = flexframegen_write_samples(fg, &s, 1);

                if (written < (1 << 20)) {
                    out[0] = (int16_t)(s.real * 15000.0f);
                    out[1] = (int16_t)(s.imag * 15000.0f);
                    out = (int16_t*)((char*)out + step);
                    ++written;
                }

                if (done) break;
            }
        }

        if (written < (1 << 20)) {
            std::memset(out, 0, ((1 << 20) - written) * step);
        }

        if (iio_buffer_push(tx_buf) < 0) die("iio_buffer_push");

        if (ioctl(v_fd, VIDIOC_QBUF, &vbuf) < 0) die("VIDIOC_QBUF(loop)");

        if (g_mode.load() == MODE_PHOTO) sleep(2);
        else usleep(120000);
    }

    g_run = false;
    if (t_cmd.joinable()) t_cmd.join();

    flexframegen_destroy(fg);
    flexframesync_destroy(fs);
    iio_buffer_destroy(tx_buf);
    iio_buffer_destroy(rx_buf);
    iio_context_destroy(ctx);

    munmap(cam_mem, vbuf.length);
    close(v_fd);
    return 0;
}