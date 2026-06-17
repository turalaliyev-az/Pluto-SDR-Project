#include <iio.h>
#include <liquid/liquid.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <vector>
#include <cstdlib>
#include <cerrno>

#define CHUNK_SIZE 1024
#define URI_TX "usb:1.13.5"

static void die(const char* msg) {
    std::perror(msg);
    std::exit(1);
}

// NASA Interleaving
void interleave(uint8_t* data, size_t len) {
    std::vector<uint8_t> temp(len);
    int cols = 32;
    int rows = len / cols;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            temp[c * rows + r] = data[r * cols + c];
        }
    }
    std::memcpy(data, temp.data(), len);
}

int main() {
    int v_fd = open("/dev/video0", O_RDWR);
    if (v_fd < 0) die("open /dev/video0");

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 320;
    fmt.fmt.pix.height = 240;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    if (ioctl(v_fd, VIDIOC_S_FMT, &fmt) < 0) die("VIDIOC_S_FMT");

    struct v4l2_requestbuffers req = {};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(v_fd, VIDIOC_REQBUFS, &req) < 0) die("VIDIOC_REQBUFS");

    struct v4l2_buffer vbuf = {};
    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_MMAP;
    vbuf.index = 0;
    if (ioctl(v_fd, VIDIOC_QUERYBUF, &vbuf) < 0) die("VIDIOC_QUERYBUF");

    void* cam_mem = mmap(NULL, vbuf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v_fd, vbuf.m.offset);
    if (cam_mem == MAP_FAILED) die("mmap");

    if (ioctl(v_fd, VIDIOC_QBUF, &vbuf) < 0) die("VIDIOC_QBUF(init)");

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v_fd, VIDIOC_STREAMON, &type) < 0) die("VIDIOC_STREAMON");

    iio_context* ctx = iio_create_context_from_uri(URI_TX);
    if (!ctx) die("iio_create_context_from_uri");

    iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (!phy || !txdev) die("TX device not found");

    iio_channel* tx_ctrl = iio_device_find_channel(phy, "voltage0", true);
    iio_channel* tx_lo = iio_device_find_channel(phy, "altvoltage1", true);
    if (!tx_ctrl || !tx_lo) die("TX control channel not found");

    iio_channel_attr_write_longlong(tx_lo, "frequency", 915000000);
    iio_channel_attr_write_longlong(tx_ctrl, "sampling_frequency", 1000000);
    iio_channel_attr_write_longlong(tx_ctrl, "hardwaregain", 0);

    iio_channel* ti = iio_device_find_channel(txdev, "voltage0", true);
    iio_channel* tq = iio_device_find_channel(txdev, "voltage1", true);
    if (!ti || !tq) die("TX IQ channels not found");

    iio_channel_enable(ti);
    iio_channel_enable(tq);

    const size_t buf_samps = 1 << 20;
    iio_buffer* buf = iio_device_create_buffer(txdev, buf_samps, false);
    if (!buf) die("iio_device_create_buffer");

    flexframegen fg = flexframegen_create(NULL);
    if (!fg) die("flexframegen_create");

    uint32_t f_id = 0;

    while (true) {
        if (ioctl(v_fd, VIDIOC_DQBUF, &vbuf) < 0) die("VIDIOC_DQBUF");

        uint32_t total = (vbuf.bytesused + CHUNK_SIZE - 1) / CHUNK_SIZE;
        f_id++;

        std::printf("Frame %u (%u paket) ötürülür...\n", f_id, total);
        std::fflush(stdout);

        int16_t* p = (int16_t*)iio_buffer_start(buf);
        ptrdiff_t step = iio_buffer_step(buf);
        size_t samples_written = 0;

        for (uint32_t i = 0; i < total; i++) {
            uint8_t hdr[14] = {0};
            std::memcpy(hdr, &f_id, 4);
            std::memcpy(hdr + 4, &i, 4);
            std::memcpy(hdr + 8, &total, 4);

            size_t off = i * CHUNK_SIZE;
            std::vector<uint8_t> payload(CHUNK_SIZE, 0xFF);

            size_t len = (vbuf.bytesused - off > CHUNK_SIZE) ? CHUNK_SIZE : (vbuf.bytesused - off);
            std::memcpy(payload.data(), (uint8_t*)cam_mem + off, len);

            interleave(payload.data(), CHUNK_SIZE);
            for (int b = 0; b < CHUNK_SIZE; b++) payload[b] ^= 0xAC;

            flexframegen_assemble(fg, hdr, payload.data(), CHUNK_SIZE);

            while (true) {
                liquid_float_complex s;
                int complete = flexframegen_write_samples(fg, &s, 1);

                if (samples_written < buf_samps) {
                    p[0] = (int16_t)(s.real * 15000.0f);
                    p[1] = (int16_t)(s.imag * 15000.0f);
                    p = (int16_t*)((char*)p + step);
                    samples_written++;
                }

                if (complete) break;
            }
        }

        size_t remaining = buf_samps - samples_written;
        if (remaining > 0) std::memset(p, 0, remaining * step);

        if (iio_buffer_push(buf) < 0) die("iio_buffer_push");

        std::printf("Frame %u tamamlandı. 3 saniyəlik fasilə...\n", f_id);
        std::fflush(stdout);

        if (ioctl(v_fd, VIDIOC_QBUF, &vbuf) < 0) die("VIDIOC_QBUF(loop)");
        sleep(3);
    }

    return 0;
}