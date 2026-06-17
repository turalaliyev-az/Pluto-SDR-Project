#include <iio.h>
#include <liquid/liquid.h>
#include <cstdio>
#include <vector>
#include <cstring>

#define CHUNK_SIZE 1024
#define URI_RX "usb:1.5.5"

struct State {
    std::vector<uint8_t> frame_data;
    uint32_t last_fid = 0;
    uint32_t chunks_received = 0;
};

void de_interleave(uint8_t* data, size_t len) {
    std::vector<uint8_t> temp(len);
    int cols = 32; int rows = len / cols;
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            temp[r * cols + c] = data[c * rows + r];
        }
    }
    std::memcpy(data, temp.data(), len);
}

static int rx_callback(unsigned char* hdr, int h_valid, unsigned char* pay, 
                       unsigned int pay_len, int p_valid, framesyncstats_s stats, void* userdata) {
    State* st = (State*)userdata;
    if (!p_valid) return 0;

    uint32_t fid, chk_idx, total;
    memcpy(&fid, hdr, 4); memcpy(&chk_idx, hdr+4, 4); memcpy(&total, hdr+8, 4);

    if (fid != st->last_fid) {
        printf("\n[*] Qəbul: Frame %u (%u paket)\n", fid, total);
        st->frame_data.clear(); st->frame_data.resize(total * CHUNK_SIZE, 0xFF);
        st->last_fid = fid; st->chunks_received = 0;
    }

    if (chk_idx * CHUNK_SIZE < st->frame_data.size()) {
        for(int b=0; b<pay_len; b++) pay[b] ^= 0xAC;
        de_interleave(pay, CHUNK_SIZE);
        std::memcpy(st->frame_data.data() + (chk_idx * CHUNK_SIZE), pay, pay_len);
        st->chunks_received++;
        printf("\r    Paket %u/%u tutuldu... RSSI: %.1f dB", st->chunks_received, total, stats.rssi);
        fflush(stdout);
    }

    if (st->chunks_received >= total - 2) { // 2 paket itkisinə dözümlülük
        printf("\n[OK] Frame %u ALINDI!\n", fid);
        char name[32]; sprintf(name, "nasa_output_%u.jpg", fid);
        FILE* f = fopen(name, "wb");
        if(f) { fwrite(st->frame_data.data(), 1, st->frame_data.size(), f); fclose(f); }
        st->chunks_received = 0;
    }
    return 0;
}

int main() {
    iio_context* ctx = iio_create_context_from_uri(URI_RX);
    iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    iio_channel* rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    
    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", 915000000);
    iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", 1000000);
    iio_channel_attr_write_longlong(rx_ctrl, "hardwaregain", 55);

    iio_channel* ri = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel* rq = iio_device_find_channel(rxdev, "voltage1", false);
    iio_channel_enable(ri); iio_channel_enable(rq);

    const size_t nsamps = 1 << 20; // Böyük kadrları tutmaq üçün böyük buffer
    iio_buffer* buf = iio_device_create_buffer(rxdev, nsamps, false);
    State* st = new State();
    flexframesync fs = flexframesync_create(rx_callback, st);

    std::vector<liquid_float_complex> iq(nsamps);
    printf("NASA RX Dinləyir (Böyük Kadr Rejimi)...\n");

    while (true) {
        if (iio_buffer_refill(buf) < 0) continue;
        char* p = (char*)iio_buffer_start(buf);
        for (size_t i=0; i < nsamps; i++) {
            int16_t* s = (int16_t*)p;
            iq[i] = {(float)s[0]/32768.0f, (float)s[1]/32768.0f};
            p += iio_buffer_step(buf);
        }
        flexframesync_execute(fs, iq.data(), nsamps);
    }
}