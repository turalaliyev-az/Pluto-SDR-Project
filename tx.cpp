#include <iio.h>
#include <liquid/liquid.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static void die(const char* msg) { std::perror(msg); std::exit(1); }

int main(int argc, char** argv) {
    std::string uri = (argc >= 2) ? argv[1] : "usb:1.6.5"; // Öz Pluto IP/USB ünvanınızı yazın
    const long long LO_HZ = 915000000;

    iio_context* ctx = iio_create_context_from_uri(uri.c_str());
    if (!ctx) die("Context yaradıla bilmədi");

    iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    iio_channel* tx0_ctrl = iio_device_find_channel(phy, "voltage0", true);
    iio_channel* tx_lo = iio_device_find_channel(phy, "altvoltage1", true);
    iio_channel* tx_i = iio_device_find_channel(txdev, "voltage0", true);
    iio_channel* tx_q = iio_device_find_channel(txdev, "voltage1", true);

    iio_channel_attr_write_longlong(tx_lo, "frequency", LO_HZ);
    iio_channel_attr_write_longlong(tx0_ctrl, "sampling_frequency", 2500000);
    iio_channel_attr_write_longlong(tx0_ctrl, "hardwaregain", 0); // Max güc

    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);

    const size_t nsamps = 1 << 16;
    iio_buffer* buf = iio_device_create_buffer(txdev, nsamps, false);

    // Liquid DSP: Frame Generator
    flexframegenprops_s fgprops;
    flexframegenprops_init_default(&fgprops);
    fgprops.mod_scheme = LIQUID_MODEM_QPSK;
    fgprops.fec0 = LIQUID_FEC_GOLAY2412; 
    flexframegen fg = flexframegen_create(&fgprops);

    uint32_t counter = 0;
    uint8_t header[14] = {0};
    bool frame_active = false;

    std::fprintf(stderr, "TX başladı: %lld Hz\n", LO_HZ);

    while (true) {
        int16_t* p = (int16_t*)iio_buffer_start(buf);
        ptrdiff_t step = iio_buffer_step(buf);

        for (size_t n = 0; n < nsamps; n++) {
            if (!frame_active) {
                counter++;
                uint8_t payload[4];
                std::memcpy(payload, &counter, 4);
                
                flexframegen_assemble(fg, header, payload, 4);
                frame_active = true;
                
                if (counter % 100 == 0) 
                    std::printf("Göndərilir: %u\n", counter);
            }

            liquid_float_complex s;
            int complete = flexframegen_write_samples(fg, &s, 1);
            if (complete) frame_active = false;

            p[0] = (int16_t)(s.real * 15000.0f);
            p[1] = (int16_t)(s.imag * 15000.0f);
            p = (int16_t*)((char*)p + step);
        }
        iio_buffer_push(buf);
    }

    iio_buffer_destroy(buf);
    iio_context_destroy(ctx);
    return 0;
}