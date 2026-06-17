#include <iio.h>
#include <liquid/liquid.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>   // Əlavə olundu
#include <vector>

static void die(const char* msg) { std::perror(msg); std::exit(1); }

// Gələn paketi emal edən funksiya
static int callback(unsigned char* header, int header_valid, 
                    unsigned char* payload, unsigned int payload_len, 
                    int payload_valid, framesyncstats_s stats, void* userdata) 
{
    if (payload_valid && payload_len == 4) {
        uint32_t val;
        std::memcpy(&val, payload, 4);
        
        // stats.snr səhvini düzəltmək üçün rssi və ya evm istifadə edirik
        std::printf("\r[ALINDI] Say: %u | RSSI: %.1f dB | EVM: %.1f dB   ", 
                    val, stats.rssi, stats.evm);
        std::fflush(stdout);
    }
    return 0;
}

int main(int argc, char** argv) {
    // Öz cihazınızın URI ünvanını bura yazın (məs: usb:1.5.5)
    std::string uri = (argc >= 2) ? argv[1] : "usb:1.5.5"; 
    const long long LO_HZ = 915000000;

    iio_context* ctx = iio_create_context_from_uri(uri.c_str());
    if (!ctx) die("Context tapılmadı. Pluto qoşuludurmu?");

    iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    
    if (!phy || !rxdev) die("Cihaz tapılmadı");

    iio_channel* rx0_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel* rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    iio_channel* rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel* rx_q = iio_device_find_channel(rxdev, "voltage1", false);

    iio_channel_attr_write_longlong(rx_lo, "frequency", LO_HZ);
    iio_channel_attr_write_longlong(rx0_ctrl, "sampling_frequency", 2500000);
    iio_channel_attr_write(rx0_ctrl, "gain_control_mode", "slow_attack");

    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);

    const size_t nsamps = 1 << 16;
    iio_buffer* buf = iio_device_create_buffer(rxdev, nsamps, false);
    if (!buf) die("Buffer yaradıla bilmədi");
    
    flexframesync fs = flexframesync_create(callback, NULL);
    std::vector<liquid_float_complex> rx_buf(nsamps);

    std::printf("RX Gözləyir (Tezlik: %lld MHz, URI: %s)...\n", LO_HZ/1000000, uri.c_str());

    while (true) {
        int ret = iio_buffer_refill(buf);
        if (ret < 0) break;

        void* start = iio_buffer_start(buf);
        ptrdiff_t step = iio_buffer_step(buf);
        char* p = (char*)start;

        for (size_t i = 0; i < nsamps; i++) {
            int16_t* s = (int16_t*)p;
            rx_buf[i].real = (float)s[0] / 32768.0f;
            rx_buf[i].imag = (float)s[1] / 32768.0f;
            p += step;
        }
        flexframesync_execute(fs, rx_buf.data(), nsamps);
    }

    iio_buffer_destroy(buf);
    iio_context_destroy(ctx);
    flexframesync_destroy(fs);
    return 0;
}