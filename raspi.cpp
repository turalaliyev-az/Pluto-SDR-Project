// rx_video.cpp - AĞILLI DİAQNOSTİKA QƏBULEDİCİSİ
// Kompilyasiya: g++ -O3 -o raspi raspi.cpp -liio -lliquid -lm
/*
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
static void check_ret(int ret, const char* what) { if (ret < 0) { std::fprintf(stderr, "ERR: %s (%d)\n", what, ret); std::exit(1); } }

static bool try_write_ll(iio_channel* ch, const char* attr, long long v) {
    int ret = iio_channel_attr_write_longlong(ch, attr, v);
    return (ret == -2 || ret < 0) ? false : true;
}

static constexpr long long LO_HZ = 915000000LL;
static constexpr long long SR_HZ = 2500000LL;
static constexpr long long BW_HZ = 2000000LL;
static constexpr long long RX_GAIN_DB = 45LL; 
static constexpr int PAYLOAD_SIZE = 64; 

static void write_jpeg(const char *path, const uint8_t *data, unsigned long size) {
    char tmp_path[256]; snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    FILE *f = std::fopen(tmp_path, "wb"); if (!f) return;
    std::fwrite(data, 1, size, f);
    std::fclose(f);
    rename(tmp_path, path);
}

struct RxState {
    long long current_gain = RX_GAIN_DB; 
    iio_channel *rx_ctrl = nullptr; 
    uint64_t ok = 0, bad = 0, frames = 0; 
    float last_rssi = -99.0f;
    int agc_counter = 0; 

    uint16_t current_fid = 0xFFFF;
    uint8_t frame_buf[131072] = {0}; 
    bool chunks_got[2048]; 
    int received_chunks = 0;
    int expected_chunks = 0;
    unsigned long max_offset = 0;
    bool frame_already_saved = false;

    RxState() { memset(chunks_got, 0, sizeof(chunks_got)); }

    void flush_jpeg() {
        if (received_chunks > 0 && expected_chunks > 0) {
            float success_rate = (float)received_chunks / expected_chunks;
            
            if (success_rate >= 0.60f && !frame_already_saved) {
                // Yoxlayırıq: Şəklin başlığı (0xFF 0xD8) yerdədirmi?
                if (max_offset > 4 && frame_buf[0] == 0xFF && frame_buf[1] == 0xD8) {
                    write_jpeg("/tmp/rx_latest.jpg", frame_buf, max_offset);
                    frames++;
                    frame_already_saved = true;
                    std::fprintf(stderr, "\n[AXIN] ID: %u | Bütövlük: %d%% | Kadr ekranda yeniləndi!\n", 
                        current_fid, (int)(success_rate * 100));
                } else {
                    std::fprintf(stderr, "\n[XƏTA] ID: %u | Bütövlük: %d%% | BAŞLIQ İTİB (Kadr aça bilmir, atıldı)!\n", 
                        current_fid, (int)(success_rate * 100));
                }
            } else if (!frame_already_saved) {
                 std::fprintf(stderr, "\n[ZƏİF] ID: %u | Bütövlük yalnız %d%%, kadr atıldı.\n", 
                    current_fid, (int)(success_rate*100));
            }
        }
        
        memset(chunks_got, 0, sizeof(chunks_got));
        memset(frame_buf, 0, sizeof(frame_buf)); 
        received_chunks = 0; expected_chunks = 0; max_offset = 0;
        frame_already_saved = false;
    }
};

static int frame_callback(unsigned char *hdr, int h_val, unsigned char *pay, unsigned int p_len, int p_val, framesyncstats_s st, void *ud) {
    RxState *s = (RxState*)ud; if (!s) return 0;
    s->last_rssi = st.rssi;
    
    if (!h_val || !p_val || p_len == 0) { s->bad++; return 0; }
    
    uint16_t fid = ((uint16_t)hdr[0] << 8) | hdr[1]; 
    uint16_t chunk_idx = ((uint16_t)hdr[2] << 8) | hdr[3]; 
    uint16_t total_chunks = ((uint16_t)hdr[4] << 8) | hdr[5]; 
    uint16_t actual_len = ((uint16_t)hdr[6] << 8) | hdr[7]; 
    
    if (actual_len > PAYLOAD_SIZE || chunk_idx >= 2048) { s->bad++; return 0; }
    s->ok++;
    
    s->agc_counter++;
    if (s->agc_counter > 200) { 
        long long ng = s->current_gain;
        if (st.rssi > -35.0f) ng -= 1; else if (st.rssi < -55.0f) ng += 1;
        ng = ng < 0 ? 0 : ng > 71 ? 71 : ng;
        if (ng != s->current_gain && s->rx_ctrl) 
            if (try_write_ll(s->rx_ctrl, "hardwaregain", ng)) s->current_gain = ng;
        s->agc_counter = 0;
    }
    
    if (fid != s->current_fid) {
        s->flush_jpeg();
        s->current_fid = fid;
    }
    
    int offset = chunk_idx * PAYLOAD_SIZE; 
    if (offset + actual_len <= sizeof(s->frame_buf)) {
        if (!s->chunks_got[chunk_idx]) {
            memcpy(s->frame_buf + offset, pay, actual_len); 
            s->chunks_got[chunk_idx] = true;
            s->received_chunks++;
        }
        s->expected_chunks = total_chunks;
        if (offset + actual_len > s->max_offset) s->max_offset = offset + actual_len;
        
        if (s->received_chunks == s->expected_chunks && !s->frame_already_saved) {
            if (s->max_offset > 4 && s->frame_buf[0] == 0xFF && s->frame_buf[1] == 0xD8) {
                write_jpeg("/tmp/rx_latest.jpg", s->frame_buf, s->max_offset);
                s->frames++;
                s->frame_already_saved = true;
                std::fprintf(stderr, "\n[AXIN] ID: %u | Bütövlük: 100%% | Mükəmməl Kadr!\n", s->current_fid);
            }
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    std::string uri = (argc >= 2) ? argv[1] : "usb:1.7.5";
    iio_context *ctx = iio_create_context_from_uri(uri.c_str()); if (!ctx) die("Cihaz tapılmadı");
    
    iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device *rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    iio_channel *rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    iio_channel *rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel *rx_q = iio_device_find_channel(rxdev, "voltage1", false);
    
    check_ret(iio_channel_attr_write_longlong(rx_lo, "frequency", LO_HZ), "freq");
    check_ret(iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", SR_HZ), "sr");
    check_ret(iio_channel_attr_write_longlong(rx_ctrl, "rf_bandwidth", BW_HZ), "bw");
    try_write_ll(rx_ctrl, "hardwaregain", RX_GAIN_DB);
    
    iio_channel_enable(rx_i); iio_channel_enable(rx_q);
    
    const size_t nsamps = 1 << 16; 
    iio_buffer *buf = iio_device_create_buffer(rxdev, nsamps, false);
    
    std::fprintf(stderr, "\n[RX] NASA DİAQNOSTİKA QƏBULEDİCİSİ BAŞLADI.\n");
    std::fprintf(stderr, "[RX] İzləmək üçün: feh --reload 0.05 /tmp/rx_latest.jpg\n\n");
    
    RxState state; state.current_gain = RX_GAIN_DB; state.rx_ctrl = rx_ctrl;
    flexframesync fs = flexframesync_create(frame_callback, &state); 
    std::vector<liquid_float_complex> rx_buf(nsamps);
    float i_avg = 0.0f, q_avg = 0.0f; const float scale = 1.0f / 32768.0f;
    uint64_t loop = 0;

    while (true) {
        int nbytes = iio_buffer_refill(buf); if (nbytes < 0) break;
        loop++;
        ptrdiff_t step = iio_buffer_step(buf); char *p = (char*)iio_buffer_start(buf); size_t count = (size_t)nbytes / (size_t)step;
        for (size_t i = 0; i < count; ++i) {
            int16_t *sp = (int16_t*)p; float r = (float)sp[0] * scale, im = (float)sp[1] * scale; p += step;
            i_avg = 0.9995f * i_avg + 0.0005f * r; q_avg = 0.9995f * q_avg + 0.0005f * im;
            rx_buf[i].real = r - i_avg; rx_buf[i].imag = im - q_avg;
        }
        flexframesync_execute(fs, rx_buf.data(), (unsigned int)count);

        if (loop % 100 == 0) {
            std::fprintf(stderr, "[RADİO] Oxunur... Paket: %llu | İtmiş: %llu | RSSI: %.1f\r", 
                (unsigned long long)state.ok, (unsigned long long)state.bad, state.last_rssi);
        }
    }
    return 0;
}
  

*/





























































  


























































// rx_video.cpp - STABİL GÖRÜNTÜ QƏBULEDİCİSİ (75% Bütövlük)
/*
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
static void check_ret(int ret, const char* what) { if (ret < 0) { std::fprintf(stderr, "ERR: %s (%d)\n", what, ret); std::exit(1); } }

static bool try_write_ll(iio_channel* ch, const char* attr, long long v) {
    int ret = iio_channel_attr_write_longlong(ch, attr, v);
    return (ret == -2 || ret < 0) ? false : true;
}

static constexpr long long LO_HZ = 915000000LL;
static constexpr long long SR_HZ = 2500000LL;
static constexpr long long BW_HZ = 2000000LL;
static constexpr long long RX_GAIN_DB = 45LL; 
static constexpr int PAYLOAD_SIZE = 64; 

static void write_jpeg(const char *path, const uint8_t *data, unsigned long size) {
    char tmp_path[256]; snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    FILE *f = std::fopen(tmp_path, "wb"); if (!f) return;
    std::fwrite(data, 1, size, f);
    std::fclose(f);
    rename(tmp_path, path);
}

struct RxState {
    long long current_gain = RX_GAIN_DB; 
    iio_channel *rx_ctrl = nullptr; 
    uint64_t ok = 0, bad = 0, frames = 0; 
    float last_rssi = -99.0f;
    int agc_counter = 0; 

    uint16_t current_fid = 0xFFFF;
    uint8_t frame_buf[131072] = {0}; 
    bool chunks_got[2048]; 
    int received_chunks = 0;
    int expected_chunks = 0;
    unsigned long max_offset = 0;
    bool frame_already_saved = false;

    RxState() { memset(chunks_got, 0, sizeof(chunks_got)); }

    void flush_jpeg() {
        if (received_chunks > 0 && expected_chunks > 0) {
            float success_rate = (float)received_chunks / expected_chunks;
            
            // 75% HƏDDİ: Həm axıcılığı təmin edir, həm də gözü yoran kəskin karıncalanmanı gizlədir.
            if (success_rate >= 0.75f && !frame_already_saved) {
                if (max_offset > 4 && frame_buf[0] == 0xFF && frame_buf[1] == 0xD8) {
                    write_jpeg("/tmp/rx_latest.jpg", frame_buf, max_offset);
                    frames++;
                    frame_already_saved = true;
                    std::fprintf(stderr, "\n[AXIN] ID: %u | Təmizlik: %d%% | Ekrana verildi!\n", 
                        current_fid, (int)(success_rate * 100));
                }
            }
        }
        
        memset(chunks_got, 0, sizeof(chunks_got));
        memset(frame_buf, 0, sizeof(frame_buf)); 
        received_chunks = 0; expected_chunks = 0; max_offset = 0;
        frame_already_saved = false;
    }
};

static int frame_callback(unsigned char *hdr, int h_val, unsigned char *pay, unsigned int p_len, int p_val, framesyncstats_s st, void *ud) {
    RxState *s = (RxState*)ud; if (!s) return 0;
    s->last_rssi = st.rssi;
    
    if (!h_val || !p_val || p_len == 0) { s->bad++; return 0; }
    
    uint16_t fid = ((uint16_t)hdr[0] << 8) | hdr[1]; 
    uint16_t chunk_idx = ((uint16_t)hdr[2] << 8) | hdr[3]; 
    uint16_t total_chunks = ((uint16_t)hdr[4] << 8) | hdr[5]; 
    uint16_t actual_len = ((uint16_t)hdr[6] << 8) | hdr[7]; 
    
    if (actual_len > PAYLOAD_SIZE || chunk_idx >= 2048) { s->bad++; return 0; }
    s->ok++;
    
    s->agc_counter++;
    if (s->agc_counter > 200) { 
        long long ng = s->current_gain;
        if (st.rssi > -35.0f) ng -= 1; else if (st.rssi < -55.0f) ng += 1;
        ng = ng < 0 ? 0 : ng > 71 ? 71 : ng;
        if (ng != s->current_gain && s->rx_ctrl) 
            if (try_write_ll(s->rx_ctrl, "hardwaregain", ng)) s->current_gain = ng;
        s->agc_counter = 0;
    }
    
    if (fid != s->current_fid) {
        s->flush_jpeg();
        s->current_fid = fid;
    }
    
    int offset = chunk_idx * PAYLOAD_SIZE; 
    if (offset + actual_len <= sizeof(s->frame_buf)) {
        if (!s->chunks_got[chunk_idx]) {
            memcpy(s->frame_buf + offset, pay, actual_len); 
            s->chunks_got[chunk_idx] = true;
            s->received_chunks++;
        }
        s->expected_chunks = total_chunks;
        if (offset + actual_len > s->max_offset) s->max_offset = offset + actual_len;
        
        if (s->received_chunks == s->expected_chunks && !s->frame_already_saved) {
            if (s->max_offset > 4 && s->frame_buf[0] == 0xFF && s->frame_buf[1] == 0xD8) {
                write_jpeg("/tmp/rx_latest.jpg", s->frame_buf, s->max_offset);
                s->frames++;
                s->frame_already_saved = true;
                std::fprintf(stderr, "\n[AXIN] ID: %u | Təmizlik: 100%% | Mükəmməl Kadr!\n", s->current_fid);
            }
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    std::string uri = (argc >= 2) ? argv[1] : "usb:1.10.5";
    iio_context *ctx = iio_create_context_from_uri(uri.c_str()); if (!ctx) die("Cihaz tapılmadı");
    
    iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device *rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    iio_channel *rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    iio_channel *rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel *rx_q = iio_device_find_channel(rxdev, "voltage1", false);
    
    check_ret(iio_channel_attr_write_longlong(rx_lo, "frequency", LO_HZ), "freq");
    check_ret(iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", SR_HZ), "sr");
    check_ret(iio_channel_attr_write_longlong(rx_ctrl, "rf_bandwidth", BW_HZ), "bw");
    try_write_ll(rx_ctrl, "hardwaregain", RX_GAIN_DB);
    
    iio_channel_enable(rx_i); iio_channel_enable(rx_q);
    
    const size_t nsamps = 1 << 16; 
    iio_buffer *buf = iio_device_create_buffer(rxdev, nsamps, false);
    
    std::fprintf(stderr, "\n[RX] STABİL VƏ AXICI QƏBULEDİCİ BAŞLADI.\n");
    std::fprintf(stderr, "[RX] İzləmək üçün: feh --reload 0.05 /tmp/rx_latest.jpg\n\n");
    
    RxState state; state.current_gain = RX_GAIN_DB; state.rx_ctrl = rx_ctrl;
    flexframesync fs = flexframesync_create(frame_callback, &state); 
    std::vector<liquid_float_complex> rx_buf(nsamps);
    float i_avg = 0.0f, q_avg = 0.0f; const float scale = 1.0f / 32768.0f;
    uint64_t loop = 0;

    while (true) {
        int nbytes = iio_buffer_refill(buf); if (nbytes < 0) break;
        loop++;
        ptrdiff_t step = iio_buffer_step(buf); char *p = (char*)iio_buffer_start(buf); size_t count = (size_t)nbytes / (size_t)step;
        for (size_t i = 0; i < count; ++i) {
            int16_t *sp = (int16_t*)p; float r = (float)sp[0] * scale, im = (float)sp[1] * scale; p += step;
            i_avg = 0.9995f * i_avg + 0.0005f * r; q_avg = 0.9995f * q_avg + 0.0005f * im;
            rx_buf[i].real = r - i_avg; rx_buf[i].imag = im - q_avg;
        }
        flexframesync_execute(fs, rx_buf.data(), (unsigned int)count);

        if (loop % 100 == 0) {
            std::fprintf(stderr, "[RADİO] Oxunur... Paket: %llu | İtmiş: %llu | RSSI: %.1f\r", 
                (unsigned long long)state.ok, (unsigned long long)state.bad, state.last_rssi);
        }
    }
    return 0;
}
    */






































































































#include <iio.h>
#include <liquid/liquid.h>
#include <cstdio>
#include <vector>
#include <iostream>
#include <cstring>

// --- AYARLAR ---
static constexpr long long FREQ = 915000000LL;
static constexpr long long RATE = 2000000LL;
static constexpr int PAYLOAD = 64;

struct RxState {
    uint8_t frame_buf[131072]; // 128KB JPEG üçün
    bool chunks_got[2048];
    uint16_t current_fid = 0xFFFF;
    int received = 0;
    unsigned long max_off = 0;

    void reset() {
        memset(chunks_got, 0, sizeof(chunks_got));
        received = 0;
        max_off = 0;
    }

    void save() {
        if (received > 10) { // Ən az 10 paket gəlibsə şəkli yarat
            // JPEG sonluğunu məcburi əlavə et (EOI)
            frame_buf[max_off] = 0xFF; 
            frame_buf[max_off+1] = 0xD9;
            FILE *f = fopen("/tmp/rx_latest.jpg", "wb");
            if(f) {
                fwrite(frame_buf, 1, max_off+2, f);
                fclose(f);
                printf("\r[RX] YENİ KADR: ID %d | Paket sayı: %d | Ölçü: %lu bytes    ", current_fid, received, max_off);
                fflush(stdout);
            }
        }
    }
};

// Siqnal tapıldıqda Liquid-DSP tərəfindən çağırılır
int rx_callback(unsigned char *hdr, int h_v, unsigned char *pay, unsigned int p_l, int p_v, framesyncstats_s st, void *ud) {
    RxState *s = (RxState*)ud;
    if (!h_v || !p_v) return 0;

    uint16_t fid = (hdr[0]<<8) | hdr[1]; // Frame ID
    uint16_t cid = (hdr[2]<<8) | hdr[3]; // Chunk ID

    // Yeni kadr başlayıbsa köhnəni saxla
    if (fid != s->current_fid) {
        s->save();
        s->reset();
        s->current_fid = fid;
    }

    if (cid < 2048 && !s->chunks_got[cid]) {
        memcpy(s->frame_buf + (cid * PAYLOAD), pay, PAYLOAD);
        s->chunks_got[cid] = true;
        s->received++;
        if ((cid * PAYLOAD) + PAYLOAD > s->max_off) 
            s->max_off = (cid * PAYLOAD) + PAYLOAD;
    }
    return 0;
}

int main(int argc, char **argv) {
    // 1. PlutoSDR Bağlantısı
    iio_context *ctx = (argc > 1) ? iio_create_context_from_uri(argv[1]) : iio_create_context_from_uri("ip:192.168.2.1");
    if (!ctx) ctx = iio_create_local_context();
    if (!ctx) {
        std::cerr << "Pluto tapılmadı! Cihazın bağlı olduğundan və ya başqa proqramın açıq olmadığından əmin ol.\n";
        return -1;
    }

    iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device *rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

    // 2. Radio Ayarları
    iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    iio_channel_attr_write_longlong(rx_lo, "frequency", FREQ);

    iio_channel *rx_cfg = iio_device_find_channel(phy, "voltage0", false);
    iio_channel_attr_write_longlong(rx_cfg, "sampling_frequency", RATE);
    
    // Həssaslıq ayarı (Ağıllı AGC)
    iio_channel_attr_write(rx_cfg, "gain_control_mode", "slow_attack"); 

    iio_channel *ri = iio_device_find_channel(rx_dev, "voltage0", false);
    iio_channel *rq = iio_device_find_channel(rx_dev, "voltage1", false);
    iio_channel_enable(ri); 
    iio_channel_enable(rq);

    // 3. Bufer və Sinxronizasiya
    iio_buffer *rx_buf = iio_device_create_buffer(rx_dev, 32768, false);
    RxState state;
    flexframesync fs = flexframesync_create(rx_callback, &state);
    std::vector<liquid_float_complex> l_buf(32768);

    std::cout << "--- QƏBULEDİCİ AKTİVDİR ---\n";
    std::cout << "Tezlik: " << FREQ/1e6 << " MHz | Sürət: " << RATE/1e6 << " Msps\n";
    std::cout << "Görüntü üçün: feh --reload 0.1 /tmp/rx_latest.jpg\n\n";

    

    // 4. Əsas Qəbul Döngüsü
    while (true) {
        ssize_t nbytes = iio_buffer_refill(rx_buf);
        if (nbytes < 0) break;

        char *rp = (char*)iio_buffer_start(rx_buf);
        ptrdiff_t rs = iio_buffer_step(rx_buf);

        for(int i=0; i<32768; i++) {
            int16_t *v = (int16_t*)rp;
            // Siqnalı Liquid-DSP üçün -1.0 ilə 1.0 arasına gətir
            l_buf[i] = {(float)v[0]/2048.0f, (float)v[1]/2048.0f};
            rp += rs;
        }
        flexframesync_execute(fs, l_buf.data(), 32768);
    }

    // Təmizlik
    flexframesync_destroy(fs);
    iio_buffer_destroy(rx_buf);
    iio_context_destroy(ctx);
    return 0;
}