#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iio.h>
#include <liquid/liquid.h>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define URI_DEV "usb:1.9.5"
#define CHUNK_SIZE 1024

static constexpr uint8_t CMD_VIDEO = 0xA1;
static constexpr uint8_t CMD_PHOTO = 0xB2;

static constexpr long long CMD_TX_FREQ   = 900000000;
static constexpr long long VIDEO_RX_FREQ = 915000000;
static constexpr long long SAMPLE_RATE   = 2083333;
static constexpr long long RF_BW         = 1500000;

struct FrameState {
    std::vector<uint8_t> frame_data;
    std::vector<uint8_t> got;
    uint32_t last_fid = 0;
    uint32_t chunks_received = 0;
    uint16_t total_chunks = 0;
};

static std::mutex g_mtx;
static std::string g_ready_file;
static std::atomic<bool> g_new_frame{false};
static std::atomic<bool> g_run{true};
static std::atomic<bool> g_first_cmd_sent{false};

static std::atomic<float> g_rssi{-999.0f};
static std::atomic<float> g_evm{0.0f};
static std::atomic<uint32_t> g_frame_id{0};
static std::atomic<uint32_t> g_chunk_ok{0};
static std::atomic<uint32_t> g_chunk_total{0};
static std::atomic<int> g_mode{0};

static void die(const char* msg) {
    std::perror(msg);
    std::exit(1);
}

static void DrawCircle(SDL_Renderer* renderer, int32_t cx, int32_t cy, int32_t radius) {
    const int32_t diameter = radius * 2;
    int32_t x = radius - 1;
    int32_t y = 0;
    int32_t tx = 1;
    int32_t ty = 1;
    int32_t error = tx - diameter;

    while (x >= y) {
        SDL_RenderDrawPoint(renderer, cx + x, cy - y);
        SDL_RenderDrawPoint(renderer, cx + x, cy + y);
        SDL_RenderDrawPoint(renderer, cx - x, cy - y);
        SDL_RenderDrawPoint(renderer, cx - x, cy + y);
        SDL_RenderDrawPoint(renderer, cx + y, cy - x);
        SDL_RenderDrawPoint(renderer, cx + y, cy + x);
        SDL_RenderDrawPoint(renderer, cx - y, cy - x);
        SDL_RenderDrawPoint(renderer, cx - y, cy + x);

        if (error <= 0) {
            ++y;
            error += ty;
            ty += 2;
        }
        if (error > 0) {
            --x;
            tx += 2;
            error += tx - diameter;
        }
    }
}

static void de_interleave(uint8_t* data, size_t len) {
    std::vector<uint8_t> temp(len);
    const int cols = 32;
    const int rows = (int)(len / cols);

    for (int c = 0; c < cols; ++c) {
        for (int r = 0; r < rows; ++r) {
            temp[r * cols + c] = data[c * rows + r];
        }
    }

    std::memcpy(data, temp.data(), len);
}

static std::vector<uint8_t> trim_jpeg(const std::vector<uint8_t>& in) {
    size_t soi = (size_t)-1;
    size_t eoi = (size_t)-1;

    for (size_t i = 1; i < in.size(); ++i) {
        if (soi == (size_t)-1 && in[i - 1] == 0xFF && in[i] == 0xD8) soi = i - 1;
        if (in[i - 1] == 0xFF && in[i] == 0xD9) eoi = i;
    }

    if (soi != (size_t)-1 && eoi != (size_t)-1 && eoi > soi) {
        return std::vector<uint8_t>(in.begin() + (long)soi, in.begin() + (long)eoi + 1);
    }
    return {};
}

static int rx_callback(unsigned char* hdr,
                       int h_valid,
                       unsigned char* pay,
                       unsigned int pay_len,
                       int p_valid,
                       framesyncstats_s stats,
                       void* userdata) {
    FrameState* st = (FrameState*)userdata;

    g_rssi = stats.rssi;
    g_evm = stats.evm;

    if (!p_valid || !h_valid || !hdr || !pay || !st) return 0;

    uint32_t fid = 0;
    uint16_t chk = 0;
    uint16_t total = 0;

    std::memcpy(&fid,   hdr + 0, 4);
    std::memcpy(&chk,   hdr + 4, 2);
    std::memcpy(&total, hdr + 6, 2);

    if (total == 0 || total > 10000) return 0;

    g_frame_id = fid;
    g_chunk_total = total;

    if (fid != st->last_fid) {
        std::printf("\n[YER] Yeni frame %u (%u paket)\n", fid, total);
        st->frame_data.assign((size_t)total * CHUNK_SIZE, 0xFF);
        st->got.assign(total, 0);
        st->last_fid = fid;
        st->chunks_received = 0;
        st->total_chunks = total;
    }

    if (chk < st->got.size() && !st->got[chk]) {
        std::vector<uint8_t> buf(CHUNK_SIZE, 0xFF);
        std::memcpy(buf.data(), pay, std::min<unsigned int>(pay_len, CHUNK_SIZE));

        for (size_t i = 0; i < CHUNK_SIZE; ++i) buf[i] ^= 0xAC;
        de_interleave(buf.data(), CHUNK_SIZE);

        std::memcpy(st->frame_data.data() + (size_t)chk * CHUNK_SIZE, buf.data(), CHUNK_SIZE);
        st->got[chk] = 1;
        st->chunks_received++;
        g_chunk_ok = st->chunks_received;

        std::printf("\r[YER] Paket %u/%u RSSI=%.1f EVM=%.2f   ",
                    st->chunks_received, total, stats.rssi, stats.evm);
        std::fflush(stdout);
    }

    if (st->chunks_received == total && total > 0) {
        auto jpg = trim_jpeg(st->frame_data);

        if (!jpg.empty()) {
            char name[128];
            std::snprintf(name, sizeof(name), "recv_%u.jpg", fid);

            FILE* f = std::fopen(name, "wb");
            if (f) {
                std::fwrite(jpg.data(), 1, jpg.size(), f);
                std::fclose(f);

                {
                    std::lock_guard<std::mutex> lk(g_mtx);
                    g_ready_file = name;
                    g_new_frame = true;
                }

                std::printf("\n[YER] Frame %u tam alindi, JPEG=%zu bayt\n", fid, jpg.size());
            }
        } else {
            std::printf("\n[YER] Frame %u tam geldi, amma JPEG tapilmadi\n", fid);
        }

        st->frame_data.clear();
        st->got.clear();
        st->chunks_received = 0;
        st->total_chunks = 0;
    }

    return 0;
}

static void rx_worker(iio_buffer* buf, flexframesync fs) {
    const size_t nsamps = 1 << 20;
    std::vector<liquid_float_complex> iq(nsamps);

    while (g_run) {
        const ssize_t nbytes = iio_buffer_refill(buf);
        if (nbytes < 0) continue;

        char* p = (char*)iio_buffer_start(buf);
        const ptrdiff_t step = iio_buffer_step(buf);
        if (!p || step <= 0) continue;

        size_t count = (size_t)(nbytes / step);
        if (count > nsamps) count = nsamps;

        for (size_t i = 0; i < count; ++i) {
            int16_t* s = (int16_t*)p;
            iq[i].real = (float)s[0] / 32768.0f;
            iq[i].imag = (float)s[1] / 32768.0f;
            p += step;
        }

        flexframesync_execute(fs, iq.data(), count);
    }
}

static void send_cmd(uint8_t code, iio_buffer* tx_buf, flexframegen fg) {
    if (!tx_buf || !fg) return;

    flexframegen_reset(fg);
    flexframegen_assemble(fg, nullptr, &code, 1);

    int16_t* p = (int16_t*)iio_buffer_start(tx_buf);
    const ptrdiff_t step = iio_buffer_step(tx_buf);
    if (!p || step <= 0) return;

    const size_t max_samps = 1 << 18;
    size_t written = 0;

    while (true) {
        liquid_float_complex s;
        int done = flexframegen_write_samples(fg, &s, 1);

        if (written < max_samps) {
            p[0] = (int16_t)(s.real * 15000.0f);
            p[1] = (int16_t)(s.imag * 15000.0f);
            p = (int16_t*)((char*)p + step);
            ++written;
        }

        if (done) break;
    }

    if (written < max_samps) {
        std::memset(p, 0, (max_samps - written) * step);
    }

    int ret = iio_buffer_push(tx_buf);
    std::printf("\n[YER] Komanda gonderildi: 0x%02X push=%d\n", code, ret);
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        return 1;
    }

    if (!(IMG_Init(IMG_INIT_JPG) & IMG_INIT_JPG)) {
        std::fprintf(stderr, "IMG_Init error: %s\n", IMG_GetError());
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow(
        "YER STANSIYASI",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1000, 700,
        SDL_WINDOW_SHOWN
    );
    if (!win) {
        std::fprintf(stderr, "Window error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_RaiseWindow(win);

    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if (!ren) {
        std::fprintf(stderr, "Renderer error: %s\n", SDL_GetError());
        return 1;
    }

    iio_context* ctx = iio_create_context_from_uri(URI_DEV);
    if (!ctx) die("iio_create_context_from_uri");

    iio_device* phy   = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* txdev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    iio_device* rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!phy || !txdev || !rxdev) {
        std::fprintf(stderr, "XETA: Pluto device tapilmadi\n");
        return 1;
    }

    iio_channel* tx_ctrl = iio_device_find_channel(phy, "voltage0", true);
    iio_channel* tx_lo   = iio_device_find_channel(phy, "altvoltage1", true);

    iio_channel* rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel* rx_lo   = iio_device_find_channel(phy, "altvoltage0", true);

    if (!tx_ctrl || !tx_lo || !rx_ctrl || !rx_lo) {
        std::fprintf(stderr, "XETA: ctrl/LO tapilmadi\n");
        return 1;
    }

    iio_channel_attr_write(tx_ctrl, "rf_port_select", "A");
    iio_channel_attr_write_longlong(tx_ctrl, "hardwaregain", -10);
    iio_channel_attr_write_longlong(tx_ctrl, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(tx_ctrl, "rf_bandwidth", RF_BW);
    iio_channel_attr_write_longlong(tx_lo, "frequency", CMD_TX_FREQ);

    iio_channel_attr_write(rx_ctrl, "rf_port_select", "B_BALANCED");
    iio_channel_attr_write(rx_ctrl, "gain_control_mode", "manual");
    iio_channel_attr_write_longlong(rx_ctrl, "hardwaregain", 60);
    iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(rx_ctrl, "rf_bandwidth", RF_BW);
    iio_channel_attr_write_longlong(rx_lo, "frequency", VIDEO_RX_FREQ);

    iio_channel* tx_i = iio_device_find_channel(txdev, "voltage0", true);
    iio_channel* tx_q = iio_device_find_channel(txdev, "voltage1", true);

    iio_channel* rx_i = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel* rx_q = iio_device_find_channel(rxdev, "voltage1", false);

    if (!tx_i || !tx_q || !rx_i || !rx_q) {
        std::fprintf(stderr, "XETA: IQ stream channel tapilmadi\n");
        return 1;
    }

    iio_channel_enable(tx_i);
    iio_channel_enable(tx_q);
    iio_channel_enable(rx_i);
    iio_channel_enable(rx_q);

    iio_buffer* rx_buf = iio_device_create_buffer(rxdev, 1 << 20, false);
    iio_buffer* tx_buf = iio_device_create_buffer(txdev, 1 << 18, false);
    if (!rx_buf || !tx_buf) {
        std::fprintf(stderr, "XETA: RX/TX buffer yaradilmaadi\n");
        return 1;
    }

    FrameState* st = new FrameState();

    flexframesync fs = flexframesync_create(rx_callback, st);
    flexframegen fg_cmd = flexframegen_create(nullptr);
    if (!fs || !fg_cmd) {
        std::fprintf(stderr, "XETA: liquid obyektleri yaradilmaadi\n");
        return 1;
    }

    std::thread t(rx_worker, rx_buf, fs);
    SDL_Texture* tex = nullptr;

    Uint32 last_cmd_tick = 0;

    std::printf("[YER] Basladi | TX(A)=900MHz komanda | RX(B)=915MHz video\n");
    std::printf("[YER] V=VIDEO, P=FOTO, ESC=CIXIS\n");

    while (g_run) {
        if (!g_first_cmd_sent.load()) {
            send_cmd(CMD_VIDEO, tx_buf, fg_cmd);
            g_mode = 0;
            g_first_cmd_sent = true;
            last_cmd_tick = SDL_GetTicks();
            std::printf("[YER] Baslangic ucun VIDEO komandasi avtomatik gonderildi\n");
        }

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) g_run = false;

            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) {
                    g_run = false;
                } else if (e.key.keysym.sym == SDLK_v) {
                    g_mode = 0;
                    send_cmd(CMD_VIDEO, tx_buf, fg_cmd);
                    last_cmd_tick = SDL_GetTicks();
                } else if (e.key.keysym.sym == SDLK_p) {
                    g_mode = 1;
                    send_cmd(CMD_PHOTO, tx_buf, fg_cmd);
                    last_cmd_tick = SDL_GetTicks();
                }
            }
        }

        Uint32 now = SDL_GetTicks();
        if (now - last_cmd_tick > 1000) {
            if (g_mode.load() == 0) send_cmd(CMD_VIDEO, tx_buf, fg_cmd);
            else send_cmd(CMD_PHOTO, tx_buf, fg_cmd);
            last_cmd_tick = now;
        }

        if (g_new_frame) {
            std::string file;
            {
                std::lock_guard<std::mutex> lk(g_mtx);
                file = g_ready_file;
                g_new_frame = false;
            }

            if (!file.empty()) {
                SDL_Texture* nt = IMG_LoadTexture(ren, file.c_str());
                if (nt) {
                    if (tex) SDL_DestroyTexture(tex);
                    tex = nt;
                } else {
                    std::fprintf(stderr, "\n[YER] JPEG decode error: %s\n", IMG_GetError());
                }
            }
        }

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        int win_w = 0, win_h = 0;
        SDL_GetWindowSize(win, &win_w, &win_h);

        if (tex) {
            int tw = 0, th = 0;
            SDL_QueryTexture(tex, nullptr, nullptr, &tw, &th);

            SDL_Rect dst{};
            if (tw > 0 && th > 0) {
                float sx = (float)win_w / tw;
                float sy = (float)win_h / th;
                float s = std::min(sx, sy);

                dst.w = (int)(tw * s);
                dst.h = (int)(th * s);
                dst.x = (win_w - dst.w) / 2;
                dst.y = (win_h - dst.h) / 2;

                SDL_RenderCopy(ren, tex, nullptr, &dst);
            }
        } else {
            SDL_Rect box{win_w / 2 - 150, win_h / 2 - 100, 300, 200};
            SDL_SetRenderDrawColor(ren, 0, 255, 0, 255);
            SDL_RenderDrawRect(ren, &box);
        }

        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

        SDL_SetRenderDrawColor(ren, 0, 255, 0, 30);
        for (int i = 0; i < win_w; i += 100) SDL_RenderDrawLine(ren, i, 0, i, win_h);
        for (int i = 0; i < win_h; i += 100) SDL_RenderDrawLine(ren, 0, i, win_w, i);

        SDL_SetRenderDrawColor(ren, 0, 255, 0, 150);
        SDL_RenderDrawLine(ren, win_w / 2, 0, win_w / 2, win_h);
        SDL_RenderDrawLine(ren, 0, win_h / 2, win_w, win_h / 2);
        SDL_Rect center_box = { win_w / 2 - 40, win_h / 2 - 40, 80, 80 };
        SDL_RenderDrawRect(ren, &center_box);

        Uint32 ticks = SDL_GetTicks();
        int scan_y = (ticks / 2) % std::max(1, win_h);
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 100);
        SDL_RenderDrawLine(ren, 0, scan_y, win_w, scan_y);

        char title[512];
        std::snprintf(title, sizeof(title),
            "[YER HUD] MODE:%s | Frame:%u | Paket:%u/%u | RSSI:%.1f | EVM:%.2f",
            g_mode.load() == 0 ? "VIDEO" : "FOTO",
            g_frame_id.load(),
            g_chunk_ok.load(),
            g_chunk_total.load(),
            g_rssi.load(),
            g_evm.load());
        SDL_SetWindowTitle(win, title);

        SDL_RenderPresent(ren);
        SDL_Delay(10);
    }

    g_run = false;
    if (t.joinable()) t.join();

    if (tex) SDL_DestroyTexture(tex);
    flexframegen_destroy(fg_cmd);
    flexframesync_destroy(fs);
    iio_buffer_destroy(tx_buf);
    iio_buffer_destroy(rx_buf);
    iio_context_destroy(ctx);
    delete st;

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    IMG_Quit();
    SDL_Quit();
    return 0;
}