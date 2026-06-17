#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iio.h>
#include <liquid/liquid.h>

#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <mutex>
#include <thread>
#include <cmath>
#include <atomic>
#include <cstdlib>

#define CHUNK_SIZE 1024
#define URI_RX "usb:1.12.5"

struct State {
    std::vector<uint8_t> frame_data;
    std::vector<uint8_t> got;
    uint32_t last_fid = 0;
    uint32_t chunks_received = 0;
    uint32_t total_chunks = 0;
};

static std::mutex g_mtx;
static std::string g_ready_file;
static std::atomic<bool> g_new_frame{false};
static std::atomic<bool> g_run{true};

static std::atomic<float> g_rssi{-999.0f};
static std::atomic<float> g_evm{0.0f};
static std::atomic<uint32_t> g_frame_id{0};
static std::atomic<uint32_t> g_chunk_ok{0};
static std::atomic<uint32_t> g_chunk_total{0};

static void die(const char* msg) {
    std::perror(msg);
    std::exit(1);
}

// Dairə çəkmək üçün köməkçi funksiya (Radar üçün)
void DrawCircle(SDL_Renderer * renderer, int32_t centreX, int32_t centreY, int32_t radius) {
   const int32_t diameter = (radius * 2);
   int32_t x = (radius - 1);
   int32_t y = 0;
   int32_t tx = 1;
   int32_t ty = 1;
   int32_t error = (tx - diameter);
   while (x >= y) {
      SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
      SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
      SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
      SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
      SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
      SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
      SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
      SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);
      if (error <= 0) {
         ++y;
         error += ty;
         ty += 2;
      }
      if (error > 0) {
         --x;
         tx += 2;
         error += (tx - diameter);
      }
   }
}

void de_interleave(uint8_t* data, size_t len) {
    std::vector<uint8_t> temp(len);
    int cols = 32;
    int rows = len / cols;
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            temp[r * cols + c] = data[c * rows + r];
        }
    }
    std::memcpy(data, temp.data(), len);
}

static std::vector<uint8_t> trim_jpeg_end(const std::vector<uint8_t>& in) {
    size_t soi = (size_t)-1;
    size_t eoi = (size_t)-1;

    for (size_t i = 1; i < in.size(); ++i) {
        if (soi == (size_t)-1 && in[i - 1] == 0xFF && in[i] == 0xD8) {
            soi = i - 1;
        }
        if (in[i - 1] == 0xFF && in[i] == 0xD9) {
            eoi = i;
        }
    }

    if (soi != (size_t)-1 && eoi != (size_t)-1 && eoi > soi) {
        return std::vector<uint8_t>(in.begin() + soi, in.begin() + eoi + 1);
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
    State* st = (State*)userdata;

    g_rssi = stats.rssi;
    g_evm = stats.evm;

    if (!p_valid || !h_valid || !hdr || !pay) return 0;

    uint32_t fid = 0, chk_idx = 0, total = 0;
    std::memcpy(&fid, hdr, 4);
    std::memcpy(&chk_idx, hdr + 4, 4);
    std::memcpy(&total, hdr + 8, 4);

    g_frame_id = fid;
    g_chunk_total = total;

    if (total == 0 || total > 10000) return 0;

    // Yeni frame gələndə yalnız fid dəyişibsə reset et
    if (fid != st->last_fid) {
        std::printf("\n[*] Qəbul: Frame %u (%u paket)\n", fid, total);
        st->frame_data.assign((size_t)total * CHUNK_SIZE, 0xFF);
        st->got.assign(total, 0);
        st->last_fid = fid;
        st->chunks_received = 0;
        st->total_chunks = total;
    }

    if (chk_idx < st->got.size() && !st->got[chk_idx]) {
        std::vector<uint8_t> buf(CHUNK_SIZE, 0xFF);
        std::memcpy(buf.data(), pay, std::min<unsigned int>(pay_len, CHUNK_SIZE));

        for (int b = 0; b < CHUNK_SIZE; b++) buf[b] ^= 0xAC;
        de_interleave(buf.data(), CHUNK_SIZE);

        std::memcpy(st->frame_data.data() + (size_t)chk_idx * CHUNK_SIZE, buf.data(), CHUNK_SIZE);
        st->got[chk_idx] = 1;
        st->chunks_received++;
        g_chunk_ok = st->chunks_received;

        std::printf("\r    Paket %u/%u tutuldu... RSSI: %.1f dB",
                    st->chunks_received, total, stats.rssi);
        std::fflush(stdout);
    }

    // GUI üçün yalnız tam frame
    if (st->chunks_received == total && total > 0) {
        auto jpg = trim_jpeg_end(st->frame_data);

        if (!jpg.empty()) {
            char name[64];
            std::sprintf(name, "nasa_output_%u.jpg", fid);

            FILE* f = std::fopen(name, "wb");
            if (f) {
                std::fwrite(jpg.data(), 1, jpg.size(), f);
                std::fclose(f);

                {
                    std::lock_guard<std::mutex> lk(g_mtx);
                    g_ready_file = name;
                    g_new_frame = true;
                }

                std::printf("\n[OK] Frame %u TAM ALINDI! JPEG=%zu bayt\n", fid, jpg.size());
            }
        } else {
            std::printf("\n[WARN] Frame %u tam gəldi, amma JPEG sonu tapılmadı\n", fid);
        }

        st->frame_data.clear();
        st->got.clear();
        st->chunks_received = 0;
        st->total_chunks = 0;
    }

    return 0;
}

static void radio_worker(iio_buffer* buf, flexframesync fs) {
    const size_t nsamps = 1 << 20;
    std::vector<liquid_float_complex> iq(nsamps);

    while (g_run) {
        if (iio_buffer_refill(buf) < 0) continue;

        char* p = (char*)iio_buffer_start(buf);
        ptrdiff_t step = iio_buffer_step(buf);

        for (size_t i = 0; i < nsamps; i++) {
            int16_t* s = (int16_t*)p;
            iq[i].real = (float)s[0] / 32768.0f;
            iq[i].imag = (float)s[1] / 32768.0f;
            p += step;
        }

        flexframesync_execute(fs, iq.data(), nsamps);
    }
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        return 1;
    }

    if (!(IMG_Init(IMG_INIT_JPG) & IMG_INIT_JPG)) {
        std::fprintf(stderr, "IMG_Init error: %s\n", IMG_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow(
        "NASA RX GUI",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1000, 700,
        SDL_WINDOW_SHOWN
    );

    SDL_Renderer* ren = SDL_CreateRenderer(
        win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!win || !ren) {
        std::fprintf(stderr, "SDL window/renderer error\n");
        return 1;
    }

    iio_context* ctx = iio_create_context_from_uri(URI_RX);
    if (!ctx) die("iio_create_context_from_uri");

    iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    iio_device* rxdev = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if (!phy || !rxdev) die("RX device not found");

    iio_channel* rx_ctrl = iio_device_find_channel(phy, "voltage0", false);
    iio_channel* rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    if (!rx_ctrl || !rx_lo) die("RX control channel not found");

    iio_channel_attr_write_longlong(rx_lo, "frequency", 915000000);
    iio_channel_attr_write_longlong(rx_ctrl, "sampling_frequency", 1000000);
    iio_channel_attr_write_longlong(rx_ctrl, "hardwaregain", 55);

    iio_channel* ri = iio_device_find_channel(rxdev, "voltage0", false);
    iio_channel* rq = iio_device_find_channel(rxdev, "voltage1", false);
    if (!ri || !rq) die("RX IQ channels not found");

    iio_channel_enable(ri);
    iio_channel_enable(rq);

    const size_t nsamps = 1 << 20;
    iio_buffer* buf = iio_device_create_buffer(rxdev, nsamps, false);
    if (!buf) die("iio_device_create_buffer");

    State* st = new State();
    flexframesync fs = flexframesync_create(rx_callback, st);
    if (!fs) die("flexframesync_create");

    std::thread t(radio_worker, buf, fs);

    SDL_Texture* tex = nullptr;

    std::printf("NASA RX GUI Dinləyir...\n");

    while (g_run) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) g_run = false;
        }

        if (g_new_frame) {
            std::string local_file;
            {
                std::lock_guard<std::mutex> lk(g_mtx);
                local_file = g_ready_file;
                g_new_frame = false;
            }

            if (!local_file.empty()) {
                SDL_Texture* nt = IMG_LoadTexture(ren, local_file.c_str());
                if (nt) {
                    if (tex) SDL_DestroyTexture(tex);
                    tex = nt;
                } else {
                    std::fprintf(stderr, "\nJPEG decode error: %s\n", IMG_GetError());
                }
            }
        }

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        int win_w, win_h;
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
            }

            SDL_RenderCopy(ren, tex, nullptr, &dst);
        }

        // HUD / Overlay Qrafikası
        SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

        // 1. Tor (Grid) və Nişangah (Crosshair)
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 30);
        for (int i = 0; i < win_w; i += 100) SDL_RenderDrawLine(ren, i, 0, i, win_h);
        for (int i = 0; i < win_h; i += 100) SDL_RenderDrawLine(ren, 0, i, win_w, i);

        SDL_SetRenderDrawColor(ren, 0, 255, 0, 150);
        SDL_RenderDrawLine(ren, win_w / 2, 0, win_w / 2, win_h);
        SDL_RenderDrawLine(ren, 0, win_h / 2, win_w, win_h / 2);
        SDL_Rect center_box = { win_w / 2 - 40, win_h / 2 - 40, 80, 80 };
        SDL_RenderDrawRect(ren, &center_box);

        // 2. Skanlama xətti (Scanline effekti)
        Uint32 ticks = SDL_GetTicks();
        int scan_y = (ticks / 2) % win_h;
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 100);
        SDL_RenderDrawLine(ren, 0, scan_y, win_w, scan_y);

        // 3. Siqnal gücü (RSSI) barları
        float rssi_val = g_rssi.load();
        int rssi_level = 0;
        if (rssi_val > -40) rssi_level = 5;
        else if (rssi_val > -55) rssi_level = 4;
        else if (rssi_val > -70) rssi_level = 3;
        else if (rssi_val > -85) rssi_level = 2;
        else if (rssi_val > -95) rssi_level = 1;

        for (int i = 0; i < 5; i++) {
            if (i < rssi_level) SDL_SetRenderDrawColor(ren, 0, 255, 0, 200);
            else SDL_SetRenderDrawColor(ren, 50, 50, 50, 150);
            SDL_Rect bar = { 30 + i * 15, win_h - 30 - (i + 1) * 15, 10, (i + 1) * 15 };
            SDL_RenderFillRect(ren, &bar);
        }

        // 4. Radar və Təxmini Məsafə Simulyasiyası
        // Qeyd: Əsl GPS olmadığı üçün məsafəni RSSI-yə əsasən simulyasiya edirik
        float dist_m = std::pow(10.0f, (-30.0f - rssi_val) / 20.0f);
        if (dist_m > 10000) dist_m = 10000;
        if (dist_m < 0) dist_m = 0;

        int radar_x = win_w - 100;
        int radar_y = win_h - 100;
        SDL_SetRenderDrawColor(ren, 0, 255, 0, 150);
        DrawCircle(ren, radar_x, radar_y, 60);
        DrawCircle(ren, radar_x, radar_y, 40);
        DrawCircle(ren, radar_x, radar_y, 20);
        SDL_RenderDrawLine(ren, radar_x - 60, radar_y, radar_x + 60, radar_y);
        SDL_RenderDrawLine(ren, radar_x, radar_y - 60, radar_x, radar_y + 60);

        // Radarda Hədəf (Blinking effekti)
        if ((ticks / 500) % 2 == 0) {
            float angle = ticks / 1000.0f; // Hədəfin fırlanma effekti
            int target_r = (int)((dist_m / 10000.0f) * 60.0f);
            if (target_r > 60) target_r = 60;
            int tx = radar_x + (int)(target_r * std::cos(angle));
            int ty = radar_y + (int)(target_r * std::sin(angle));

            SDL_SetRenderDrawColor(ren, 255, 50, 50, 255);
            SDL_Rect target = { tx - 4, ty - 4, 8, 8 };
            SDL_RenderFillRect(ren, &target);
        }

        // Pəncərə başlığında Telemetriya Məlumatları
        char title[512];
        float evm_val = g_evm.load();
        
        // Koordinatları vizual effekt üçün məsafəyə əsasən simulyasiya edirik
        float coord_x = dist_m * 0.45f * std::cos(ticks / 5000.0f);
        float coord_y = dist_m * 0.89f * std::sin(ticks / 5000.0f);
        float coord_z = 400.0f + (rssi_val * 2.0f);

        std::snprintf(title, sizeof(title),
            "[NASA HUD] Frame:%u | Paket:%u/%u | RSSI:%.1f dBm | Mesafe:~%.1f m | POS: X:%.1f Y:%.1f Z:%.1f",
            g_frame_id.load(), g_chunk_ok.load(), g_chunk_total.load(),
            rssi_val, dist_m, coord_x, coord_y, coord_z);
        SDL_SetWindowTitle(win, title);

        SDL_RenderPresent(ren);
        SDL_Delay(10);
    }

    g_run = false;
    if (t.joinable()) t.join();

    if (tex) SDL_DestroyTexture(tex);
    flexframesync_destroy(fs);
    iio_buffer_destroy(buf);
    iio_context_destroy(ctx);
    delete st;

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    IMG_Quit();
    SDL_Quit();
    return 0;
}