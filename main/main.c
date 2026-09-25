// RF Scanner 433 MHz - ESP32-C6-LCD-1.47 + XL4432-SMT (Si4432)
// Landscape 320x172. Moduri (buton BOOT comuta):
//  1 GO/NOGO   : "emite?" - EMITE/NIMIC + nivel
//  2 FRECVENTA : mini-spectru + frecventa varfului
//  3 IDENTIFICARE : modulatie (OOK/FSK) + TIP (TELECOMANDA/SENZOR/FM) + bitrate
//  4 TX TEST   : beacon MCW (Morse) pe TX_FREQ_KHZ, in afara benzii LPD 433.05-434.79
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "si4432.h"
#include "display.h"

// ---------- Pini ----------
#define PIN_MOSI   6      // LCD (SPI hardware)
#define PIN_MISO   5
#define PIN_SCLK   7
#define PIN_LCD_DC 15
#define PIN_LCD_CS 14
#define PIN_LCD_RST 21
#define PIN_LCD_BL 22
#define PIN_SD_CS  4
// Si4432 pe SPI SOFTWARE (NON-USB: 12/13=USB, 16/17=consola, 4/5/8/9/15=strapping)
#define PIN_SI_SCLK 0
#define PIN_SI_MOSI 1
#define PIN_SI_MISO 23
#define PIN_SI_CS   18
#define PIN_SI_SDN  19
#define PIN_SI_IRQ  20
#define PIN_SI_RXD  2
#define PIN_LED    8
#define PIN_BTN    9

// ---------- Banda ----------
#define BAND_START_KHZ 433050u
#define BAND_STEP_KHZ  100u
#define BAND_NCH       18
#define RSSI_MARGIN    28     // ~14 dB peste mediana; zgomotul ajunge la ~+23
// Intre baleiaje receptorul sta "parcat" cu filtru larg si intrerupere de prag RSSI,
// ca sa prinda si rafalele scurte care cad cat timp se deseneaza ecranul.
// Centru 434.15 + filtru ~620 kHz => acopera 433.84-434.46 (433.92 si 434.42).
#define PARK_CENTER_KHZ 434150u
#define PARK_BW_KHZ     620
// Marja intreruperii fata de fondul parcarii. Masurat pe placa (26 sep): cu filtrul
// larg zgomotul are varfuri mari; +24 declansa 16-19 din 20 cicluri, +36 1-5, +42
// rar, +48 niciodata. Semnalele slabe raman acoperite de baleiaj (marja RSSI_MARGIN).
#define PARK_MARGIN     48
#define EVENT_HOLD_US   300000    // pauze mai scurte de atat = aceeasi apasare

// Beacon TX: banda de radioamatori 70 cm, in afara benzii LPD a cheilor/portilor.
#define TX_FREQ_KHZ    432500u
#define TX_POWER       6          // 0..7 => 1,2,5,8,11,14,17,20 dBm
static const int8_t TX_DBM[8] = { 1, 2, 5, 8, 11, 14, 17, 20 };

typedef enum { MODE_GONOGO=0, MODE_FREQ, MODE_IDENTIFY, MODE_TXTEST, MODE_COUNT } mode_t;
static const char *MODE_NAME[] = { "GO/NOGO", "FRECVENTA", "IDENTIFICARE", "TX TEST CW" };

// ---------------- WS2812 ----------------
static rmt_channel_handle_t s_led_chan;
static rmt_encoder_handle_t s_led_enc;
static void led_init(void)
{
    rmt_tx_channel_config_t tx = {
        .gpio_num = PIN_LED, .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10*1000*1000, .mem_block_symbols = 64, .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx, &s_led_chan));
    rmt_bytes_encoder_config_t bc = {
        .bit0 = { .level0=1,.duration0=3,.level1=0,.duration1=9 },
        .bit1 = { .level0=1,.duration0=9,.level1=0,.duration1=3 },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bc, &s_led_enc));
    ESP_ERROR_CHECK(rmt_enable(s_led_chan));
}
static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t t = { .loop_count = 0 };
    rmt_transmit(s_led_chan, s_led_enc, grb, 3, &t);
    rmt_tx_wait_all_done(s_led_chan, 50);
}

// ---------------- Baleiaj RSSI ----------------
static uint8_t noise_floor = 60;
static void scan_band(uint8_t *rssi, int nch)
{
    si4432_set_channel(0);
    si4432_set_freq_khz(BAND_START_KHZ);
    si4432_set_hop_step_khz(BAND_STEP_KHZ);
    si4432_config_scan(150);                 // readuce filtrul IF dupa parcare
    vTaskDelay(pdMS_TO_TICKS(3));            // asezare AGC dupa reconfigurare
    for (int ch = 0; ch < nch; ch++) {
        si4432_set_channel(ch);
        esp_rom_delay_us(400);
        rssi[ch] = si4432_rssi_raw();
    }
}

// Parcheaza receptorul pe filtru larg si armeaza intreruperea de prag RSSI.
// Pragul urmareste zgomotul de fond al filtrului larg (alt nivel decat la baleiaj).
static uint8_t park_floor = 0;
static void park_and_arm(void)
{
    si4432_set_channel(0);
    si4432_set_freq_khz(PARK_CENTER_KHZ);
    si4432_config_scan(PARK_BW_KHZ);
    esp_rom_delay_us(500);
    uint8_t r = si4432_rssi_raw();
    if (park_floor == 0) park_floor = r;
    else park_floor = (park_floor * 15 + r) / 16;
    int thr = park_floor + PARK_MARGIN;
    si4432_rssi_irq_arm(thr > 255 ? 255 : (uint8_t)thr);
}

// ---------------- Frecventa exacta (baleiaj fin, margini la -6 dB) ----------------
// AFC-ul Si4432 ramane pe 0 fara pachet recunoscut (verificat pe placa 26 sep), deci
// frecventa se masoara din spectru. Filtrul IF de 75 kHz: cele mai inguste (19 kHz)
// lasa receptorul surd (verificat pe placa). Cu 75 kHz raspunsul la un semnal ingust
// e un platou cat filtrul, iar centrul lui = frecventa semnalului; se ia mijlocul
// dintre marginile la -6 dB (interpolate), robust la forma platoului. Max-hold
// 400 ms ca rafalele intermitente sa umple toate punctele.
#define FINE_SPAN_KHZ  160
#define FINE_STEP_KHZ  5
#define FINE_N         (2 * FINE_SPAN_KHZ / FINE_STEP_KHZ + 1)
#define HOP_N          ((FINE_N + 1) / 2)    // canale de 10 kHz pe trecere
#define FINE_BW_KHZ    75
// Corectie de calibrare (eroarea cuartului Si4432, ~25 ppm, + a metodei). Masurata
// 26 sep cu statia portabila pe 433.920 (purtatoare FM fara modulatie): 8 masuratori
// 433.9293-433.9313 MHz, mediana 433.9308 => -10.8 kHz.
#define FREQ_CAL_HZ    (-10800)

// Frecventa se schimba prin registrul de canal (hopping), ca la baleiajul normal:
// scrierea directa a registrelor de frecventa in RX nu reacorda PLL-ul (verificat
// pe placa). Doua treceri cu baza decalata 5 kHz, pas de hopping 10 kHz.
static void fine_spectrum(uint32_t f0, uint32_t bw_khz, uint8_t *mh)
{
    memset(mh, 0, FINE_N);
    si4432_set_hop_step_khz(10);
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 400000) {
        for (int pass = 0; pass < 2; pass++) {
            si4432_set_channel(0);
            si4432_set_freq_khz(f0 + pass * FINE_STEP_KHZ);
            si4432_config_scan(bw_khz);      // idle -> RX: PLL pe baza noua
            esp_rom_delay_us(500);
            for (int ch = 0; ch < HOP_N; ch++) {
                int i = 2 * ch + pass;
                if (i >= FINE_N) break;
                si4432_set_channel(ch);
                esp_rom_delay_us(300);
                uint8_t v = si4432_rssi_raw();
                if (v > mh[i]) mh[i] = v;
            }
        }
    }
}

static bool measure_freq_hz(uint32_t peak_khz, uint32_t *f_hz)
{
    uint8_t mh[FINE_N];
    uint32_t f0 = peak_khz - FINE_SPAN_KHZ;
    fine_spectrum(f0, FINE_BW_KHZ, mh);
    uint8_t rmax = 0, rmin = 255;
    for (int i = 0; i < FINE_N; i++) {
        if (mh[i] > rmax) rmax = mh[i];
        if (mh[i] < rmin) rmin = mh[i];
    }
    if (rmax < rmin + 16) return false;      // sub 8 dB peste fond: semnal pierdut
    int L = rmax - 12;                       // -6 dB (RSSI = 0.5 dB/unitate)
    int il = 0, ir = FINE_N - 1;
    while (il < FINE_N && mh[il] < L) il++;
    while (ir >= 0 && mh[ir] < L) ir--;
    if (il == 0 || ir == FINE_N - 1) return false;   // platoul atinge marginea ferestrei
    // interpolare liniara intre punctul de sub prag si cel de peste
    double fl = f0 + FINE_STEP_KHZ * (il - 1 + (double)(L - mh[il - 1]) / (mh[il] - mh[il - 1]));
    double fr = f0 + FINE_STEP_KHZ * (ir + (double)(mh[ir] - L) / (mh[ir] - mh[ir + 1]));
    // un semnal real da un platou cel putin cat filtrul (~60-90 kHz masurat);
    // mai ingust = varf de zgomot
    if (fr - fl < 40) return false;
    *f_hz = (uint32_t)lround((fl + fr) / 2 * 1000.0) + FREQ_CAL_HZ;
    ESP_LOGI("freq", "margini -6dB %.1f / %.1f kHz, latime %.1f kHz, centru %lu Hz",
             fl, fr, fr - fl, (unsigned long)*f_hz);
    return true;
}

// Frecventa nominala cea mai apropiata (telecomenzi EU) si abaterea fata de ea
static const uint32_t NOMINAL_KHZ[] = { 433920, 434420 };
static uint32_t nearest_nominal(uint32_t f_hz)
{
    uint32_t best = NOMINAL_KHZ[0];
    for (size_t i = 1; i < sizeof(NOMINAL_KHZ) / sizeof(NOMINAL_KHZ[0]); i++)
        if (llabs((int64_t)f_hz - NOMINAL_KHZ[i] * 1000LL) < llabs((int64_t)f_hz - best * 1000LL))
            best = NOMINAL_KHZ[i];
    return best;
}

// "433.918 MHZ" si "ABATERE -2.4 KHZ (433.920)"
static void fmt_freq(char *out, size_t n, uint32_t f_hz)
{
    uint32_t khz = (f_hz + 500) / 1000;
    snprintf(out, n, "%lu.%03lu MHZ", (unsigned long)(khz / 1000), (unsigned long)(khz % 1000));
}
static void fmt_dev(char *out, size_t n, uint32_t f_hz)
{
    uint32_t nom = nearest_nominal(f_hz);
    int32_t d = (int32_t)((int64_t)f_hz - nom * 1000LL);
    int32_t a = d < 0 ? -d : d;
    if (a > 999999) a = 999999;              // max +-120 kHz din baleiaj; limita pt. snprintf
    snprintf(out, n, "ABATERE %s%d.%d KHZ (%u.%03u)", d < 0 ? "-" : "",
             (int)(a / 1000), (int)((a % 1000) / 100),
             (unsigned)(nom / 1000 % 1000), (unsigned)(nom % 1000));
}

// ---------------- Identificare (modulatie + tip) ----------------
typedef struct { const char *mod; const char *type; int bitrate; uint32_t f_hz; } ident_t;

static ident_t identify(uint32_t peak_khz)
{
    ident_t r = { "?", "-", 0, 0 };

    // (1) modulatie prin fronturile liniei brute OOK, ~50 ms
    si4432_config_ook_raw();
    si4432_set_channel(0);                   // dupa baleiaj canalul ramane pe ultimul
    si4432_set_freq_khz(peak_khz);
    si4432_enter_rx();
    vTaskDelay(pdMS_TO_TICKS(2));
    int last = si4432_rxdata_level(), edges = 0;
    int64_t t0 = esp_timer_get_time(), le = t0, minp = 1000000;
    while (esp_timer_get_time() - t0 < 50000) {
        int lv = si4432_rxdata_level();
        if (lv != last) {
            int64_t n = esp_timer_get_time(), w = n - le;
            if (w > 60 && w < minp) minp = w;
            le = n; last = lv; edges++;
        }
    }
    bool ook = (edges > 20 && minp < 3000);

    // (2) frecventa exacta, cat timp telecomanda inca emite
    if (!measure_freq_hz(peak_khz, &r.f_hz)) r.f_hz = 0;

    // (3) tipul semnalului: monitorizeaza ~1.2 s la frecventa varfului
    si4432_config_scan(150);
    si4432_set_freq_khz(peak_khz);
    si4432_set_hop_step_khz(100);
    si4432_set_channel(0);
    si4432_enter_rx();
    int N = 200, activeCnt = 0, bursts = 0; bool prevA = false;
    uint8_t thr = noise_floor + RSSI_MARGIN;
    for (int i = 0; i < N; i++) {
        uint8_t v = si4432_rssi_raw();
        bool a = (v > thr);
        if (a) activeCnt++;
        if (a && !prevA) bursts++;
        prevA = a;
        vTaskDelay(pdMS_TO_TICKS(6));    // ~1.2 s total, cedeaza CPU
    }
    float ratio = (float)activeCnt / N;
    if (ratio > 0.85f)                    r.type = "FM/CONTINUU";
    else if (bursts >= 3 || ratio > 0.30f) r.type = "TELECOMANDA";
    else if (activeCnt > 0)               r.type = "SENZOR";
    else                                  r.type = "-";

    // Modulatia: OOK se vede in timp (amplitudinea pulseaza). Altfel anvelopa e
    // constanta => FSK daca vine in rafale, purtatoare FM daca e continuu. In
    // frecventa nu se poate deosebi: cu filtrul minim utilizabil (75 kHz) deviatia
    // cheilor nu lateste spectrul (verificat pe placa: cheia Audi ~70 kHz, purtatoarea
    // statiei ~90 kHz), iar filtrele mai inguste lasa receptorul surd.
    if (ook) { r.mod = "OOK/ASK"; r.bitrate = minp > 0 ? (int)(1000000 / minp) : 0; }
    else if (activeCnt == 0)          r.mod = "?";     // semnalul a disparut
    else if (ratio > 0.85f)           r.mod = "FM/PURTATOARE";
    else                              r.mod = "FSK/GFSK";
    return r;
}

// ---------------- TX beacon MCW (ton audibil 800 Hz) "TEST" ----------------
// Pulseaza purtatoarea la 800 Hz in timpul fiecarui element Morse, ca receptorul
// FM sa auda un "biip" clar (nu doar rupere de squelch ca la CW pur).
static void mcw_tone(int dur_ms)
{
    // Cheiere on/off la 800 Hz (varianta care emite sigur si se aude pe FM).
    int64_t end = esp_timer_get_time() + (int64_t)dur_ms * 1000;
    while (esp_timer_get_time() < end) {
        si4432_tx_key(true);  esp_rom_delay_us(625);   // 800 Hz => semiperioada 625 us
        si4432_tx_key(false); esp_rom_delay_us(625);
    }
    si4432_tx_key(false);
}

#define TX_MSG "TEST YO8RYG"
#define TX_UNIT 140                       // ms / unitate Morse

static const char *morse_of(char c)
{
    switch (c) {
        case 'A': return ".-";    case 'B': return "-...";  case 'C': return "-.-.";
        case 'D': return "-..";   case 'E': return ".";     case 'F': return "..-.";
        case 'G': return "--.";   case 'H': return "....";  case 'I': return "..";
        case 'J': return ".---";  case 'K': return "-.-";   case 'L': return ".-..";
        case 'M': return "--";    case 'N': return "-.";    case 'O': return "---";
        case 'P': return ".--.";  case 'Q': return "--.-";  case 'R': return ".-.";
        case 'S': return "...";   case 'T': return "-";     case 'U': return "..-";
        case 'V': return "...-";  case 'W': return ".--";   case 'X': return "-..-";
        case 'Y': return "-.--";  case 'Z': return "--..";
        case '0': return "-----"; case '1': return ".----"; case '2': return "..---";
        case '3': return "...--"; case '4': return "....-"; case '5': return ".....";
        case '6': return "-....";  case '7': return "--...";  case '8': return "---..";
        case '9': return "----.";
        default:  return "";
    }
}

// Emite textul in MCW. Verifica butonul intre litere; intoarce true daca s-a cerut STOP.
static bool send_morse_text(const char *txt, int U)
{
    for (const char *p = txt; *p; p++) {
        if (gpio_get_level(PIN_BTN) == 0) return true;      // STOP
        char c = *p;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c == ' ') { vTaskDelay(pdMS_TO_TICKS(4 * U)); continue; }  // pauza intre cuvinte (~7U)
        for (const char *s = morse_of(c); *s; s++) {
            mcw_tone((*s == '-') ? 3 * U : U);
            vTaskDelay(pdMS_TO_TICKS(U));                   // pauza intre elemente
        }
        vTaskDelay(pdMS_TO_TICKS(2 * U));                   // pauza intre litere (total 3U)
    }
    return false;
}

// ---------------- Buton (scurt / lung) ----------------
typedef enum { BTN_NONE, BTN_SHORT, BTN_LONG } btn_t;
static btn_t button_event(void)
{
    static bool down = false;
    static int64_t t_down = 0;
    int lv = gpio_get_level(PIN_BTN);            // 0 = apasat
    if (lv == 0 && !down) { down = true; t_down = esp_timer_get_time(); }
    else if (lv == 1 && down) {
        down = false;
        int64_t held = esp_timer_get_time() - t_down;
        if (held > 50000) return (held > 800000) ? BTN_LONG : BTN_SHORT;
    }
    return BTN_NONE;
}

// ---------------- UI ----------------
static void ui_header(mode_t m, int temp)
{
    char t[12];
    disp_fill(0, 0, LCD_W, 22, C_BLUE);
    disp_text(4, 4, "RF SCAN 433", C_WHITE, C_BLUE, 2);
    snprintf(t, sizeof(t), "%dC", temp);
    disp_text(LCD_W - 60, 4, t, C_WHITE, C_BLUE, 2);
    disp_fill(0, 22, LCD_W, 20, C_DGRAY);
    disp_text(4, 25, MODE_NAME[m], C_YELLOW, C_DGRAY, 2);
}

void app_main(void)
{
    gpio_config_t sd = { .pin_bit_mask = (1ULL << PIN_SD_CS), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&sd); gpio_set_level(PIN_SD_CS, 1);
    gpio_config_t btn = { .pin_bit_mask = (1ULL << PIN_BTN), .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&btn);

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO, .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_H * 2 + 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    led_init();
    led_set(0, 0, 8);

    if (!disp_init(SPI2_HOST, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST, PIN_LCD_BL)) { led_set(20,0,0); return; }
    disp_set_backlight(40);    // luminozitate redusa (mai putina caldura)
    disp_clear(C_BLACK);
    disp_text(4, 60, "INIT SI4432", C_WHITE, C_BLACK, 2);
    disp_flush();

    bool ok = si4432_init(PIN_SI_SCLK, PIN_SI_MOSI, PIN_SI_MISO, PIN_SI_CS,
                          PIN_SI_SDN, PIN_SI_IRQ, PIN_SI_RXD);
    if (!ok) {
        disp_clear(C_BLACK);
        disp_text(4, 50, "SI4432 LIPSA", C_RED, C_BLACK, 3);
        disp_text(4, 90, "VERIFICA SPI", C_WHITE, C_BLACK, 2);
        disp_flush();
        while (1) {
            uint8_t dt = si4432_reg_read(SI_R_DEVTYPE), vr = si4432_reg_read(SI_R_VERSION);
            ESP_LOGE("main", "Si4432 lipsa: DEVTYPE=0x%02X VERSION=0x%02X", dt, vr);
            led_set(30,0,0); vTaskDelay(pdMS_TO_TICKS(500));
            led_set(0,0,0);  vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    ESP_LOGI("main", "Si4432 OK. Start.");

    mode_t mode = MODE_GONOGO;
    uint8_t rssi[BAND_NCH];
    char line[32];
    uint32_t exact_hz = 0;             // ultima frecventa exacta masurata (mod FRECVENTA)
    int64_t exact_at = 0;
    int burst_count = 0;               // apasari (rafale separate de >EVENT_HOLD_US)
    int64_t last_event = -EVENT_HOLD_US * 2;
    bool parked = false;
    // latch pt. rezultatul de identificare (sa nu dispara imediat)
    ident_t held = { "?", "-", 0, 0 };
    bool has_held = false;
    uint32_t held_khz = 0;
    int held_dbm = 0;
    int64_t last_analyze = 0;
    int chip_temp = 0;
    int64_t last_temp = 0;
    // luminozitate: nivele ciclate cu apasare lunga
    const uint8_t BL_LEVELS[] = { 15, 40, 70, 100 };
    int bl_idx = 1;                    // 40% implicit
    int64_t lum_until = 0;            // pana cand se afiseaza indicatorul "LUM %"

    while (1) {
        btn_t ev = button_event();
        if (ev == BTN_SHORT) {
            mode = (mode + 1) % MODE_COUNT;
            burst_count = 0;
            has_held = false;          // rezultat nou la fiecare intrare in mod
            disp_clear(C_BLACK);
        } else if (ev == BTN_LONG) {
            bl_idx = (bl_idx + 1) % (int)(sizeof(BL_LEVELS));
            disp_set_backlight(BL_LEVELS[bl_idx]);
            lum_until = esp_timer_get_time() + 1200000;   // arata 1.2 s
        }

        // rafala prinsa de intreruperea RSSI cat timp receptorul a stat parcat
        bool irq_hit = false;
        if (parked) {
            irq_hit = si4432_rssi_irq_fired();
            si4432_rssi_irq_disarm();
            parked = false;
        }

        int64_t tnow = esp_timer_get_time();
        if (tnow - last_temp > 2000000) { chip_temp = si4432_read_temp_c(); last_temp = tnow; }

        // ---- MOD TX (beacon MCW in bucla continua) ----
        if (mode == MODE_TXTEST) {
            ui_header(mode, chip_temp);
            disp_fill(0, 44, LCD_W, LCD_H - 44, C_BLACK);
            disp_text(10, 48, "TX BEACON", C_RED, C_BLACK, 3);
            snprintf(line, sizeof(line), "%lu.%03lu MHZ",
                     (unsigned long)(TX_FREQ_KHZ / 1000), (unsigned long)(TX_FREQ_KHZ % 1000));
            disp_text(10, 80, line, C_YELLOW, C_BLACK, 3);
            disp_text(10, 112, TX_MSG, C_GREEN, C_BLACK, 2);
            disp_text(10, 140, "BOOT = STOP", C_GRAY, C_BLACK, 2);
            snprintf(line, sizeof(line), "%d DBM", TX_DBM[TX_POWER]);
            disp_text(220, 140, line, C_WHITE, C_BLACK, 2);
            disp_flush();

            si4432_set_channel(0);
            si4432_set_freq_khz(TX_FREQ_KHZ);
            si4432_set_tx_power(TX_POWER);
            si4432_tx_prep();
            led_set(30, 0, 0);
            bool stop = send_morse_text(TX_MSG, TX_UNIT);
            si4432_tx_key(false);
            led_set(0, 0, 0);

            if (stop) {
                si4432_config_scan(150);                     // inapoi pe RX
                while (gpio_get_level(PIN_BTN) == 0) vTaskDelay(pdMS_TO_TICKS(10)); // asteapta eliberarea
                mode = (mode + 1) % MODE_COUNT;
                disp_clear(C_BLACK);
            } else {
                // pauza intre repetari, dar iesi rapid daca se apasa BOOT
                for (int i = 0; i < 14; i++) {
                    if (gpio_get_level(PIN_BTN) == 0) break;
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            continue;
        }

        // ---- MODURI RX ----
        scan_band(rssi, BAND_NCH);
        int peak_ch = 0; uint8_t peak = 0;
        for (int i = 0; i < BAND_NCH; i++) if (rssi[i] > peak) { peak = rssi[i]; peak_ch = i; }
        uint32_t peak_khz = BAND_START_KHZ + peak_ch * BAND_STEP_KHZ;

        // fondul = mediana canalelor. Cu minimul, varfurile zgomotului (max din 18
        // citiri) ieseau cu ~28 peste fond, adica peste RSSI_MARGIN => EMITE fals.
        uint8_t srt[BAND_NCH];
        memcpy(srt, rssi, sizeof(srt));
        for (int i = 1; i < BAND_NCH; i++) {
            uint8_t v = srt[i]; int j = i - 1;
            while (j >= 0 && srt[j] > v) { srt[j + 1] = srt[j]; j--; }
            srt[j + 1] = v;
        }
        noise_floor = (noise_floor * 15 + srt[BAND_NCH / 2]) / 16;

        bool active = (peak > noise_floor + RSSI_MARGIN);
        if (active || irq_hit) {
            if (tnow - last_event > EVENT_HOLD_US) burst_count++;   // apasare noua
            last_event = tnow;
        }
        bool emitting = (tnow - last_event < EVENT_HOLD_US);

        // mod FRECVENTA: masurare fina o data pe apasare (max o data la 1.5 s)
        if (mode == MODE_FREQ && active && tnow - exact_at > 1500000) {
            disp_text(6, 158, "MASOR FRECVENTA...", C_ORANGE, C_BLACK, 2);
            disp_flush();
            uint32_t f;
            if (measure_freq_hz(peak_khz, &f)) exact_hz = f;
            exact_at = esp_timer_get_time();
        }

        // intre baleiaje: parcare cu intrerupere RSSI (nu in IDENTIFICARE, care
        // reconfigureaza singura receptorul)
        if (mode != MODE_IDENTIFY) { park_and_arm(); parked = true; }

        static int logdiv = 0, n_act = 0, n_irq = 0;
        static uint8_t pk_max = 0;
        if (active) n_act++;
        if (irq_hit) n_irq++;
        if (peak > pk_max) pk_max = peak;
        if (++logdiv >= 20) {
            ESP_LOGI("scan", "20 cicluri: scan_activ=%d irq=%d peak_max=%u floor=%u park_floor=%u",
                     n_act, n_irq, pk_max, noise_floor, park_floor);
            logdiv = 0; n_act = 0; n_irq = 0; pk_max = 0;
        }

        ui_header(mode, chip_temp);

        if (mode == MODE_GONOGO) {
            disp_fill(0, 44, LCD_W, LCD_H - 44, C_BLACK);
            if (emitting) { led_set(0, 30, 0);
                disp_text(10, 50, "EMITE", C_GREEN, C_BLACK, 5); }
            else { led_set(0, 0, 0);
                disp_text(10, 50, "NIMIC", C_GRAY, C_BLACK, 5); }
            snprintf(line, sizeof(line), "%d DBM", (peak / 2) - 130);
            disp_text(10, 104, line, C_WHITE, C_BLACK, 2);
            snprintf(line, sizeof(line), "APASARI %d", burst_count);
            disp_text(160, 104, line, C_CYAN, C_BLACK, 2);
            int bar = (peak > noise_floor) ? (peak - noise_floor) * LCD_W / 100 : 0;
            if (bar > LCD_W) bar = LCD_W;
            disp_fill(0, 150, LCD_W, 18, C_DGRAY);
            disp_fill(0, 150, bar, 18, emitting ? C_GREEN : C_ORANGE);

        } else if (mode == MODE_FREQ) {
            disp_fill(0, 44, LCD_W, LCD_H - 44, C_BLACK);
            int bw = LCD_W / BAND_NCH;
            for (int i = 0; i < BAND_NCH; i++) {
                int h = (rssi[i] > noise_floor) ? (rssi[i] - noise_floor) * 84 / 120 : 2;
                if (h > 84) h = 84;
                if (h < 2) h = 2;
                uint16_t c = (i == peak_ch && active) ? C_GREEN : C_DGRAY;
                disp_fill(i * bw, 44 + 84 - h, bw - 1, h, c);
            }
            // frecventa exacta ramane afisata 10 s dupa masurare
            bool show_exact = exact_hz && (tnow - exact_at < 10000000);
            if (active) led_set(0, 20, 10); else led_set(0, 0, 0);
            if (show_exact) {
                fmt_freq(line, sizeof(line), exact_hz);
                disp_text(6, 132, line, C_YELLOW, C_BLACK, 3);
                fmt_dev(line, sizeof(line), exact_hz);
                disp_text(4, 158, line, C_ORANGE, C_BLACK, 2);
            } else if (active) {
                snprintf(line, sizeof(line), "%lu.%02lu MHZ",
                         (unsigned long)(peak_khz/1000), (unsigned long)((peak_khz%1000)/10));
                disp_text(6, 132, line, C_YELLOW, C_BLACK, 3);
            } else disp_text(6, 132, "-- SCAN --", C_GRAY, C_BLACK, 3);
            if (active) {
                snprintf(line, sizeof(line), "%d DBM", (peak / 2) - 130);
                disp_text(214, 138, line, C_WHITE, C_BLACK, 2);
            }

        } else { // MODE_IDENTIFY
            int64_t now = esp_timer_get_time();
            // reanalizeaza doar daca e semnal si a trecut >3s de la ultima (sau n-avem niciun rezultat)
            if (active && (!has_held || now - last_analyze > 3000000)) {
                led_set(20, 20, 0);
                disp_fill(0, 44, LCD_W, LCD_H - 44, C_BLACK);
                disp_text(6, 60, "ANALIZEZ...", C_ORANGE, C_BLACK, 2);
                disp_flush();
                held = identify(peak_khz);
                held_khz = peak_khz;
                held_dbm = (peak / 2) - 130;
                has_held = true;
                last_analyze = esp_timer_get_time();
            }
            // desenare (rezultatul RAMANE afisat pana la urmatoarea analiza)
            disp_fill(0, 44, LCD_W, LCD_H - 44, C_BLACK);
            if (!has_held) {
                led_set(0, 0, 0);
                disp_text(6, 70, "ASTEPT SEMNAL", C_GRAY, C_BLACK, 2);
            } else {
                led_set(0, 25, 0);
                if (held.f_hz) fmt_freq(line, sizeof(line), held.f_hz);
                else snprintf(line, sizeof(line), "%lu.%02lu MHZ",
                              (unsigned long)(held_khz/1000), (unsigned long)((held_khz%1000)/10));
                disp_text(6, 46, line, C_YELLOW, C_BLACK, 2);
                snprintf(line, sizeof(line), "%d DBM", held_dbm);
                disp_text(200, 46, line, C_WHITE, C_BLACK, 2);
                disp_text(6, 72, held.mod, C_CYAN, C_BLACK, 2);
                disp_text(6, 98, held.type, C_GREEN, C_BLACK, 3);   // TIPUL, mare
                if (held.bitrate > 0) { snprintf(line, sizeof(line), "%d BPS", held.bitrate);
                    disp_text(6, 140, line, C_WHITE, C_BLACK, 2); }
                if (held.f_hz) { fmt_dev(line, sizeof(line), held.f_hz);
                    disp_text(6, 158, line, C_ORANGE, C_BLACK, 2); }
            }
        }

        // indicator luminozitate (dupa apasare lunga), suprapus temporar
        if (esp_timer_get_time() < lum_until) {
            disp_fill(50, 60, 220, 52, C_DGRAY);
            snprintf(line, sizeof(line), "LUM %d%%", BL_LEVELS[bl_idx]);
            disp_text(70, 74, line, C_WHITE, C_DGRAY, 3);
        }

        disp_flush();
        vTaskDelay(pdMS_TO_TICKS(mode == MODE_IDENTIFY ? 300 : 40));
    }
}
