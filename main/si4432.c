#include "si4432.h"
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "si4432";
static int s_sclk = -1, s_mosi = -1, s_miso = -1, s_cs = -1;
static int s_sdn = -1, s_irq = -1, s_rxd = -1;
static uint32_t s_base_khz = 433920;

// ---- SPI SOFTWARE (mode 0: CPOL=0, CPHA=0) ----
static inline void bb_delay(void) { esp_rom_delay_us(1); }   // ~ sub 500 kHz

static uint8_t bb_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(s_mosi, (out >> i) & 1);
        bb_delay();
        gpio_set_level(s_sclk, 1);               // front crescator: slave-ul eșantionează
        bb_delay();
        in = (in << 1) | (gpio_get_level(s_miso) & 1);
        gpio_set_level(s_sclk, 0);
    }
    return in;
}

// reg|0x80 = scriere, reg&0x7F = citire (2 octeți)
void si4432_reg_write(uint8_t reg, uint8_t val)
{
    gpio_set_level(s_cs, 0);
    bb_xfer(reg | 0x80);
    bb_xfer(val);
    gpio_set_level(s_cs, 1);
}

uint8_t si4432_reg_read(uint8_t reg)
{
    gpio_set_level(s_cs, 0);
    bb_xfer(reg & 0x7F);
    uint8_t v = bb_xfer(0x00);
    gpio_set_level(s_cs, 1);
    return v;
}

// ---- Frecvență: fRF = 10MHz*(hbsel+1)*(fb+24 + fc/64000) ----
void si4432_set_freq_khz(uint32_t khz)
{
    s_base_khz = khz;
    double f = khz / 1000.0;                 // MHz
    uint8_t hbsel = (f >= 480.0) ? 1 : 0;
    double N = f / (10.0 * (hbsel + 1));      // ex. 43.392
    uint8_t fb = (uint8_t)N - 24;             // partea intreagă - 24
    uint16_t fc = (uint16_t)lround((N - (uint8_t)N) * 64000.0);
    si4432_reg_write(SI_R_FBANDSEL, 0x40 | (hbsel << 5) | (fb & 0x1F)); // sbsel=1
    si4432_reg_write(SI_R_FCARRIER1, fc >> 8);
    si4432_reg_write(SI_R_FCARRIER2, fc & 0xFF);
}

void si4432_set_hop_step_khz(uint32_t step)  // reg în unități de 10 kHz
{
    uint32_t v = step / 10;
    if (v > 255) v = 255;
    si4432_reg_write(SI_R_HOPSTEP, (uint8_t)v);
}

void si4432_set_channel(uint8_t ch)
{
    si4432_reg_write(SI_R_HOPCH, ch);
}

void si4432_enter_rx(void)
{
    si4432_reg_write(SI_R_OPMODE1, 0x05); // xton + rxon
}

void si4432_idle(void)
{
    si4432_reg_write(SI_R_OPMODE1, 0x01); // doar xton (ready)
}

uint8_t si4432_rssi_raw(void)
{
    return si4432_reg_read(SI_R_RSSI);
}

int si4432_rssi_dbm(void)
{
    // Aproximare uzuală Si443x: dBm ~= RSSI/2 - 130 (necesită calibrare fină).
    return (si4432_rssi_raw() / 2) - 130;
}

int si4432_rxdata_level(void)
{
    return (s_rxd >= 0) ? gpio_get_level(s_rxd) : 0;
}

// Filtrul IF (reg 0x1C = dwn3_bypass<<7 | ndec_exp<<4 | filset), din tabelul
// "Filter Bandwidth Parameters" al datasheet-ului Si4432. Se alege cea mai mica
// latime >= cea ceruta. 150 kHz ramane pe 0x9A, valoarea validata pe placa.
static const struct { uint16_t khz10; uint8_t reg; } IFBW_TAB[] = {
    {  189, 0x21 }, {  210, 0x22 }, {  240, 0x24 }, {  282, 0x25 }, {  322, 0x26 },
    {  377, 0x11 }, {  452, 0x13 }, {  562, 0x15 }, {  641, 0x16 }, {  752, 0x01 },
    {  900, 0x03 }, { 1121, 0x05 }, { 1379, 0x07 }, { 1678, 0x95 }, { 2251, 0x81 },
    { 2849, 0x84 }, { 3618, 0x89 }, { 4684, 0x8B }, { 5770, 0x8D }, { 6207, 0x8E },
};

static uint8_t ifbw_reg(uint32_t bw_khz)
{
    if (bw_khz == 150) return 0x9A;
    for (size_t i = 0; i < sizeof(IFBW_TAB) / sizeof(IFBW_TAB[0]); i++)
        if (IFBW_TAB[i].khz10 >= bw_khz * 10) return IFBW_TAB[i].reg;
    return 0x8E;                             // maxim ~620 kHz
}

// Config comun de receptor (AGC pornit, fără packet handler)
static void common_rx_setup(uint32_t bw_khz)
{
    si4432_reg_write(SI_R_DATACTRL, 0x00);   // packet handler off
    si4432_reg_write(SI_R_AGCOVR,  0x60);    // AGC automat pornit
    si4432_reg_write(SI_R_IFBW, ifbw_reg(bw_khz));
    si4432_reg_write(SI_R_AFCGEAR,   0x40);
    si4432_reg_write(SI_R_AFCTIMING, 0x0A);
    si4432_reg_write(SI_R_RSSITH, 0x1E);
}

void si4432_config_scan(uint32_t bw_khz)
{
    si4432_idle();
    common_rx_setup(bw_khz);
    // FSK "neutru" pentru simpla măsurare de energie; GPIO2 pe RSSI analog nefolosit.
    si4432_reg_write(SI_R_MODMODE2, 0x22);   // dtmod=FIFO, modtyp=FSK (energie)
    si4432_reg_write(SI_R_GPIO2CFG, 0x00);
    si4432_enter_rx();
}

// ---- Intrerupere de prag RSSI (irssi): starea ramane agatata pana la citire,
// deci prinde si rafalele care cad cat timp CPU-ul deseneaza ecranul ----
void si4432_rssi_irq_arm(uint8_t thr_raw)
{
    si4432_reg_write(SI_R_RSSITH, thr_raw);
    si4432_reg_write(SI_R_IEN2, 0x10);       // enrssi
    (void)si4432_reg_read(SI_R_ISTAT1);      // citirea sterge starile vechi
    (void)si4432_reg_read(SI_R_ISTAT2);
}

bool si4432_rssi_irq_fired(void)
{
    (void)si4432_reg_read(SI_R_ISTAT1);
    return (si4432_reg_read(SI_R_ISTAT2) & 0x10) != 0;   // irssi
}

void si4432_rssi_irq_disarm(void)
{
    si4432_reg_write(SI_R_IEN2, 0x00);
}

#define SI_R_TXPOW 0x6D

void si4432_set_tx_power(uint8_t pwr)
{
    si4432_reg_write(SI_R_TXPOW, 0x18 | (pwr & 0x07));
}

void si4432_tx_carrier(bool on)
{
    if (on) {
        si4432_reg_write(SI_R_MODMODE2, 0x00);   // purtatoare nemodulata
        si4432_reg_write(SI_R_OPMODE1, 0x09);    // txon + xton
    } else {
        si4432_reg_write(SI_R_OPMODE1, 0x01);    // idle (ready)
    }
}

void si4432_tx_prep(void)
{
    si4432_reg_write(SI_R_MODMODE2, 0x00);       // purtatoare nemodulata, fara txon
    si4432_reg_write(SI_R_OPMODE1, 0x01);        // ready
}

void si4432_tx_key(bool on)
{
    // doar bitul txon - cheiere rapida pentru ton MCW (nu reseteaza modtype)
    si4432_reg_write(SI_R_OPMODE1, on ? 0x09 : 0x01);
}

void si4432_set_offset(int16_t units)
{
    // Frequency Offset (reg 0x73/0x74), 10 biti in complement fata de 2, LSB ~156 Hz.
    // Modificat live in timpul TX => shift de frecventa (FSK) fara re-cheiere.
    si4432_reg_write(0x73, (uint8_t)(units & 0xFF));
    si4432_reg_write(0x74, (uint8_t)((units >> 8) & 0x03));
}

int si4432_read_temp_c(void)
{
    // tsrange=00 (0.5 C/LSB, offset -64), entsoffs pornit
    si4432_reg_write(0x12, 0x20);
    // adcstart=1, adcsel=000 (senzor temperatura), adcref bandgap
    si4432_reg_write(0x0F, 0x80);
    esp_rom_delay_us(400);            // timp de conversie ADC
    uint8_t adc = si4432_reg_read(0x11);
    return (int)(adc / 2) - 64;       // T[C] = 0.5*ADC - 64
}

void si4432_config_ook_raw(void)
{
    si4432_idle();
    common_rx_setup(150);
    // OOK + Direct Mode (date brute pe GPIO). GPIO2 = ieșire "RX Data".
    // MODMODE2: dtmod[5:4]=00 (direct via GPIO), modtyp[1:0]=01 (OOK)
    si4432_reg_write(SI_R_MODMODE2, 0x01);
    si4432_reg_write(SI_R_GPIO2CFG, 0x14);   // GPIO2 = RX Data (output)
    si4432_enter_rx();
}

bool si4432_init(int sclk_gpio, int mosi_gpio, int miso_gpio, int cs_gpio,
                 int sdn_gpio, int irq_gpio, int rxdata_gpio)
{
    s_sclk = sclk_gpio; s_mosi = mosi_gpio; s_miso = miso_gpio; s_cs = cs_gpio;
    s_sdn = sdn_gpio; s_irq = irq_gpio; s_rxd = rxdata_gpio;

    // Iesiri: SCLK, MOSI, CS, SDN
    gpio_config_t o = {
        .pin_bit_mask = (1ULL << sclk_gpio) | (1ULL << mosi_gpio) |
                        (1ULL << cs_gpio)   | (1ULL << sdn_gpio),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&o);
    // Intrari: MISO (+ optional IRQ, RXDATA)
    uint64_t inmask = (1ULL << miso_gpio);
    if (irq_gpio >= 0) inmask |= (1ULL << irq_gpio);
    if (rxdata_gpio >= 0) inmask |= (1ULL << rxdata_gpio);
    gpio_config_t in = { .pin_bit_mask = inmask, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&in);

    gpio_set_level(cs_gpio, 1);    // CS inactiv
    gpio_set_level(sclk_gpio, 0);  // CPOL=0

    // Reset hardware prin SDN
    gpio_set_level(sdn_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(sdn_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(25));

    // Software reset
    si4432_reg_write(SI_R_OPMODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t dt = si4432_reg_read(SI_R_DEVTYPE);
    uint8_t ver = si4432_reg_read(SI_R_VERSION);
    ESP_LOGI(TAG, "DEVTYPE=0x%02X VERSION=0x%02X", dt, ver);
    if (dt != 0x08) {
        ESP_LOGE(TAG, "Si4432 negasit (DEVTYPE!=0x08) - verifica cablarea SPI/SDN");
        return false;
    }

    si4432_reg_write(SI_R_XTALCAP, 0x7F);
    si4432_reg_write(SI_R_MCUCLK,  0x00);
    si4432_reg_write(SI_R_IEN1, 0x00);
    si4432_reg_write(SI_R_IEN2, 0x00);
    // Comutatorul de antena al modulului (RFM22/XL4432): GPIO0=stare TX, GPIO1=stare RX.
    // Fara astea TX-ul nu ajunge la antena.
    si4432_reg_write(SI_R_GPIO0CFG, 0x12);   // TX state
    si4432_reg_write(SI_R_GPIO1CFG, 0x15);   // RX state
    si4432_set_freq_khz(433920);
    si4432_set_hop_step_khz(100);
    si4432_config_scan(150);
    return true;
}
