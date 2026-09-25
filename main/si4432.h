// si4432.h - driver minimal pentru XL4432-SMT (Silicon Labs Si4432 / RFM22)
// Scop: detectie de energie (RSSI), baleiaj pe banda ISM 433 si receptie OOK
//       in "direct mode" (data bruta pe un GPIO) pentru estimarea bitrate-ului.
// NOTA: SPI SOFTWARE (bit-bang) - pentru ca pinii SPI hardware ai LCD-ului
//       (GPIO6/7) NU sunt scosi pe header-ul ESP32-C6-LCD-1.47.
#pragma once
#include <stdint.h>
#include <stdbool.h>

// Registre Si4432 folosite (vezi datasheet Si4432 rev B1)
#define SI_R_DEVTYPE      0x00   // = 0x08
#define SI_R_VERSION      0x01   // = 0x06 (rev B1)
#define SI_R_ISTAT1       0x03
#define SI_R_ISTAT2       0x04
#define SI_R_IEN1         0x05
#define SI_R_IEN2         0x06
#define SI_R_OPMODE1      0x07   // xton=0x01 pllon=0x02 rxon=0x04 txon=0x08 swreset=0x80
#define SI_R_OPMODE2      0x08
#define SI_R_XTALCAP      0x09
#define SI_R_MCUCLK       0x0A
#define SI_R_GPIO0CFG     0x0B
#define SI_R_GPIO1CFG     0x0C
#define SI_R_GPIO2CFG     0x0D
#define SI_R_IOPORTCFG    0x0E
#define SI_R_IFBW         0x1C   // latimea de banda RX (IF filter)
#define SI_R_AFCGEAR      0x1D
#define SI_R_AFCTIMING    0x1E
#define SI_R_RSSI         0x26   // <-- RSSI (citire)
#define SI_R_RSSITH       0x27
#define SI_R_AGCOVR       0x69
#define SI_R_DATACTRL     0x30
#define SI_R_MODMODE1     0x70
#define SI_R_MODMODE2     0x71   // modtyp[1:0], eninv[2], dtmod[5:4], trclk[7:6]
#define SI_R_FDEV         0x72
#define SI_R_FBANDSEL     0x75   // sbsel[6] hbsel[5] fb[4:0]
#define SI_R_FCARRIER1    0x76   // fc high
#define SI_R_FCARRIER2    0x77   // fc low
#define SI_R_HOPCH        0x79   // canal de hopping
#define SI_R_HOPSTEP      0x7A   // pas hopping in unitati de 10 kHz

// SPI software: se dau pinii direct. Returneaza true daca cipul raspunde.
bool si4432_init(int sclk_gpio, int mosi_gpio, int miso_gpio, int cs_gpio,
                 int sdn_gpio, int irq_gpio, int rxdata_gpio);

void    si4432_reg_write(uint8_t reg, uint8_t val);
uint8_t si4432_reg_read(uint8_t reg);

void si4432_set_freq_khz(uint32_t khz);      // seteaza purtatoarea de baza (canal 0)
void si4432_set_hop_step_khz(uint32_t step); // pas de baleiaj (10..2550 kHz)
void si4432_set_channel(uint8_t ch);         // freq = baza + ch*pas
void si4432_enter_rx(void);
void si4432_idle(void);

uint8_t si4432_rssi_raw(void);   // 0..255
int     si4432_rssi_dbm(void);   // aproximativ

void si4432_config_scan(uint32_t bw_khz);  // mod detectie energie / baleiaj RSSI, filtru IF ~bw_khz (19..620)

// Intrerupere de prag RSSI: armare cu prag (unitati brute RSSI), interogare (sterge starea)
void si4432_rssi_irq_arm(uint8_t thr_raw);
bool si4432_rssi_irq_fired(void);
void si4432_rssi_irq_disarm(void);
void si4432_config_ook_raw(void);          // OOK direct mode, data bruta pe GPIO2->rxdata
int  si4432_rxdata_level(void);            // nivelul liniei de date brute (0/1)

// --- TX (pentru beacon-ul de test CW / MCW) ---
void si4432_set_tx_power(uint8_t pwr);     // 0 = minim ... 7 = maxim (+20 dBm)
void si4432_tx_carrier(bool on);           // purtatoare nemodulata on/off (cheiere CW)
void si4432_tx_prep(void);                 // pregateste TX (purtatoare nemodulata), fara emisie
void si4432_tx_key(bool on);               // cheiere rapida (doar txon) pt. ton MCW 800 Hz
void si4432_set_offset(int16_t units);     // deviatie de frecventa (LSB ~156 Hz) pt. ton FSK ingust

// --- senzor de temperatura intern al cipului ---
int  si4432_read_temp_c(void);             // grade C (aproximativ)
