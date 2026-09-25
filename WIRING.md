# WIRING.md — sursa de adevăr pentru cablare

Placă: **Waveshare ESP32-C6-LCD-1.47** (ST7789 172×320, WS2812, slot TF).
Modul RF: **XL4432-SMT** (Silicon Labs **Si4432** / RFM22, 433 MHz, SPI).

## Magistrala SPI — PARTAJATĂ (LCD + card TF + Si4432)

Toate trei stau pe SPI2. CS separat pentru fiecare; CS-ul dispozitivului inactiv rămâne HIGH.

| Semnal | GPIO C6 | Note |
|--------|---------|------|
| MOSI (SDA) | **6**  | LCD + TF + Si4432 SDI |
| MISO       | **5**  | TF + Si4432 SDO (LCD nu folosește) |
| SCLK       | **7**  | comun; clock Si4432 ≤ 8 MHz în driver |

## LCD ST7789 (confirmat pe wiki Waveshare)

| Semnal | GPIO |
|--------|------|
| DC  | 15 |
| CS  | 14 |
| RST | 21 |
| BL (backlight) | 22 |

Offset fereastră 172px: `set_gap(34, 0)`, `invert_color(true)`. Dacă imaginea
apare decalată sau cu culori inversate, aici se reglează (± câțiva px pe gap,
sau `mirror`/`swap_xy`).

## Card TF (nefolosit de firmware, dar pe aceeași magistrală)

| Semnal | GPIO |
|--------|------|
| SD_CS | 4 |

Firmware-ul îl setează **output HIGH** la boot ca să-l deselecteze. Dacă adaugi
logging pe card, refolosește MOSI6/MISO5/SCLK7 + CS4.

## On-board

| Semnal | GPIO |
|--------|------|
| WS2812 (LED RGB) | 8 |
| BOOT (schimbă modul) | 9 |

## Header-ul real al plăcii (confirmat din poza Waveshare)

Se scot **15 GPIO**:
- Stânga: `5V  GND  3V3(OUT)  GP0  GP1  GP2  GP3  GP4  GP5`
- Dreapta: `TX(GPIO16)  RX(GPIO17)  GP13  GP12  GP23  GP20  GP19  GP18  GP9`

⚠️ **GPIO6 și GPIO7 (MOSI+SCLK ale LCD-ului) NU sunt pe header.** Deci Si4432
NU poate sta pe SPI-ul hardware al LCD-ului → merge pe **SPI SOFTWARE (bit-bang)**.

⚠️⚠️ **GPIO12 și GPIO13 = liniile USB (D-/D+)!** Deși-s scoase pe header ca
GP12/GP13, dacă le folosești ca GPIO OMORI portul USB (fără flash, fără consolă).
De asemenea GP16/GP17 = consola UART. **De evitat pentru orice: 12, 13, 16, 17**
(USB/consolă) și **4, 5, 8, 9, 15** (strapping).

## Si4432 / XL4432-SMT — MONTAJ FINAL (SPI software)

| Pin modul | Funcție | GPIO C6 (pad header) |
|-----------|---------|----------------------|
| VCC  | 3,3 V | **3V3(OUT)** (Si4432 e 3,3 V, fără level shifter) |
| GND  | masă  | **GND** |
| SDN  | shutdown | **GP19** |
| nSEL | chip select | **GP18** |
| SCLK | clock | **GP0** |
| SDI  | MOSI  | **GP1** |
| SDO  | MISO  | **GP23** |
| nIRQ | întrerupere | **GP20** (optional) |
| GPIO2 | data brută OOK | **GP2** (optional, doar pt. Mod 3) |
| ANT  | antenă | **λ/4 = 17,3 cm** sârmă sau arc |

Minim ca să treacă de detecție (`DEVTYPE=0x08`): VCC, GND, **SDN, nSEL, SCLK, SDI, SDO**.
nIRQ și GPIO2 le poți lăsa la urmă. Pinii sunt în `#define PIN_SI_*` din `main/main.c`.

## Alimentare

RX/scanare: Si4432 ~18 mA, neglijabil pe USB. Dacă activezi vreodată TX (+20 dBm)
sunt ~85 mA vârf — tot OK pe USB-C, dar pune un condensator de 100 µF lângă modul.
