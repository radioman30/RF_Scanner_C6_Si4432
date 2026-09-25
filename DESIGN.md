# DESIGN.md — RF Scanner 433 MHz (ESP32-C6 + Si4432)

## Scop

Aparat mic autonom pentru **banda 433 MHz** care:
1. spune dacă o telecomandă **emite** (chei auto EU, telecomenzi de poartă);
2. arată **pe ce frecvență** emite;
3. **clasifică modulația** (OOK/ASK vs FSK/GFSK) și estimează bitrate-ul.

Nu decodează codul și nu clonează — e un **tester/analizor**, nu o clonă de
telecomandă. Rolling code (auto) se **vede** ca energie, dar nu se reproduce.

## De ce Si4432 și ce NU poate

Si4432 e transceiver **îngust**, nu SDR. „Scanarea" e prin **baleiaj**: plimbi
PLL-ul peste bandă și citești RSSI (reg 0x26). De aici toate limitele:

- Frecvența la **±50–100 kHz**, nu la kHz (rezoluția = lățimea de bandă RX).
- Poate **rata** un burst scurt căzut între ferestrele de baleiaj → ține butonul
  telecomenzii apăsat (repetă) sau îngustează banda.
- **Acoperire de frecvență:** modulul e acordat pe 433. Prinde bine 433,05–434,79
  (telecomenzi poartă EU + chei auto EU 433,92/434,42). **NU** prinde bine 315 MHz
  (multe chei non-EU) sau 868 MHz — chipul retunează, dar antena/adaptarea sunt
  pe 433. Pentru 315 ai nevoie de un al doilea modul acordat pe 315.
- Cheia de mașină are și un **imobilizator 125 kHz LF pasiv** — ăsta NU se vede
  aici; testerul verifică doar partea RF de butoane.

Etalonul real pentru „frecvență + modulație + protocol" rămâne RTL-SDR + rtl_433;
aparatul ăsta e aproximarea lui într-un gadget de buzunar.

## Gama Si4432 (cercetare)

Cip: **240–930 MHz** (PLL două benzi: joasă 240–479,9 / înaltă 480–930), OOK/FSK/GFSK,
debit 0,123–256 kbps, sensibilitate **−121 dBm**, TX până la **+20 dBm**. Extra pe cip:
RSSI, AFC, frequency hopping, FIFO 64B + packet handler (preambul/sync/CRC), senzor de
temperatură + ADC 8 biți, wake-up timer, low-battery, comutator antenă TX/RX.
**Modulul XL4432 e acordat pe 433** → practic ~400–450 MHz; 315/868 doar cu alt modul.

## Cele patru moduri (buton BOOT = comută)

### Mod 1 — GO/NOGO
Baleiaj rapid al benzii, ia RSSI max. Prag adaptiv peste zgomotul de fond
(`noise_floor` urmărit ca medie a minimelor). Afișează **EMITE / NIMIC** mare,
nivel dBm, bară, contor de burst-uri. LED verde la detecție.

### Mod 2 — FRECVENȚĂ
Același baleiaj, dar desenează un **mini-spectru** (o bară pe canal) și scrie
frecvența vârfului (ex. `433.90 MHZ`) + dBm.

### Mod 3 — IDENTIFICARE (modulație + TIP)
1. Găsește vârful prin baleiaj.
2. **Modulație — domeniul timp:** OOK direct mode, citește linia brută pe GP2 ~50 ms,
   numără fronturile și pulsul minim → OOK/ASK (+ bitrate) vs altceva.
3. **Modulație — domeniul frecvență:** baleiaj fin ±100 kHz, numără lobii peste −6 dB →
   FSK/GFSK (+ deviație) vs CW.
4. **TIP semnal — monitorizare ~1,2 s** la frecvența vârfului (RSSI în timp):
   - prezent continuu (ratio > 85%) → **FM/CONTINUU**;
   - rafale repetate (≥3 burst-uri sau ratio > 30%) → **TELECOMANDĂ** (buton ținut);
   - o rafală scurtă izolată → **SENZOR**.
   Euristic — telecomanda ținută apăsat = rafale susținute; senzorul = burst scurt periodic.

Praguri în `identify()` din `main.c` — de reglat pe banc.

### Mod 4 — TX TEST (beacon CW)
La intrarea în mod, emite **o singură dată** cuvântul **TEST** în Morse (T E S T = − · ··· −)
pe 433,92 MHz, putere **minimă** (`si4432_set_tx_power(0)`), cheiere prin `si4432_tx_carrier()`.
Unitate Morse 100 ms. Necesită configul comutatorului de antenă (reg GPIO0=0x12/GPIO1=0x15,
pus în `si4432_init`). ⚠️ Legal: 433,92 e în 70 cm amator (Regiunea 1) dar se suprapune cu
ISM (10 mW, duty 10%) — de-aia putere minimă și one-shot.

## Ce necesită calibrare pe banc (nu pot verifica de aici)

- **Registrul de BW `0x1C` (`common_rx_setup`)** = 0x9A ca valoare de start
  (~150 kHz). Regenerează cu Silicon Labs WDS/calculator dacă vrei alt BW.
- **Config OOK direct mode** (reg 0x71 = 0x01, GPIO2 = 0x14): dacă linia brută nu
  comută, verifică detectorul OOK (reg 0x2C–0x2E) și pragul.
- **Formula RSSI→dBm** = raw/2 − 130, aproximativă; calibrează cu o sursă cunoscută.
- **Offset/culori LCD** (`set_gap`, `invert_color`) dacă imaginea e decalată.

## Fișiere

- `main/si4432.c/.h` — driver Si4432 (SPI, frecvență, RSSI, OOK direct, hopping).
- `main/display.c/.h` — ST7789 + framebuffer RGB565 + font 5×7 propriu.
- `main/main.c` — mașina de stări cu cele 3 moduri, WS2812, buton.
- `WIRING.md` — cablarea (sursă de adevăr).
