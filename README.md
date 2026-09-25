# RF Scanner 433 MHz — ESP32-C6-LCD-1.47 + XL4432-SMT (Si4432)

Tester/analizor pentru banda 433: „emite?" (chei auto EU + telecomenzi poartă),
frecvența detectată și modulația (OOK/FSK). Vezi `DESIGN.md` și `WIRING.md`.

## Build & flash (Windows, ESP-IDF v5.5.5)

```powershell
& 'C:\Espressif\Initialize-Idf.ps1' -IdfId esp-idf-7a8737a10725bc5cb19d516a1186b39f
cd C:\Users\123\RF_Scanner_C6_Si4432
idf.py set-target esp32c6
idf.py build
idf.py -p COMxx flash monitor
```

(Portul COM al C6-ului: verifică în Device Manager; e USB-Serial-JTAG nativ,
GPIO12/13.)

⚠️ Doar componente din arborele ESP-IDF (esp_lcd, driver, rmt) — **fără managed
components**, deci compilează offline.

⚠️ Capcană cunoscută: flag-urile `-D` rămân în `build/CMakeCache.txt` între
build-uri. La nevoie `idf.py fullclean`.

## Utilizare

- Butonul **BOOT** comută între cele 3 moduri: GO/NOGO → FRECVENȚĂ → CLASIFICARE.
- Ține telecomanda apăsată la ~10–30 cm de antenă (repetă burst-urile, nu le ratezi).
- **Antena λ/4 = 17,3 cm** e obligatorie, altfel nu prinde nimic.

## Status

Cod complet, structurat, compilează cu componente in-tree. Necesită calibrare pe
banc: registrul de BW Si4432 (0x1C), configul OOK direct mode, formula RSSI→dBm,
și eventual offset-ul/culorile LCD. Detalii în `DESIGN.md` la „calibrare pe banc".
