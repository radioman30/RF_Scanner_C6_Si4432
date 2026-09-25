# De făcut / de testat

Stare la 26 sep 2026: firmware-ul din `main` e flashuit și merge (GO/NOGO fără alarme
false, frecvență exactă calibrată, cheia Audi = FSK / TELECOMANDA, beacon pe 432.500).

## Următorii pași (idei din repo-ul Stef-aap, vezi `referinte/stef_aap/`)

Sursa: https://github.com/Stef-aap/ESP32_Projecten (folderele `SI4432_Code`, `SI4432_ESP`).
Copie locală în `referinte/stef_aap/` (exclusă din git, cod terț).

1. **Timp de așezare mai scurt.** Stef a măsurat că ~30 µs după schimbarea canalului
   ajung (22 µs deja distorsionează). La noi: 400 µs în `scan_band`, 300 µs în
   `fine_spectrum`. De încercat 40–50 µs.
   - Test: cheia Audi în FRECVENȚĂ, 5 apăsări; împrăștierea frecvenței (acum ±10 kHz)
     ar trebui să scadă. Verifică și că GO/NOGO nu capătă alarme false.

2. **Configurație completă de recepție OOK.** `si4432_config_ook_raw()` setează doar
   modtyp + filtru. Stef (`SI4432_Init_KAKU` în `SI4432_support.h`, testat pe KAKU
   433.92 MHz, 3.85 kb/s) setează în plus:
   - clock recovery 0x20=0x0B, 0x21=0x60, 0x22=0x2A, 0x23=0x0D, 0x24=0x10, 0x25=0x2C
   - OOK counter 0x2C=0x1A, 0x2D=0x44; slicer peak detector 0x2E=0x0F
   - 0x1C=0xAE (filtru), 0x1D=0x40, 0x1F=0x00, 0x2A=0x2C, 0x69=0x60, 0x70=0x24, 0x71=0x21
   - Test: telecomandă de poartă/garaj în IDENTIFICARE → trebuie „OOK/ASK" + BPS.

3. **Captură puls OOK pe întrerupere GPIO2** (el: `_OOK_Int`, durate cu micros) →
   baza pentru ecranul „UNDĂ" + recunoaștere PT2262 / EV1527 (cod fix) vs KeeLoq
   (rolling). Documentul `SI4432 OOK Detection` descrie PT2262 (12 cifre trinare +
   sync la sfârșit).

## Observații din documentele lui

- Filtrul IF: ~100 kHz recomandat; filtrele înguste (5 kHz) fac semnalele să dispară —
  confirmă ce am văzut (19/37 kHz surde; minim 75 kHz în `ifbw_reg`).
- AN440 are mai multe tabele de filtre; la schimbarea ndec/dwn3 trebuie recalculate
  și 0x20–0x25 (formulele `SI4432_Calc_RxOsc`, `SI4432_Calc_ncoff`, `SI4432_Set_crgain`).
- Nici el n-a lămurit AFC-ul (comentariu „ZEER MOEILIJK NOG UITZOEKEN" la 0x2A).
- Sursă de test: telecomandă ieftină modificată să emită purtătoare continuă.
- Mod „Min" (minimul din N citiri pe bin) îmbunătățește SNR-ul la semnale continue.
