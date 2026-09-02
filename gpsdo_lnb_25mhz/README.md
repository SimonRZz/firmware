# GPSDO 25 MHz für LNB-Referenzeinspeisung

Arduino-Sketch für einen Pro Micro (ATmega32U4), der aus einem NEO-7M und einem
Si5351 eine GPS-disziplinierte 25-MHz-Referenz für ein LNB erzeugt.

Portiert aus `gpsdo.c` des Projekts
[SX1280_QO100_SSB_TX](https://github.com/SimonRZz/SX1280_QO100_SSB_TX), das
dieselbe Technik für 52 MHz am SX1280-XTA-Pin nutzt. Der Ansatz geht
ursprünglich auf [CT2GQV](https://speakyssb.blogspot.com/2019/10/si5351-gps-disciplined-oscillator-with.html)
zurück.

## Prinzip

```
NEO-7M TIMEPULSE: 24 MHz  ->  Si5351 XA-Pin  ->  CLK1: 25 MHz  ->  Bias-Tee  ->  LNB
```

Der Quarz auf dem Si5351-Breakout wird ausgelötet, der 24-MHz-Timepulse des
NEO-7M ersetzt ihn als Referenz. Damit ist die Disziplinierung rein
hardwareseitig — es gibt keinen PPS-Regelkreis in der Firmware. Die MCU
konfiguriert nur einmalig GPS und Si5351 und überwacht danach den Zustand.

24 MHz ist nicht beliebig gewählt: 24 = 48/2 ist ein exakter Teiler des internen
48-MHz-Takts des Moduls. Nur exakte Teiler werden nicht gedithert. Eine
datenblattkonforme Frequenz wie 10 MHz wäre als Referenz deutlich schlechter.

## Frequenzplan

| | Wert |
|---|---|
| Referenz (XA) | 24 MHz |
| PLLA | 24 MHz × **25** = 600 MHz |
| MS1 / CLK1 | 600 MHz ÷ **24** = 25 MHz |

Multiplikator und Ausgangsteiler sind beide ganzzahlig, der Teiler zusätzlich
gerade. Das ist die einzige Kombination im VCO-Fenster 600–900 MHz, die das aus
24 MHz Referenz leistet, und laut Silabs die Konstellation mit dem geringsten
Jitter. Etherkits `set_freq()` würde stattdessen VCO = 900 MHz wählen, also
einen fraktionalen Multiplikator von 37,5.

Die Registerwerte stehen als Kommentar im Sketch und lassen sich nachrechnen:
`p1 = 128·a + floor(128·b/c) − 512`, mit b = 0 und c = 1 für beide Teiler.

## Verkabelung

```
NEO-7M TX          -> Pro Micro D0 (RXI, Serial1 RX)
NEO-7M RX          -> Pro Micro D1 (TXO, Serial1 TX)
NEO-7M TIMEPULSE   -> Si5351 XA (direkt, nicht über die MCU)
NEO-7M VCC/GND     -> 3V3/GND  (220 µF + 100 nF an VCC)
Si5351 SDA         -> Pro Micro D2
Si5351 SCL         -> Pro Micro D3
Si5351 VCC         -> 3V3  (100 nF direkt am Pin)
Si5351 CLK1        -> Bias-Tee -> LNB
Pro Micro D5 -> 330R -> LED rot   -> GND
Pro Micro D6 -> 330R -> LED grün  -> GND
```

Der Quarz auf dem Si5351-Breakout muss ausgelötet werden.

Die 3,3V/8MHz-Variante des Pro Micro kann direkt verdrahtet werden. Bei der
5V/16MHz-Variante sind Pegelwandler auf UART (D0/D1) und I2C (D2/D3) nötig —
NEO-7M und Si5351 sind 3,3-V-Bausteine.

## LED-Codes

| Anzeige | Bedeutung |
|---|---|
| grün dauerhaft | Referenz läuft (PLL locked, Signal am XA) **und** GPS-Fix |
| rot dauerhaft | Referenz läuft, GPS sucht noch — Normalzustand nach dem Einschalten |
| rot 1× blinkend | PLL A nicht eingerastet (LOL) oder Chip noch im SYS_INIT |
| rot 2× blinkend | kein Signal am XA-Pin (LOS) — die 24 MHz kommen nicht an |
| rot 3× blinkend | Si5351 nicht auf dem I2C-Bus gefunden |

Die Blinkcodes kommen aus Si5351-Register 0 und sind beim Aufbau die
nützlichste Diagnose: „GPS hat Fix" heißt eben nicht „die Referenz liegt an".
Am USB-Serial (115200 Bd) läuft parallel eine Statuszeile pro Sekunde.

## Aus `gpsdo.c` übernommen

- die feldbewährten UBX-Pakete `CFG-TP5` (24 MHz, flags `0x6F`) und
  `CFG-NAV5` (dynModel = 2, stationary), byteweise identisch
- I2C-Adressscan über 0x60–0x63 mit drei Versuchen
- Baudraten-Erkennung 9600/38400/57600/115200 (die Suchzeit dient
  gleichzeitig als Bootverzögerung fürs GPS-Modul)
- NMEA-Parser mit Checksummenprüfung und Overflow-Schutz
- Auswertung von Register 0 auf LOS / LOL / SYS_INIT
- Load-Cap `0x92` (8 pF); AN619 führt `0b00` als reserved, deshalb *nicht* 0 pF

Neu gegenüber `gpsdo.c`: der Si5351 wird erst **nach** der GPS-Konfiguration
initialisiert, und wenn LOS später verschwindet, wird einmalig ein PLL-Reset
nachgeschoben. Damit rastet die PLL auch dann sicher ein, wenn das GPS-Modul
länger zum Booten braucht als der Sketch.

Der Sketch braucht **keine externe Bibliothek** außer `Wire`. Das ist Absicht:
Etherkits `si5351.init()` pollt das SYS_INIT-Bit in einer `do/while`-Schleife
und hängt ohne I2C-ACK endlos, weil `Wire.read()` dann `-1` → `0xFF` liefert
und `0xFF >> 7 == 1` ist.

## LNB: PremiumX PXS-SEW

Lowband-LO **9,750 GHz** — für QO-100 der relevante Bereich.

| | |
|---|---|
| Vervielfachung ab 25 MHz | 9750 / 25 = **exakt 390** |
| ZF Schmalbandtransponder | 10489,550–10489,800 MHz → **739,550–739,800 MHz** |
| ZF Mittenbake (PSK) | 10489,750 MHz → **739,750 MHz** |
| ZF Breitbandtransponder | 10491–10499 MHz → 741–749 MHz |

**Fehlerbudget.** Der LO-Fehler ist der Referenzfehler mal 9750 MHz:

| Referenz | LO-Fehler |
|---|---|
| 2,5 ppm (freilaufender Quarz) | ±24,4 kHz |
| 1 ppm | ±9,75 kHz |
| 0,1 ppm | ±975 Hz |

Ein freilaufender LNB-Quarz liegt damit um ein Vielfaches der SSB-Bandbreite
daneben und driftet thermisch dazu — genau deshalb der GPSDO. Mit
GPS-Disziplinierung bleibt der Fehler im Sub-Hz-Bereich.

**Konsistenzcheck zur internen Referenz:** 9750 / 25 = 390 und
10600 / 25 = 424 sind beide ganzzahlig, mit 27 MHz geht keines von beiden auf.
Der PXS-SEW arbeitet also mit hoher Wahrscheinlichkeit auf 25 MHz — beim Öffnen
trotzdem die Quarzbeschriftung prüfen, bevor der Quarz ausgelötet und durch die
Einspeisung ersetzt wird.

**Bias-Tee.** Über dasselbe Koax laufen Speisespannung, ZF und die 25-MHz-
Referenz. Für den Schmalbandtransponder (vertikale Polarisation):

- **13 V** (18 V wäre horizontal, das ist der Breitbandtransponder)
- **22-kHz-Ton aus** — mit Ton schaltet der LNB aufs Highband und der LO
  springt auf 10,600 GHz

## Hardware-Hinweise

- **XA-Einspeisung:** XA ist der Analogeingang des Quarzoszillators, kein
  CMOS-Clock-Eingang. Über 10 nF einkoppeln und auf ca. 0,6–1,0 Vpp dämpfen
  (z. B. 1 k / 470 R). XB bleibt offen. Leitung kurz halten — 24 MHz strahlen.
- **Power-Save muss aus bleiben.** Im PSM setzt der Timepulse aus. Falls das
  Modul aus einem anderen Projekt mit gespeicherter PSM-Konfiguration kommt,
  explizit auf Continuous stellen.
- **24 MHz liegen über den spezifizierten 10 MHz** des NEO-7M-Timepulses. Es
  funktioniert (und wird von diesem wie vom SX1280-Build so betrieben), aber
  die Flanken sind slew-limitiert. Einmal mit dem Scope nachsehen lohnt sich.
- **Phasenrauschen:** 25 MHz → 9,75 GHz LO ist ×390, also +52 dB auf das
  Referenzrauschen. Die Langzeitgenauigkeit wird exzellent, das
  Close-in-Phasenrauschen bleibt prinzipbedingt mäßig — für QO-100-SSB und
  Baken völlig ausreichend, aber kein OCXO-Ersatz.
- **Bei Lock-Verlust** läuft der Ausgang bewusst weiter: das GPS gibt die
  24 MHz dann aus seinem internen Oszillator aus (einige ppm daneben), das LNB
  driftet, fällt aber nicht aus. Wer lieber stummschalten will, setzt in
  `loop()` bei `!gpsLock` das OEB-Register auf `0xFF`.
