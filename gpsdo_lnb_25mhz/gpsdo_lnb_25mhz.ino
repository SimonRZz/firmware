/*
 * GPSDO fuer LNB-Referenzeinspeisung: NEO-7M + Si5351 + Arduino Pro Micro
 * ======================================================================
 * 25 MHz auf CLK1, GPS-diszipliniert ueber den 24-MHz-TIMEPULSE des NEO-7M,
 * der direkt (per Draht, nicht ueber die MCU) auf den XA-Pin des Si5351 geht
 * und dort den ausgeloeteten Quarz ersetzt. Kein PPS-Regelkreis noetig, die
 * Disziplinierung erfolgt rein hardwareseitig ueber den Referenztakt.
 *
 * Portiert aus gpsdo.c des SX1280-QO100-Projekts (dort 52 MHz fuer den
 * SX1280-XTA-Pin). Uebernommen wurden von dort: die feldbewaehrten UBX-
 * Pakete, der I2C-Adressscan, die Baudraten-Erkennung, der NMEA-Parser mit
 * Checksummenpruefung und die Auswertung von Si5351-Register 0. Geaendert
 * wurde nur die PLL-/Multisynth-Konfiguration (25 statt 52 MHz) und die
 * Statusausgabe (LEDs statt NRESET-Gating).
 *
 * KEINE BIBLIOTHEK NOETIG ausser Wire. Der Si5351 wird direkt ueber seine
 * Register konfiguriert. Das ist Absicht: Etherkits si5351.init() pollt das
 * SYS_INIT-Bit in einer do/while-Schleife und haengt ohne I2C-ACK fuer immer
 * (Wire.read() liefert -1 -> 0xFF -> 0xFF>>7 == 1). Ausserdem setzt set_freq()
 * den Integer-Mode nicht und waehlt fuer 25 MHz eine fraktionale PLL.
 *
 * FREQUENZPLAN (beide Teiler ganzzahlig -> geringster Jitter):
 *   PLLA = 24 MHz x 25 = 600 MHz    (Multiplikator 25, integer)
 *   CLK1 = 600 MHz / 24 = 25 MHz    (MS-Teiler 24, gerade, integer)
 * Das ist die einzige Kombination im VCO-Fenster 600-900 MHz, bei der aus
 * 24 MHz Referenz sowohl PLL-Multiplikator als auch Ausgangsteiler ganzzahlig
 * sind. Zum Vergleich: Etherkits set_freq() nimmt VCO = 900 MHz, also 24 x 37,5.
 *
 * BOARD: "SparkFun Pro Micro" bzw. "Arduino Leonardo" (ATmega32U4).
 * Die 3,3V/8MHz-Variante kann direkt verdrahtet werden. Bei der 5V/16MHz-
 * Variante sind Pegelwandler auf UART (D0/D1) und I2C (D2/D3) noetig, da
 * NEO-7M und Si5351 3,3-V-Bausteine sind.
 *
 * STANDALONE: Der Sketch wartet nirgends auf USB ("if (Serial)" abgesichert).
 * Versorgung ohne PC ueber RAW. Der Betriebszustand ist an den LEDs ablesbar,
 * siehe Blinkcodes unten.
 *
 * VERKABELUNG:
 *   NEO-7M TX          -> Pro Micro D0 (RXI, Serial1 RX)
 *   NEO-7M RX          -> Pro Micro D1 (TXO, Serial1 TX)
 *   NEO-7M TIMEPULSE   -> Si5351 XA-Pin (DIREKT, nicht ueber die MCU!)
 *                         ueber 10 nF einkoppeln und auf ca. 0,6-1,0 Vpp
 *                         daempfen (z.B. 1k/470R). XA ist ein Analogeingang
 *                         des Quarzoszillators, kein CMOS-Clock-Eingang.
 *                         XB bleibt offen. Leitung kurz halten.
 *   NEO-7M VCC/GND     -> 3V3/GND (220uF + 100nF Entkopplung an VCC)
 *   Si5351 SDA         -> Pro Micro D2
 *   Si5351 SCL         -> Pro Micro D3
 *   Si5351 VCC         -> 3V3 (100nF Entkopplung direkt am Pin)
 *   Si5351 Quarz       -> AUSGELOETET
 *   Si5351 CLK1        -> Bias-Tee -> LNB-Referenzeingang
 *   Pro Micro D5 -> 330R -> LED rot   -> GND
 *   Pro Micro D6 -> 330R -> LED gruen -> GND
 *
 * LED-CODES:
 *   gruen dauerhaft    Referenz laeuft (PLL locked, 24 MHz am XA) UND GPS-Fix
 *   rot dauerhaft      Referenz laeuft, aber noch kein ausreichender GPS-Fix
 *   rot 1x blinkend    PLL A nicht eingerastet (LOL) bzw. Chip noch im SYS_INIT
 *   rot 2x blinkend    kein Signal am XA-Pin (LOS) - 24 MHz kommen nicht an
 *   rot 3x blinkend    Si5351 nicht auf dem I2C-Bus gefunden
 */

#include <Wire.h>
#include <avr/pgmspace.h>

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------
#define gpsSerial Serial1

static const uint8_t  LED_RED_PIN        = 5;
static const uint8_t  LED_GREEN_PIN      = 6;

// Lock-Kriterium fuer die gruene LED. gpsdo.c im SX1280-Projekt nimmt 3;
// hoehere Werte machen die Anzeige traeger, aber aussagekraeftiger.
static const uint8_t  MIN_SATS_FOR_LOCK  = 5;

static const uint16_t GPS_STALE_MS       = 5000;  // ohne gueltiges NMEA -> kein Lock
static const uint16_t SI_POLL_MS         = 2000;  // Reg-0-Abfrage
static const uint16_t DEBUG_MS           = 1000;  // Heartbeat auf USB-Serial
static const uint8_t  NMEA_BUF_LEN       = 96;

// Zustand der Referenz, abgeleitet aus Si5351-Register 0.
// MUSS vor der ersten Funktionsdefinition stehen: die Arduino-IDE generiert
// Funktionsprototypen per ctags und fuegt sie genau dort ein. Ein Typ, der
// weiter unten definiert wird, ist im Prototyp von refState()/updateLeds()
// sonst noch unbekannt ("'RefState' does not name a type").
enum RefState { REF_FAIL, REF_LOS, REF_LOL, REF_OK };

// ---------------------------------------------------------------------------
// Si5351 - minimaler Registertreiber
// ---------------------------------------------------------------------------
static const uint8_t SI_REG_STATUS       = 0;
static const uint8_t SI_STATUS_SYS_INIT  = (1 << 7);  // 1 = Chip kalibriert noch
static const uint8_t SI_STATUS_LOL_A     = (1 << 5);  // 1 = PLL A nicht eingerastet
static const uint8_t SI_STATUS_LOS_XTAL  = (1 << 3);  // 1 = kein Signal am XA-Pin
static const uint8_t SI_REG_OEB          = 3;         // Output Enable (aktiv LOW)
static const uint8_t SI_REG_CLK0_CTRL    = 16;
static const uint8_t SI_REG_CLK1_CTRL    = 17;
static const uint8_t SI_REG_CLK2_CTRL    = 18;
static const uint8_t SI_REG_PLLA_BASE    = 26;        // Register 26-33
static const uint8_t SI_REG_MS1_BASE     = 50;        // Register 50-57
static const uint8_t SI_REG_PLL_RESET    = 177;
static const uint8_t SI_REG_XTAL_LOAD    = 183;

static uint8_t siAddr = 0x00;  // 0x00 = noch nicht gefunden

static bool siWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(siAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool siWriteRegs(uint8_t base, const uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(siAddr);
    Wire.write(base);
    for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission() == 0;
}

static bool siReadReg(uint8_t reg, uint8_t *val)
{
    Wire.beginTransmission(siAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // Repeated Start
    if (Wire.requestFrom(siAddr, (uint8_t)1) != 1) return false;
    *val = Wire.read();
    return true;
}

// Alle vier gueltigen Si5351-Adressen durchprobieren, 3 Versuche.
static bool siScan()
{
    static const uint8_t candidates[4] = { 0x60, 0x61, 0x62, 0x63 };
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        for (uint8_t i = 0; i < 4; i++) {
            Wire.beginTransmission(candidates[i]);
            if (Wire.endTransmission() == 0) {
                siAddr = candidates[i];
                return true;
            }
        }
        delay(20);
    }
    siAddr = 0x00;
    return false;
}

/*
 * PLLA = 600 MHz aus 24 MHz: a=25, b=0, c=1
 *   p1 = 128*a + floor(128*b/c) - 512 = 3200 - 512 = 2688 = 0x0A80
 *   p2 = 0, p3 = 1
 * MS1 = 24 (600 MHz -> 25 MHz): a=24, b=0, c=1
 *   p1 = 128*24 - 512 = 2560 = 0x0A00
 *   p2 = 0, p3 = 1
 */
static bool si5351Init25MHz()
{
    if (!siScan()) return false;

    // Waehrend der Konfiguration alle Ausgaenge sperren.
    if (!siWriteReg(SI_REG_OEB, 0xFF)) return false;

    // Ungenutzte Ausgaenge abschalten (weniger Strom, weniger Nebenwellen).
    siWriteReg(SI_REG_CLK0_CTRL, 0x80);
    siWriteReg(SI_REG_CLK2_CTRL, 0x80);

    // CLK1: PDN=0, MS1_INT=1 (Integer-Mode!), MS1_SRC=PLLA, INV=0,
    //       CLK1_SRC=MS1=0b11, IDRV=4mA=0b01  ->  0b0100_1101 = 0x4D
    siWriteReg(SI_REG_CLK1_CTRL, 0x4D);

    // Load-Cap 8 pF (Bits[7:6]=0b10) + vorgeschriebene Reserved-Bits 0x12.
    // AN619 fuehrt 0b00 als reserved - deshalb NICHT "0 pF", auch wenn der
    // Quarz ausgeloetet ist und der Wert praktisch keine Rolle spielt.
    siWriteReg(SI_REG_XTAL_LOAD, 0x92);

    // PLL A, Register 26-33:
    //   [p3_hi, p3_lo, p1[17:16], p1_hi, p1_lo, p3[19:16]|p2[19:16], p2_hi, p2_lo]
    static const uint8_t plla[8] = {
        0x00, 0x01,   // p3 = 1
        0x00,         // p1[17:16] = 0
        0x0A, 0x80,   // p1 = 0x0A80 (2688)
        0x00,         // obere Nibbles p3/p2 = 0
        0x00, 0x00    // p2 = 0
    };
    if (!siWriteRegs(SI_REG_PLLA_BASE, plla, 8)) return false;

    // MS1 (CLK1), Register 50-57:
    static const uint8_t ms1[8] = {
        0x00, 0x01,   // p3 = 1
        0x00,         // R_DIV=/1, DIVBY4=0, p1[17:16]=0
        0x0A, 0x00,   // p1 = 0x0A00 (2560)
        0x00,
        0x00, 0x00    // p2 = 0
    };
    if (!siWriteRegs(SI_REG_MS1_BASE, ms1, 8)) return false;

    // Optional und hier bewusst NICHT gesetzt: FBA_INT (Reg 22) schaltet
    // zusaetzlich den Delta-Sigma-Modulator der PLL ab. Der Gewinn ist klein,
    // solange der Multiplikator ohnehin ganzzahlig ist (b=0), und gpsdo.c im
    // laufenden SX1280-Build schreibt dieses Register auch nicht.

    siWriteReg(SI_REG_PLL_RESET, 0xA0);   // PLL A + B zuruecksetzen
    siWriteReg(SI_REG_OEB, 0xFD);         // nur CLK1 freigeben (Bit 1 = 0)
    return true;
}

// ---------------------------------------------------------------------------
// UBX-Pakete (byteweise uebernommen aus gpsdo.c, Checksummen nachgerechnet)
// ---------------------------------------------------------------------------

// CFG-TP5: TIMEPULSE = 24 MHz Dauerstrich, 50% Tastverhaeltnis.
// freqPeriod = freqPeriodLock = 0x016E3600 = 24.000.000
// flags = 0x6F: active | lockGnssFreq | lockedOtherSet | isFreq
//               | alignToTow | polarity
// 24 MHz = 48 MHz / 2 ist ein exakter Teiler des internen Takts des Moduls -
// nur deshalb ist der Timepulse jitterarm. Krumme Frequenzen wie 10 MHz
// werden gedithert und waeren als Referenz deutlich schlechter.
static const uint8_t UBX_CFG_TP5_24MHZ[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36,
    0x6E, 0x01, 0x00, 0x36, 0x6E, 0x01, 0x00, 0x00,
    0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x11, 0xD8
};

// CFG-NAV5: dynModel = 2 (stationary). Fuer eine ortsfeste Referenz sinnvoll,
// verbessert die Zeit- und Taktloesung des Empfaengers.
static const uint8_t UBX_CFG_NAV5_STATIONARY[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
    0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x4E, 0x60
};

static void sendUbxPacket(const uint8_t *pkt, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        gpsSerial.write(pgm_read_byte(&pkt[i]));
    }
    gpsSerial.flush();
}

// ---------------------------------------------------------------------------
// Laufzeitzustand
// ---------------------------------------------------------------------------
static uint8_t  gFixQuality   = 0;      // GGA Feld 6
static uint8_t  gSatsUsed     = 0;      // GGA Feld 7
static char     gUtc[7]       = "------";
static bool     gSiPresent    = false;  // Si5351 per I2C erreichbar
static uint8_t  gSiStatus     = 0xFF;   // Register 0, 0xFF = noch nicht gelesen
static bool     gWasLos       = true;   // fuer den PLL-Reset nach LOS -> OK

static char     gNmeaBuf[NMEA_BUF_LEN];
static uint8_t  gNmeaIdx      = 0;
static bool     gCollecting   = false;
static bool     gOverflow     = false;

static uint32_t gLastNmeaMs   = 0;
static uint32_t gLastSiPollMs = 0;
static uint32_t gLastDebugMs  = 0;
static uint32_t gRxBytes      = 0;
static uint32_t gNmeaCount    = 0;
static uint32_t gGpsBaud      = 9600;

static RefState refState()
{
    if (!gSiPresent)                       return REF_FAIL;
    if (gSiStatus == 0xFF)                 return REF_LOL;  // noch nicht gepollt
    if (gSiStatus & SI_STATUS_LOS_XTAL)    return REF_LOS;
    if (gSiStatus & SI_STATUS_LOL_A)       return REF_LOL;
    if (gSiStatus & SI_STATUS_SYS_INIT)    return REF_LOL;
    return REF_OK;
}

// ---------------------------------------------------------------------------
// NMEA
// ---------------------------------------------------------------------------
static const char *nmeaField(const char *s, uint8_t n)
{
    uint8_t cnt = 0;
    while (*s) {
        if (cnt == n) return s;
        if (*s == '*') break;
        if (*s == ',') cnt++;
        s++;
    }
    return (cnt == n) ? s : NULL;
}

static int8_t hexNibble(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

static bool nmeaChecksumOk(const char *s)
{
    if (!s || s[0] != '$') return false;
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p && *p != '*') { cs ^= (uint8_t)(*p); p++; }
    if (*p != '*' || !p[1] || !p[2]) return false;
    int8_t hi = hexNibble(p[1]);
    int8_t lo = hexNibble(p[2]);
    if (hi < 0 || lo < 0) return false;
    return cs == (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
}

static uint16_t parseUint(const char *p)
{
    if (!p) return 0;
    uint16_t v = 0;
    while (*p >= '0' && *p <= '9') { v = (uint16_t)(v * 10 + (*p - '0')); p++; }
    return v;
}

static void parseTime(const char *s)
{
    const char *f = nmeaField(s, 1);
    if (!f) return;
    for (uint8_t i = 0; i < 6; i++) {
        if (f[i] < '0' || f[i] > '9') return;
    }
    for (uint8_t i = 0; i < 6; i++) gUtc[i] = f[i];
    gUtc[6] = '\0';
}

static void processSentence(const char *s)
{
    if (!nmeaChecksumOk(s)) return;
    gLastNmeaMs = millis();
    gNmeaCount++;
    if (s[3] == 'G' && s[4] == 'G' && s[5] == 'A') {
        parseTime(s);
        gFixQuality = (uint8_t)parseUint(nmeaField(s, 6));
        gSatsUsed   = (uint8_t)parseUint(nmeaField(s, 7));
    } else if (s[3] == 'R' && s[4] == 'M' && s[5] == 'C') {
        parseTime(s);
    }
}

static void processIncomingGps()
{
    while (gpsSerial.available()) {
        char c = (char)gpsSerial.read();
        gRxBytes++;
        if (c == '$') {
            gCollecting = true;
            gOverflow   = false;
            gNmeaIdx    = 0;
            gNmeaBuf[gNmeaIdx++] = c;
            continue;
        }
        if (!gCollecting) continue;
        if (c == '\r' || c == '\n') {
            if (!gOverflow && gNmeaIdx > 6) {
                gNmeaBuf[gNmeaIdx] = '\0';
                processSentence(gNmeaBuf);
            }
            gCollecting = false;
            gOverflow   = false;
            gNmeaIdx    = 0;
            continue;
        }
        if (!gOverflow) {
            if (gNmeaIdx < (NMEA_BUF_LEN - 1)) gNmeaBuf[gNmeaIdx++] = c;
            else                               gOverflow = true;
        }
    }
}

// Baudrate suchen: NEO-7M-Default ist 9600, u-center kann das geaendert haben.
// Jede Rate 400 ms lang testen und auf ein '$' warten. Die dabei vergehende
// Zeit (bis 1,6 s) dient gleichzeitig als Bootverzoegerung fuers GPS-Modul -
// wichtig, weil ein zu frueh gesendetes CFG-TP5 sonst verloren geht.
static uint32_t detectGpsBaud()
{
    static const uint32_t candidates[4] = { 9600, 38400, 57600, 115200 };
    for (uint8_t i = 0; i < 4; i++) {
        gpsSerial.begin(candidates[i]);
        delay(10);
        while (gpsSerial.available()) gpsSerial.read();

        uint32_t tEnd    = millis() + 400;
        uint16_t rxCount = 0;
        bool     dollar  = false;
        while ((int32_t)(millis() - tEnd) < 0) {
            if (gpsSerial.available()) {
                char c = (char)gpsSerial.read();
                rxCount++;
                if (c == '$') dollar = true;
            }
        }
        if (dollar && rxCount >= 10) {
            gRxBytes = rxCount;
            return candidates[i];   // Port bleibt auf dieser Rate offen
        }
        gpsSerial.end();
    }
    gpsSerial.begin(9600);          // Fallback
    return 9600;
}

// ---------------------------------------------------------------------------
// LEDs
// ---------------------------------------------------------------------------
static void updateLeds(RefState ref, bool gpsLock)
{
    if (ref == REF_OK && gpsLock) {
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_RED_PIN, LOW);
        return;
    }
    digitalWrite(LED_GREEN_PIN, LOW);

    if (ref == REF_OK) {            // Referenz laeuft, GPS sucht noch
        digitalWrite(LED_RED_PIN, HIGH);
        return;
    }

    // Fehlercode blinken: n kurze Blitze, dann Pause.
    const uint8_t  flashes = (ref == REF_FAIL) ? 3 : (ref == REF_LOS) ? 2 : 1;
    const uint32_t onWindow = (uint32_t)flashes * 400UL;
    const uint32_t period   = onWindow + 1200UL;
    const uint32_t t        = millis() % period;
    bool on = false;
    if (t < onWindow) on = (t % 400UL) < 150UL;
    digitalWrite(LED_RED_PIN, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
void setup()
{
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);

    // USB-CDC oeffnen, aber NICHT darauf warten - der GPSDO laeuft ohne Host.
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(100000);          // 100 kHz, zuverlaessig mit internen Pull-ups

    // Erst das GPS auf 24 MHz konfigurieren, dann den Si5351 - so liegt beim
    // PLL-Reset schon eine Referenz am XA-Pin an.
    gGpsBaud = detectGpsBaud();
    sendUbxPacket(UBX_CFG_TP5_24MHZ, sizeof(UBX_CFG_TP5_24MHZ));
    delay(50);
    sendUbxPacket(UBX_CFG_NAV5_STATIONARY, sizeof(UBX_CFG_NAV5_STATIONARY));
    delay(200);

    gSiPresent = si5351Init25MHz();
    if (gSiPresent) {
        delay(50);                  // PLL Zeit zum Einrasten geben
        siReadReg(SI_REG_STATUS, &gSiStatus);
        gWasLos = (gSiStatus & SI_STATUS_LOS_XTAL) != 0;
    }
    gLastSiPollMs = millis();

    if (Serial) {
        Serial.print(F("[GPSDO] GPS-Baud="));
        Serial.println(gGpsBaud);
        if (gSiPresent) {
            Serial.print(F("[GPSDO] Si5351 @0x"));
            Serial.print(siAddr, HEX);
            Serial.print(F(" reg0=0x"));
            Serial.println(gSiStatus, HEX);
        } else {
            Serial.println(F("[GPSDO] Si5351 nicht gefunden (I2C)"));
        }
    }
}

void loop()
{
    processIncomingGps();

    // Si5351-Status regelmaessig nachlesen. Erscheint die Referenz erst
    // nachtraeglich (GPS braucht laenger als der Sketch), einmalig einen
    // PLL-Reset nachschieben, damit die PLL sicher auf die 24 MHz einrastet.
    if (millis() - gLastSiPollMs >= SI_POLL_MS) {
        gLastSiPollMs = millis();
        if (siAddr != 0x00) {
            uint8_t st;
            if (siReadReg(SI_REG_STATUS, &st)) {
                gSiPresent = true;
                gSiStatus  = st;
                const bool isLos = (st & SI_STATUS_LOS_XTAL) != 0;
                if (gWasLos && !isLos) {
                    siWriteReg(SI_REG_PLL_RESET, 0xA0);
                    if (Serial) Serial.println(F("[GPSDO] Referenz da - PLL-Reset"));
                }
                gWasLos = isLos;
            } else {
                gSiPresent = false;
            }
        }
    }

    const bool nmeaFresh = (gLastNmeaMs != 0) && (millis() - gLastNmeaMs < GPS_STALE_MS);
    const bool gpsLock   = nmeaFresh && (gFixQuality > 0) && (gSatsUsed >= MIN_SATS_FOR_LOCK);
    const RefState ref   = refState();

    updateLeds(ref, gpsLock);

    // Heartbeat: laeuft unabhaengig davon, ob GPS-Pakete ankommen, und zeigt
    // damit auch bei falsch verdrahtetem GPS, dass der Sketch ueberhaupt laeuft.
    if (millis() - gLastDebugMs >= DEBUG_MS) {
        gLastDebugMs = millis();
        if (Serial) {
            Serial.print(F("uptime="));
            Serial.print(millis() / 1000);
            Serial.print(F("s rx="));
            Serial.print(gRxBytes);
            Serial.print(F(" nmea="));
            Serial.print(gNmeaCount);
            Serial.print(F(" fix="));
            Serial.print(gFixQuality);
            Serial.print(F(" sats="));
            Serial.print(gSatsUsed);
            Serial.print(F(" utc="));
            Serial.print(gUtc);
            Serial.print(F(" clk1="));
            switch (ref) {
                case REF_FAIL: Serial.print(F("fail")); break;
                case REF_LOS:  Serial.print(F("los"));  break;
                case REF_LOL:  Serial.print(F("lol"));  break;
                default:       Serial.print(F("ok"));   break;
            }
            Serial.println(gpsLock ? F(" LOCK") : F(" ---"));
        }
    }
}
