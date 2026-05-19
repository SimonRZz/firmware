#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RCWL9620Sensor.h"
#include "TelemetrySensor.h"

RCWL9620Sensor::RCWL9620Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_RCWL9620, "RCWL9620") {}

bool RCWL9620Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("Init sensor: %s", sensorName);
    status = 1;
    begin(bus, dev->address.address);
    initI2CSensor();
    return status;
}

bool RCWL9620Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    measurement->variant.environment_metrics.has_distance = true;
    LOG_DEBUG("RCWL9620 getMetrics");
    measurement->variant.environment_metrics.distance = getDistance();
    return true;
}

void RCWL9620Sensor::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed)
{
    _wire = wire;
    _addr = addr;
    _sda = sda;
    _scl = scl;
    _speed = speed;
    _wire->begin();
}

float RCWL9620Sensor::getDistance()
{
    uint32_t data = 0;
    uint8_t b1 = 0, b2 = 0, b3 = 0;

    LOG_DEBUG("[RCWL9620] Start measure command");

    _wire->beginTransmission(_addr);
    _wire->write(0x01); // À tester aussi sans cette ligne si besoin
    uint8_t result = _wire->endTransmission();
    LOG_DEBUG("[RCWL9620] endTransmission result = %d", result);
    delay(120); // RCWL9620 needs ~120ms for a measurement

    LOG_DEBUG("[RCWL9620] Read i2c data:");
    _wire->requestFrom(_addr, (uint8_t)3);

    if (_wire->available() < 3) {
        LOG_DEBUG("[RCWL9620] less than 3 octets !");
#if defined(ARCH_NRF52) && defined(PIN_WIRE_SDA) && defined(PIN_WIRE_SCL)
        // Sensor likely holding SDA low — recover the bus so the next
        // measurement (and any other I2C traffic) is not blocked.
        _wire->end();
        pinMode(PIN_WIRE_SCL, OUTPUT);
        pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
        digitalWrite(PIN_WIRE_SCL, HIGH);
        delayMicroseconds(5);
        for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_WIRE_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(PIN_WIRE_SCL, HIGH);
            delayMicroseconds(5);
            if (digitalRead(PIN_WIRE_SDA) == HIGH)
                break;
        }
        pinMode(PIN_WIRE_SDA, OUTPUT);
        digitalWrite(PIN_WIRE_SDA, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_WIRE_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_WIRE_SDA, HIGH);
        delayMicroseconds(5);
        pinMode(PIN_WIRE_SCL, INPUT);
        pinMode(PIN_WIRE_SDA, INPUT);
        delay(10);
        _wire->begin();
#endif
        return 0.0;
    }

    b1 = _wire->read();
    b2 = _wire->read();
    b3 = _wire->read();

    data = ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;

    float Distance = float(data) / 1000.0;

    LOG_DEBUG("[RCWL9620] Bytes readed = %02X %02X %02X", b1, b2, b3);
    LOG_DEBUG("[RCWL9620] data=%.2f, level=%.2f", (double)data, (double)Distance);

    if (Distance > 4500.00) {
        return 4500.00;
    } else {
        return Distance;
    }
}

#endif
