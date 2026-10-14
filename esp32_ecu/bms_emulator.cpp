// =============================================================================
// bms_emulator.cpp — Emulador BMS Xiaomi M365 (Basado en CamiAlfa M365_BMS_EMULATOR)
// =============================================================================

#include "bms_emulator.h"
#include "config.h"
#include "m365_protocol.h"

#include <SoftwareSerial.h>

namespace BmsEmulator {

// BMS Emulator en SoftwareSerial (D34 RX / D33 TX a 76800 baud) con buffer ISR dedicado
static SoftwareSerial bmsSerial(PIN_BMS_RX, PIN_BMS_TX);

// Memoria virtual del BMS (128 words de 16 bits = 256 bytes)
static constexpr uint8_t BMS_MEM_SIZE = 128;
static volatile uint16_t _memory[BMS_MEM_SIZE];

// Contadores de diagnóstico
static uint32_t _queriesReceived = 0;
static uint32_t _repliesSent     = 0;
static uint32_t _lastQueryTime   = 0;

// Registros estándar de batería Xiaomi M365 (CamiAlfa)
static constexpr uint8_t BATT_SERIAL_REG     = 0x10; // 14 bytes (7 words)
static constexpr uint8_t BATT_BMS_VER_REG    = 0x17; // Version: 0x0115 (v1.1.5)
static constexpr uint8_t BATT_FAB_CAP_REG    = 0x18; // Capacidad de fábrica en mAh
static constexpr uint8_t BATT_REAL_CAP_REG   = 0x19; // Capacidad real en mAh
static constexpr uint8_t BATT_CYCLES_REG     = 0x1B; // Ciclos de descarga
static constexpr uint8_t BATT_CHARGES_REG    = 0x1C; // Ciclos de carga
static constexpr uint8_t BATT_MAX_VOLT_REG   = 0x1D; // Voltaje máximo (4200 cV)
static constexpr uint8_t BATT_DATE_REG       = 0x20; // Fecha de fabricación
static constexpr uint8_t BATT_FLAGS_REG      = 0x30; // Flags de estado (0x0001 / 0x0003 = normal)
static constexpr uint8_t BATT_REMAIN_CAP_REG = 0x31; // mAh restantes
static constexpr uint8_t BATT_PERCENT_REG    = 0x32; // Porcentaje (0-100%)
static constexpr uint8_t BATT_CURRENT_REG    = 0x33; // Corriente en cA (0)
static constexpr uint8_t BATT_VOLT_REG       = 0x34; // Voltaje en cV (3610 = 36.10V)
static constexpr uint8_t BATT_TEMP_REG       = 0x35; // Temp1 & Temp2 (+20 grados)
static constexpr uint8_t BATT_HEALTH_REG     = 0x3B; // Salud (0-100, 95% = excelente)
static constexpr uint8_t BATT_PACK1_REG      = 0x40; // Voltaje celdas 1..10 en mV

// Auto-baud detection (76800 para BotoX Open-Source BMS, o 115200 stock)
static uint32_t _currentBaud     = 76800;
static uint32_t _lastBaudSwitch  = 0;
static bool     _baudLocked      = false;

// Framer secuencial robusto para paquetes BMS M365 (0x55 0xAA)
struct BmsFramer {
    uint8_t  buf[96];
    uint8_t  idx = 0;
    uint32_t lastByteTime = 0;

    void reset() { idx = 0; }
};
static BmsFramer _framer;

static uint16_t sendSerialByte(uint8_t data) {
    bmsSerial.write(data);
    return (uint16_t)data;
}

static uint16_t sendSerial16bLsb(uint16_t data) {
    return sendSerialByte((uint8_t)(data & 0xFF));
}

static uint16_t sendSerial16bMsb(uint16_t data) {
    return sendSerialByte((uint8_t)((data >> 8) & 0xFF));
}

static uint16_t sendSerialMem(uint8_t target, uint16_t numBytes) {
    uint8_t isOdd = numBytes & 0x01;
    uint8_t wordCount = numBytes >> 1;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < wordCount; i++) {
        uint8_t addr = (target + i) & 0x7F;
        sum += sendSerial16bLsb(_memory[addr]);
        sum += sendSerial16bMsb(_memory[addr]);
    }
    if (isOdd) {
        uint8_t addr = (target + wordCount) & 0x7F;
        sum += sendSerial16bLsb(_memory[addr]);
    }
    return sum;
}

void init() {
    bmsSerial.begin(76800, SWSERIAL_8N1, PIN_BMS_RX, PIN_BMS_TX, false, 256, 256);

    // Inicializar memoria emulada estándar Xiaomi M365 (BotoX / CamiAlfa)
    for (uint8_t i = 0; i < BMS_MEM_SIZE; i++) {
        _memory[i] = 0;
    }

    // Cabecera estándar BotoX M365BMS
    _memory[0] = 0x5A;
    _memory[1] = 0x5A;

    // Número de serie oficial Xiaomi (empieza por '3J'): "3JCamiAlfa_BMS"
    _memory[BATT_SERIAL_REG + 0] = ((uint16_t)'3') | (((uint16_t)'J') << 8);
    _memory[BATT_SERIAL_REG + 1] = ((uint16_t)'C') | (((uint16_t)'a') << 8);
    _memory[BATT_SERIAL_REG + 2] = ((uint16_t)'m') | (((uint16_t)'i') << 8);
    _memory[BATT_SERIAL_REG + 3] = ((uint16_t)'A') | (((uint16_t)'l') << 8);
    _memory[BATT_SERIAL_REG + 4] = ((uint16_t)'f') | (((uint16_t)'a') << 8);
    _memory[BATT_SERIAL_REG + 5] = ((uint16_t)'_') | (((uint16_t)'B') << 8);
    _memory[BATT_SERIAL_REG + 6] = ((uint16_t)'M') | (((uint16_t)'S') << 8);

    // Versión firmware BMS: v1.1.5 (0x0115)
    _memory[BATT_BMS_VER_REG] = 0x0115;

    // Capacidad de fábrica y real: 7800 mAh
    _memory[BATT_FAB_CAP_REG]  = 7800;
    _memory[BATT_REAL_CAP_REG] = 7800;

    // Ciclos y cargas
    _memory[BATT_CYCLES_REG]   = 5;
    _memory[BATT_CHARGES_REG]  = 6;
    _memory[BATT_MAX_VOLT_REG] = 4200; // 42.00V

    // Fecha fabricación: 2020-08-31
    _memory[BATT_DATE_REG] = (((uint16_t)20) << 9) | (((uint16_t)8) << 5) | (31);

    // Flags: 0x0003 (Config válida, descarga habilitada)
    _memory[BATT_FLAGS_REG] = 0x0003;

    // Salud de batería: 95%
    _memory[BATT_HEALTH_REG] = 95;

    // Temperatura: 25°C celda 1, 26°C celda 2 (+20 de offset según protocolo = 45 / 46)
    _memory[BATT_TEMP_REG] = (((uint16_t)(25 + 20)) << 8) | ((uint16_t)(26 + 20));

    // Corriente inicial: 0
    _memory[BATT_CURRENT_REG] = 0;

    // Batería inicial: 36.1V y 85%
    setBatteryVoltage(36.1f);
    setBatteryPercentage(85);

    Serial.println("[BMS_EMU] ✓ Emulador BMS M365 (BotoX) inicializado a 76800 baud.");
    Serial.printf("[BMS_EMU]   Pines: RX=GPIO%d (Amarillo ESC TX) | TX=GPIO%d (Blanco ESC RX)\n", PIN_BMS_RX, PIN_BMS_TX);
}

void setBatteryVoltage(float volts) {
    uint16_t cV = (uint16_t)(volts * 100.0f); // centivoltios (ej: 3610 cV = 36.10V)
    _memory[BATT_VOLT_REG] = cV;

    // Calcular voltaje de celda en mV (ej: 3610 mV) para 10S
    uint16_t cellMv = (uint16_t)((volts / 10.0f) * 1000.0f);
    for (uint8_t i = 0; i < 10; i++) {
        _memory[BATT_PACK1_REG + i] = cellMv;
    }
}

void setBatteryPercentage(uint8_t percent) {
    if (percent > 100) percent = 100;
    _memory[BATT_PERCENT_REG] = percent;
    _memory[BATT_REMAIN_CAP_REG] = (uint16_t)(((uint32_t)percent * 7800) / 100);
}

void setCapacity(uint16_t mah) {
    _memory[BATT_FAB_CAP_REG] = mah;
    _memory[BATT_REAL_CAP_REG] = mah;
}

static uint32_t _rawBytesReceived = 0;
static uint32_t _lastDiagTime      = 0;
static uint32_t _badChecksums     = 0;

uint32_t getQueriesReceived() { return _queriesReceived; }
uint32_t getRepliesSent()     { return _repliesSent; }
bool     isActive()           { return (_lastQueryTime > 0 && (millis() - _lastQueryTime < 3000)); }

void update() {
    uint32_t now = millis();

    // Timeout de paquete parcial (>25ms sin bytes)
    if (_framer.idx > 0 && (now - _framer.lastByteTime > 25)) {
        _framer.reset();
    }

    // 1. Leer bytes entrantes desde la controladora (ESC TX)
    while (bmsSerial.available()) {
        uint8_t b = (uint8_t)bmsSerial.read();
        _rawBytesReceived++;

        if (_framer.idx == 0) {
            if (b == 0x55) {
                _framer.buf[_framer.idx++] = b;
                _framer.lastByteTime = now;
            }
            continue;
        }

        if (_framer.idx == 1) {
            if (b == 0xAA) {
                _framer.buf[_framer.idx++] = b;
                _framer.lastByteTime = now;
            } else {
                _framer.reset();
                if (b == 0x55) {
                    _framer.buf[_framer.idx++] = b;
                    _framer.lastByteTime = now;
                }
            }
            continue;
        }

        _framer.buf[_framer.idx++] = b;
        _framer.lastByteTime = now;

        uint8_t expectedTotal = _framer.buf[2] + 6;
        if (expectedTotal < 8 || expectedTotal > sizeof(_framer.buf)) {
            _framer.reset();
            continue;
        }

        if (_framer.idx == expectedTotal) {
            uint8_t* p = _framer.buf;

            // Validar checksum M365 (Suma desde p[2] hasta p[expectedTotal - 3], XOR 0xFFFF)
            uint16_t sum = 0;
            for (uint8_t i = 2; i < expectedTotal - 2; i++) {
                sum += p[i];
            }
            sum ^= 0xFFFF;

            uint16_t pktCk = (uint16_t)p[expectedTotal - 2] | ((uint16_t)p[expectedTotal - 1] << 8);

            uint8_t dest = p[3];
            uint8_t type = p[4];
            uint8_t reg  = p[5];
            uint8_t len  = p[6];

            bool ckOk = (sum == pktCk);

            if (ckOk) {
                // Aceptar cualquier consulta de lectura dirigida al puerto BMS
                if (type == 0x01) {
                    _queriesReceived++;
                    _lastQueryTime = now;

                    uint16_t outCk = 0;
                    sendSerialByte(0x55);
                    sendSerialByte(0xAA);
                    outCk += sendSerialByte(len + 2); // Length
                    outCk += sendSerialByte(0x25);    // Destino: BATTtoMaster (0x25)
                    outCk += sendSerialByte(0x01);    // Tipo: Read response
                    outCk += sendSerialByte(reg);     // Registro
                    outCk += sendSerialMem(reg, len);
                    outCk ^= 0xFFFF;
                    sendSerial16bLsb(outCk);
                    sendSerial16bMsb(outCk);
                    bmsSerial.flush(); // Enviar completamente por UART Hardware

                    _repliesSent++;
                }
            } else {
                _badChecksums++;
            }
            _framer.reset();
        }
    }
}

} // namespace BmsEmulator
