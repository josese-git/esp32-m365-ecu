// =============================================================================
// m365_protocol.cpp — Implementación del Protocolo UART Xiaomi M365
// =============================================================================
//
// ENTORNO FÍSICO Y CIRCUITO DE ENCENDIDO:
//   En el patinete Xiaomi M365 original, la controladora (ESC) se encuentra
//   apagada en reposo (regulador buck 5V desactivado, microcontrolador STM32 sin
//   alimentación).
//   El cable que conecta la pantalla con la controladora tiene 4 pines (G-T-P-5):
//     - G: GND (Masa)
//     - 5: 5V (Alimentación pantalla cuando la controladora está encendida)
//     - T: UART Data (Comunicación half-duplex 115200 baud)
//     - P: Power Switch / Botón físico conectado directamente a GND al pulsar
//
//   Para encender la controladora desde el ESP32:
//     1. Se tira a nivel LOW (GND) durante ~350ms la línea conectada al pin P
//        (simula la pulsación del botón físico original).
//     2. Se restaura el modo UART a 115200 baud.
//     3. Se espera ~150ms a que el regulador y el STM32 arranquen.
//     4. Se envía la trama UART de UNLOCK / WAKE (Registro 0x71).
//     5. Se mantiene un Heartbeat periódico cada 50ms (lectura reg 0x70) para
//        evitar que la controladora se apague automáticamente tras 35 segundos.
// =============================================================================

#include "m365_protocol.h"
#include "uart_bridge.h"
#include "config.h"
#include <Arduino.h>

namespace M365Protocol {

// ---------------------------------------------------------------------------
// Constantes de protocolo
// ---------------------------------------------------------------------------
static const uint8_t M365_HDR_0     = 0x55;
static const uint8_t M365_HDR_1     = 0xAA;
static const uint8_t M365_DEST_ESC  = 0x20;  // Master → ESC
static const uint8_t M365_TYPE_READ = 0x01;  // Lectura
static const uint8_t M365_TYPE_WRIT = 0x03;  // Escritura

static const uint8_t M365_REG_ALIVE = 0x70;  // Estado general / Heartbeat / Lock
static const uint8_t M365_REG_POWER = 0x71;  // Wake / Unlock
static const uint8_t M365_REG_SPEED = 0x7B;  // Modo de conducción (0=ECO, 1=DRIVE, 2=SPORT)
static const uint8_t M365_REG_LIGHT = 0x7D;  // Luz trasera / delantera (0=OFF, 2=ON)

// Tramas verificadas estándar M365:
// UNLOCK (reg 0x71 = 1): 55 AA 04 20 03 71 01 00 66 FF
static const uint8_t PKT_UNLOCK[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x71, 0x01, 0x00, 0x66, 0xFF };

// LOCK / STANDBY (reg 0x70 = 1): 55 AA 04 20 03 70 01 00 67 FF
static const uint8_t PKT_LOCK[]   = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x70, 0x01, 0x00, 0x67, 0xFF };

// Modo de conducción oficial M365 / Ninebot (reg 0x75 — NB_CTL_WORKMODE / M365ecorREG):
// 0 = Normal / Drive, 1 = ECO, 2 = Sport
static const uint8_t PKT_REG75_DRIVE[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x75, 0x00, 0x00, 0x63, 0xFF };
static const uint8_t PKT_REG75_ECO[]   = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x75, 0x01, 0x00, 0x62, 0xFF };
static const uint8_t PKT_REG75_SPORT[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x75, 0x02, 0x00, 0x61, 0xFF };

// Modos de velocidad / KERS (reg 0x7B):
// ECO   (0x00): 55 AA 04 20 03 7B 00 00 5D FF
// DRIVE (0x01): 55 AA 04 20 03 7B 01 00 5C FF
// SPORT (0x02): 55 AA 04 20 03 7B 02 00 5B FF
static const uint8_t PKT_ECO[]   = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x7B, 0x00, 0x00, 0x5D, 0xFF };
static const uint8_t PKT_DRIVE[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x7B, 0x01, 0x00, 0x5C, 0xFF };
static const uint8_t PKT_SPORT[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x7B, 0x02, 0x00, 0x5B, 0xFF };

// Luces (reg 0x7D):
// LIGHT ON  (0x02): 55 AA 04 20 03 7D 02 00 59 FF
// LIGHT OFF (0x00): 55 AA 04 20 03 7D 00 00 5B FF
static const uint8_t PKT_LIGHT_ON[]  = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x7D, 0x02, 0x00, 0x59, 0xFF };
static const uint8_t PKT_LIGHT_OFF[] = { 0x55, 0xAA, 0x04, 0x20, 0x03, 0x7D, 0x00, 0x00, 0x5B, 0xFF };

// Heartbeat periódico (Lectura de 2 registros en 0x70):
// 55 AA 03 20 01 70 02 69 FF
static const uint8_t PKT_HEARTBEAT[] = { 0x55, 0xAA, 0x03, 0x20, 0x01, 0x70, 0x02, 0x69, 0xFF };

// Buffer máximo para tramas dinámicas
static const uint8_t PKT_BUF_SIZE = 32;

// ---------------------------------------------------------------------------
// Máquina de estados de encendido (FSM no bloqueante)
// ---------------------------------------------------------------------------
enum PowerFsmState {
    PWR_OFF,
    PWR_PULSE_LOW,      // Generando pulso a GND en pin P (~350ms)
    PWR_WAIT_BOOT,      // Esperando que arranque el STM32 del ESC (~150ms)
    PWR_INIT_HANDSHAKE, // Enviando tramas UNLOCK y modo inicial
    PWR_RUNNING         // Controladora activa con Heartbeat continuo
};

// ---------------------------------------------------------------------------
// Estado interno
// ---------------------------------------------------------------------------
static PowerFsmState _pwrState          = PWR_OFF;
static uint32_t      _pwrStateTime      = 0;
static bool          _keepAlive         = false;
static uint32_t      _lastHeartbeat     = 0;
static DriveMode     _activeMode        = MODE_ECO;
static uint8_t       _pktBuf[PKT_BUF_SIZE];
static uint8_t       _currentThrottle   = M365_THROTTLE_REST;
static uint8_t       _currentBrake      = M365_BRAKE_REST;
static uint32_t      _lastThrottleSend  = 0;

// ---------------------------------------------------------------------------
// calcChecksum() — Checksum M365
// Suma todos los bytes desde el campo L hasta el fin de datos y XOR con 0xFFFF
// ---------------------------------------------------------------------------
uint16_t calcChecksum(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum ^ 0xFFFF;
}

// ---------------------------------------------------------------------------
// buildPacket() — Construye un paquete M365 genérico
// ---------------------------------------------------------------------------
uint8_t buildPacket(uint8_t*       outBuf,
                    uint8_t        dest,
                    uint8_t        type,
                    uint8_t        reg,
                    const uint8_t* payload,
                    uint8_t        payloadLen) {

    uint8_t L = 1 + 1 + payloadLen;  // type(1) + reg(1) + payloadLen

    uint8_t idx = 0;
    outBuf[idx++] = M365_HDR_0;  // 0x55
    outBuf[idx++] = M365_HDR_1;  // 0xAA
    outBuf[idx++] = L;
    outBuf[idx++] = dest;
    outBuf[idx++] = type;
    outBuf[idx++] = reg;
    for (uint8_t i = 0; i < payloadLen; i++) {
        outBuf[idx++] = payload[i];
    }

    uint16_t ck = calcChecksum(&outBuf[2], idx - 2);
    outBuf[idx++] = (uint8_t)(ck & 0xFF);        // CK0 = LSB
    outBuf[idx++] = (uint8_t)((ck >> 8) & 0xFF); // CK1 = MSB

    return idx;
}

// ---------------------------------------------------------------------------
// _sendHeartbeat() — Envía un heartbeat periódico al ESC
// ---------------------------------------------------------------------------
static void _sendHeartbeat() {
    UartBridge::sendToController(PKT_HEARTBEAT, sizeof(PKT_HEARTBEAT));
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------
void init() {
    _pwrState         = PWR_OFF;
    _keepAlive        = false;
    _lastHeartbeat    = 0;
    _activeMode       = MODE_ECO;
    _currentThrottle  = M365_THROTTLE_REST;
    _currentBrake     = M365_BRAKE_REST;

    Serial.println("[M365] ✓ Módulo de protocolo M365 inicializado.");
    Serial.println("[M365]   Estado: APAGADO / STANDBY (Throttle/Brake descansando en 0x26)");
}

static uint32_t _pwrPulseDuration = 600;

// ---------------------------------------------------------------------------
// powerOn() — Dispara la secuencia completa de encendido
// ---------------------------------------------------------------------------
void powerOn() {
    triggerPowerPulse(600);
}

// ---------------------------------------------------------------------------
// triggerPowerPulse() — Genera un pulso directo a GND en pin D14
// ---------------------------------------------------------------------------
void triggerPowerPulse(uint32_t durationMs) {
    Serial.printf("[M365] ► PULSO EN D14 (Pin P controladora) A GND (%u ms)...\n", durationMs);
    UartBridge::setControllerPowerPin(true); // D14 a LOW (GND)
    _pwrPulseDuration = durationMs;
    _pwrState         = PWR_PULSE_LOW;
    _pwrStateTime     = millis();
}

// ---------------------------------------------------------------------------
// powerOff() — Apaga la controladora limpiamente
// ---------------------------------------------------------------------------
void powerOff() {
    Serial.println("[M365] ■ APAGANDO CONTROLADORA: Enviando LOCK (0x70) y deteniendo heartbeat...");
    UartBridge::sendToController(PKT_LOCK, sizeof(PKT_LOCK));
    _keepAlive = false;
    _pwrState  = PWR_OFF;
}

// ---------------------------------------------------------------------------
// isPowerActive()
// ---------------------------------------------------------------------------
bool isPowerActive() {
    return (_pwrState == PWR_RUNNING);
}

// ---------------------------------------------------------------------------
// sendSpeedMode() — Inyecta paquete de modo de velocidad
// ---------------------------------------------------------------------------
void sendSpeedMode(DriveMode mode) {
    _activeMode = mode;
    Serial.printf("[M365] ► Modo %d solicitado. Disparando doble clic físico simulado en ESC...\n", mode);
    UartBridge::simulateDoubleClick();
}

// ---------------------------------------------------------------------------
// sendLight() — Inyecta comando de luces (ON / OFF)
// ---------------------------------------------------------------------------
void sendLight(bool enable) {
    if (enable) {
        UartBridge::sendToController(PKT_LIGHT_ON, sizeof(PKT_LIGHT_ON));
    } else {
        UartBridge::sendToController(PKT_LIGHT_OFF, sizeof(PKT_LIGHT_OFF));
    }
}

static bool _directThrottleActive = false;

void setThrottle(uint8_t val) {
    _currentThrottle = max((uint8_t)M365_THROTTLE_REST, val);
    _directThrottleActive = false;
}
uint8_t getThrottle() { return _currentThrottle; }

void setDirectThrottle(uint8_t val) {
    if (val <= M365_THROTTLE_REST) {
        _currentThrottle = M365_THROTTLE_REST;
        _directThrottleActive = false;
    } else {
        _currentThrottle = val;
        _directThrottleActive = true;
    }
}
bool isDirectThrottleActive() { return _directThrottleActive; }

void setBrake(uint8_t val)    { _currentBrake = max((uint8_t)M365_BRAKE_REST, val); }
uint8_t getBrake()            { return _currentBrake; }

// ---------------------------------------------------------------------------
// sendThrottle() — Inyecta trama periódica de acelerador y freno (0x65)
// ---------------------------------------------------------------------------
void sendThrottle(uint8_t throttle, uint8_t brake) {
    uint8_t pkt[13] = {
        0x55, 0xAA, 0x07, 0x20, 0x65, 0x00, 0x04,
        throttle, brake, 0x00, 0x00, 0x00, 0x00
    };
    uint16_t ck = calcChecksum(&pkt[2], 9);
    pkt[11] = (uint8_t)(ck & 0xFF);
    pkt[12] = (uint8_t)((ck >> 8) & 0xFF);
    UartBridge::sendToController(pkt, sizeof(pkt));
}

// ---------------------------------------------------------------------------
// setKeepAlive() / isKeepAliveActive()
// ---------------------------------------------------------------------------
void setKeepAlive(bool enable) {
    if (enable) {
        powerOn();
    } else {
        powerOff();
    }
}

bool isKeepAliveActive() {
    return isPowerActive();
}

// ---------------------------------------------------------------------------
// update() — FSM principal no bloqueante
// ---------------------------------------------------------------------------
void update() {
    uint32_t now = millis();

    switch (_pwrState) {
        case PWR_OFF:
            break;

        case PWR_PULSE_LOW:
            // ¿Completó el tiempo del pulso a GND?
            if (now - _pwrStateTime >= _pwrPulseDuration) {
                UartBridge::setControllerPowerPin(false); // D14 liberado a Alta Impedancia (INPUT)
                _pwrState     = PWR_WAIT_BOOT;
                _pwrStateTime = now;
                Serial.println("[M365] ✓ D14 liberado (High-Z). Esperando arranque STM32 de controladora (200ms)...");
            }
            break;

        case PWR_WAIT_BOOT:
            // ¿Arrancó el STM32 de la controladora?
            if (now - _pwrStateTime >= 200) {
                _pwrState = PWR_INIT_HANDSHAKE;
                if (!UartBridge::isDashboardActive()) {
                    Serial.println("[M365] ► Sin pantalla: Enviando UNLOCK (0x71) y modo inicial...");
                    UartBridge::sendToController(PKT_UNLOCK, sizeof(PKT_UNLOCK));
                    delayMicroseconds(200);
                    sendSpeedMode(_activeMode);
                    _keepAlive = true;
                } else {
                    Serial.println("[M365] ✓ Pantalla detectada: Handshake puro delegado a la pantalla original.");
                }
                _lastHeartbeat    = now;
                _lastThrottleSend = now;
                _pwrState         = PWR_RUNNING;
                Serial.println("[M365] ★ CONTROLADORA ACTIVA. Emulando acelerador y freno.");
            }
            break;

        case PWR_INIT_HANDSHAKE:
            break;

        case PWR_RUNNING:
            // SÓLO inyectar acelerador y heartbeat autónomos SI LA PANTALLA ORIGINAL NO ESTÁ PRESENTE.
            // Si la pantalla física está conectada, el puente UART (uart_bridge) ya se encarga
            // de interceptar, sanear y silenciar errores sin colisiones en el bus monohilo.
            static bool _screenEverSeen = false;
            if (UartBridge::isDashboardActive()) {
                _screenEverSeen = true;
            }

            if (!UartBridge::isDashboardActive() && !_screenEverSeen) {
                if (now - _lastThrottleSend >= 25) {
                    _lastThrottleSend = now;
                    sendThrottle(_currentThrottle, _currentBrake);
                }

                if (_keepAlive && (now - _lastHeartbeat >= M365_HEARTBEAT_MS)) {
                    _lastHeartbeat = now;
                    _sendHeartbeat();
                }
            }
            break;
    }
}

} // namespace M365Protocol
