// =============================================================================
// uart_bridge.cpp — Passthrough Puro con Emulación Anti-Error 14 (Gas y Freno)
// =============================================================================
//
// FUNCIONALIDAD:
//   1. Enrutamiento directo y transparente entre Pantalla (D13) y Controladora (D27).
//   2. Filtro de eco determinista para buses monohilo (Half-Duplex 1-wire).
//   3. Reflejo directo por hardware del botón físico (D32 Pantalla -> D14 ESC).
//   4. Emulación activa de Acelerador y Freno en reposo (0x26):
//      Intercepta las tramas salientes de la pantalla hacia la controladora (0x20).
//      Si los sensores de gas o freno marcan < 0x26 (desconectados físicamente),
//      los fija en 0x26 (38 dec = reposo 0%) y recalcula el checksum exacto.
//      Esto elimina el ERROR 14 y los pitidos de fallo en la controladora.
// =============================================================================

#include "uart_bridge.h"
#include "config.h"
#include "m365_protocol.h"
#include "ui_manager.h"
#include "led_controller.h"

namespace UartBridge {

static HardwareSerial* sDash = &Serial1;
static HardwareSerial* sCtrl = &Serial2;

// Estadísticas de actividad
static uint32_t _bytesFromDash    = 0;
static uint32_t _bytesFromCtrl    = 0;
static uint32_t _pktsSanitized    = 0;
static uint32_t _pktsBattSpoofed  = 0;
static uint32_t _lastDashActivity = 0;
static uint32_t _lastCtrlActivity = 0;
static uint8_t  _lastThrottlePct  = 0;
static bool     _lastBrakeActive   = false;
static uint8_t  _smoothedGas       = M365Protocol::M365_THROTTLE_REST;

// Filtros de eco para bus monohilo (Half-Duplex 1-wire)
static uint32_t _ctrlEchoExpected = 0;
static uint32_t _dashEchoExpected = 0;
static uint32_t _lastTxCtrlTime   = 0;
static uint32_t _lastTxDashTime   = 0;

// Pulso manual de encendido
static uint32_t _manualPulseEnd   = 0;

// Máquina de estados no bloqueante para doble clic simulado en pin P (D14)
enum DblClickState {
    DBL_IDLE,
    DBL_PULSE1,
    DBL_GAP,
    DBL_PULSE2
};
static DblClickState _dblState = DBL_IDLE;
static uint32_t      _dblTimer = 0;


// Buffer reensamblador para saneamiento al vuelo de la pantalla (DASH -> ESC)
struct DashPacketFramer {
    uint8_t  buf[64];
    uint8_t  idx = 0;
    uint32_t lastByteTime = 0;

    void reset() { idx = 0; }
};
static DashPacketFramer _dashFramer;

// Buffer reensamblador para saneamiento de telemetría / batería (ESC -> DASH)
struct CtrlPacketFramer {
    uint8_t  buf[96];
    uint8_t  idx = 0;
    uint32_t lastByteTime = 0;

    void reset() { idx = 0; }
};
static CtrlPacketFramer _ctrlFramer;

// ---------------------------------------------------------------------------
// init() — Configura UARTs y pines de potencia
// ---------------------------------------------------------------------------
void init() {
    // 1. Pines de Hardware Power (P)
    // El cable verde P está conectado directamente entre pantalla y controladora por hardware.
    // D14 se mantiene en INPUT (Alta Impedancia / Hi-Z) para no interferir en ningún momento.
    pinMode(PIN_CTRL_PWR, INPUT);
    pinMode(PIN_DASH_PWR, INPUT_PULLUP);

    // 2. Iniciar UARTs monohilo en los pines designados
    sDash->begin(BAUD_RATE, SERIAL_8N1, PIN_DASH_DATA, PIN_DASH_DATA);
    sCtrl->begin(BAUD_RATE, SERIAL_8N1, PIN_CTRL_DATA, PIN_CTRL_DATA);

    Serial.println("[BRIDGE] ✓ Passthrough con Detección y Escalado de Modos de la Botonera activo.");
    Serial.printf("[BRIDGE]   Pantalla     → T=GPIO%d (UART 1-wire)\n", PIN_DASH_DATA);
    Serial.printf("[BRIDGE]   Controladora → T=GPIO%d (UART 1-wire)\n", PIN_CTRL_DATA);
}

// ---------------------------------------------------------------------------
// simulateDoubleClick() — Genera un doble clic por hardware en D14
// ---------------------------------------------------------------------------
void simulateDoubleClick() {
    if (_dblState == DBL_IDLE) {
        _dblState = DBL_PULSE1;
        _dblTimer = millis();
        pinMode(PIN_CTRL_PWR, OUTPUT);
        digitalWrite(PIN_CTRL_PWR, LOW);
        Serial.println("[BRIDGE] ► Doble clic simulado en D14 iniciado (Pulso 1 LOW)");
    }
}

// ---------------------------------------------------------------------------
// setControllerPowerPin() — Control manual del pin de potencia
// ---------------------------------------------------------------------------
void setControllerPowerPin(bool pullToGnd) {
    if (pullToGnd) {
        _manualPulseEnd = millis() + 600;
        pinMode(PIN_CTRL_PWR, OUTPUT);
        digitalWrite(PIN_CTRL_PWR, LOW);
    } else {
        _manualPulseEnd = 0;
        pinMode(PIN_CTRL_PWR, INPUT);
    }
}

// ---------------------------------------------------------------------------
// isDashboardButtonPressed()
// ---------------------------------------------------------------------------
bool isDashboardButtonPressed() {
    return (digitalRead(PIN_DASH_PWR) == LOW);
}

// ---------------------------------------------------------------------------
// update() — Passthrough directo con emulación al vuelo de gas y freno
// ---------------------------------------------------------------------------
void update() {
    uint32_t now = millis();
    uint32_t nowMicros = micros();

    // =========================================================================
    // 1. GESTIÓN DE LÍNEA P (D14): SIMULACIÓN DOBLE CLIC / PULSO MANUAL
    // =========================================================================
    bool dashBtnPressed = (digitalRead(PIN_DASH_PWR) == LOW);

    if (_dblState != DBL_IDLE) {
        uint32_t elapsed = now - _dblTimer;
        switch (_dblState) {
            case DBL_PULSE1:
                if (elapsed >= 90) {
                    pinMode(PIN_CTRL_PWR, INPUT); // Hi-Z pausa
                    _dblState = DBL_GAP;
                    _dblTimer = now;
                }
                break;
            case DBL_GAP:
                if (elapsed >= 90) {
                    pinMode(PIN_CTRL_PWR, OUTPUT);
                    digitalWrite(PIN_CTRL_PWR, LOW);  // Pulso 2 LOW
                    _dblState = DBL_PULSE2;
                    _dblTimer = now;
                }
                break;
            case DBL_PULSE2:
                if (elapsed >= 90) {
                    pinMode(PIN_CTRL_PWR, INPUT);     // Hi-Z en reposo
                    _dblState = DBL_IDLE;
                    Serial.println("[BRIDGE] ✓ Doble clic simulado en D14 completado.");
                }
                break;
            default:
                pinMode(PIN_CTRL_PWR, INPUT);
                _dblState = DBL_IDLE;
                break;
        }
    } else if (_manualPulseEnd > 0) {
        if (now >= _manualPulseEnd) {
            _manualPulseEnd = 0;
            pinMode(PIN_CTRL_PWR, INPUT); // Hi-Z tras completar pulso
        }
    }

    // Diagnóstico y medición de clics físicos en pantalla original (D32)
    static bool     _lastBtnState = false;
    static uint32_t _pressStartMs = 0;
    static uint32_t _lastClickEnd = 0;
    static uint8_t  _clickStreak  = 0;

    if (dashBtnPressed != _lastBtnState) {
        _lastBtnState = dashBtnPressed;
        if (!dashBtnPressed) {
            uint32_t dur = now - _pressStartMs;
            if (dur >= 25 && dur <= 600) {
                if (now - _lastClickEnd < 500) {
                    _clickStreak++;
                } else {
                    _clickStreak = 1;
                }
                _lastClickEnd = now;

                if (_clickStreak == 2) {
                    _clickStreak = 0;
                    Serial.println("[EVENTO] Botón pantalla: Doble clic físico detectado");
                }
            }
        } else {
            _pressStartMs = now;
        }
    }

    // =========================================================================
    // 2. RESINCRONIZACIÓN DE ECO (limpieza tras 12ms de silencio)
    // =========================================================================
    if (_ctrlEchoExpected > 0 && (nowMicros - _lastTxCtrlTime > 12000)) {
        _ctrlEchoExpected = 0;
    }
    if (_dashEchoExpected > 0 && (nowMicros - _lastTxDashTime > 12000)) {
        _dashEchoExpected = 0;
    }

    // =========================================================================
    // 3. PANTALLA (D13) → CONTROLADORA (D27) CON EMULACIÓN DE GAS/FRENO (ANTI-14)
    // =========================================================================
    // Flush por timeout si hay bytes pendientes de una trama incompleta (>12ms)
    if (_dashFramer.idx > 0 && (now - _dashFramer.lastByteTime > 12)) {
        for (uint8_t i = 0; i < _dashFramer.idx; i++) {
            sCtrl->write(_dashFramer.buf[i]);
            _ctrlEchoExpected++;
        }
        _lastTxCtrlTime = micros();
        _dashFramer.reset();
    }

    while (sDash->available() > 0) {
        uint8_t b = sDash->read();
        if (_dashEchoExpected > 0) {
            _dashEchoExpected--;
            continue;
        }

        _bytesFromDash++;
        _lastDashActivity = now;

        // Si no hemos detectado inicio de paquete 0x55 0xAA:
        if (_dashFramer.idx == 0) {
            if (b == 0x55) {
                _dashFramer.buf[_dashFramer.idx++] = b;
                _dashFramer.lastByteTime = now;
            } else {
                sCtrl->write(b);
                _ctrlEchoExpected++;
                _lastTxCtrlTime = micros();
            }
            continue;
        }

        if (_dashFramer.idx == 1) {
            if (b == 0xAA) {
                _dashFramer.buf[_dashFramer.idx++] = b;
                _dashFramer.lastByteTime = now;
            } else {
                sCtrl->write(_dashFramer.buf[0]);
                _ctrlEchoExpected++;
                sCtrl->write(b);
                _ctrlEchoExpected++;
                _lastTxCtrlTime = micros();
                _dashFramer.reset();
            }
            continue;
        }

        // Almacenar byte en buffer de trama
        _dashFramer.buf[_dashFramer.idx++] = b;
        _dashFramer.lastByteTime = now;

        uint8_t expectedTotal = _dashFramer.buf[2] + 6; // Total M365 = L + 6
        if (_dashFramer.idx == expectedTotal) {
            uint8_t* p = _dashFramer.buf;
            uint8_t totalLen = _dashFramer.idx;
            uint8_t dest = p[3];
            uint8_t cmd  = p[4];

            // Validar Checksum original
            uint16_t calcCk = M365Protocol::calcChecksum(&p[2], p[2] + 2);
            uint16_t pktCk  = (uint16_t)p[totalLen - 2] | ((uint16_t)p[totalLen - 1] << 8);

            if (calcCk == pktCk && dest == 0x20) {
                bool modified = false;

                // Emulación y escalado de acelerador según Modo de Conducción activo:
                if ((cmd == 0x65 || cmd == 0x64) && totalLen >= 11) {
                    DriveMode mode = UiManager::getCurrentMode();
                    constexpr uint16_t THROTTLE_SPAN = M365Protocol::M365_THROTTLE_MAX - M365Protocol::M365_THROTTLE_REST; // 172

                    // 1. Saneamiento de freno primero y detección de frenada activa
                    if (p[8] < M365Protocol::M365_BRAKE_REST) {
                        p[8] = M365Protocol::getBrake(); // Reposo seguro (0x26 = 38)
                        modified = true;
                    }
                    _lastBrakeActive = (p[8] > 50);

                    // 2. Determinar demanda de acelerador (maneta o botón pito)
                    uint8_t rawDemand = p[7];
                    uint8_t manualGas = M365Protocol::getThrottle();
                    if (manualGas > M365Protocol::M365_THROTTLE_REST) {
                        rawDemand = manualGas;
                    }

                    // 3. Calcular objetivo escalado por modo (con prioridad absoluta de corte por freno)
                    uint8_t targetScaledGas = M365Protocol::M365_THROTTLE_REST;
                    if (!_lastBrakeActive && rawDemand > M365Protocol::M365_THROTTLE_REST) {
                        if (M365Protocol::isDirectThrottleActive()) {
                            targetScaledGas = rawDemand;
                        } else {
                            uint16_t delta = rawDemand - M365Protocol::M365_THROTTLE_REST;
                            if (mode == MODE_ECO) {
                                targetScaledGas = M365Protocol::M365_THROTTLE_REST + (uint8_t)(delta * 55 / 100);
                            } else if (mode == MODE_DRIVE) {
                                targetScaledGas = M365Protocol::M365_THROTTLE_REST + (uint8_t)(delta * 78 / 100);
                            } else {
                                targetScaledGas = rawDemand;
                            }
                        }
                    }

                    // 4. Rampa suave (Slew-Rate Limiter de 2 etapas en subida, corte instantáneo en bajada)
                    if (targetScaledGas <= M365Protocol::M365_THROTTLE_REST) {
                        // Seguridad: al soltar gas o frenar, caída instantánea en 0 ms
                        _smoothedGas = M365Protocol::M365_THROTTLE_REST;
                    } else if (_smoothedGas < targetScaledGas) {
                        // Etapa 1 (38..58 raw): +1 por trama (~20ms) para enganche ultra-suave sin tirón
                        // Etapa 2 (59..210 raw): +2 por trama para aceleración progresiva
                        uint8_t step = (_smoothedGas < 58) ? 1 : 2;
                        uint16_t nextGas = (uint16_t)_smoothedGas + step;
                        _smoothedGas = (nextGas > targetScaledGas) ? targetScaledGas : (uint8_t)nextGas;
                    } else if (_smoothedGas > targetScaledGas) {
                        // Si se reduce el objetivo en caliente (ej. bajar de SPORT a ECO), ajuste suave de -3 por trama
                        uint8_t step = 3;
                        _smoothedGas = (_smoothedGas - targetScaledGas > step) ? (_smoothedGas - step) : targetScaledGas;
                    }

                    if (p[7] != _smoothedGas) {
                        p[7] = _smoothedGas;
                        modified = true;
                    }

                    _lastThrottlePct = (p[7] > M365Protocol::M365_THROTTLE_REST) ?
                                       (uint8_t)(((uint16_t)(p[7] - M365Protocol::M365_THROTTLE_REST) * 100) / THROTTLE_SPAN) : 0;
                }

                if (modified) {
                    _pktsSanitized++;
                    uint16_t newCk = M365Protocol::calcChecksum(&p[2], p[2] + 2);
                    p[totalLen - 2] = (uint8_t)(newCk & 0xFF);
                    p[totalLen - 1] = (uint8_t)((newCk >> 8) & 0xFF);
                }
            }

            // Enviar la trama (saneada o intacta) a la controladora
            for (uint8_t i = 0; i < totalLen; i++) {
                sCtrl->write(p[i]);
                _ctrlEchoExpected++;
            }
            _lastTxCtrlTime = micros();
            _dashFramer.reset();
        } else if (_dashFramer.idx >= sizeof(_dashFramer.buf)) {
            // Buffer lleno por anomalía de longitud: enviar directo
            for (uint8_t i = 0; i < _dashFramer.idx; i++) {
                sCtrl->write(_dashFramer.buf[i]);
                _ctrlEchoExpected++;
            }
            _lastTxCtrlTime = micros();
            _dashFramer.reset();
        }
    }

    // =========================================================================
    // 4. CONTROLADORA (D27) → PANTALLA (D13) CON SPOOFING DE BATERÍA (4 LEDS)
    // =========================================================================
    // Flush por timeout si hay bytes pendientes de una trama incompleta (>12ms)
    if (_ctrlFramer.idx > 0 && (now - _ctrlFramer.lastByteTime > 12)) {
        for (uint8_t i = 0; i < _ctrlFramer.idx; i++) {
            sDash->write(_ctrlFramer.buf[i]);
            _dashEchoExpected++;
        }
        _lastTxDashTime = micros();
        _ctrlFramer.reset();
    }

    while (sCtrl->available() > 0) {
        uint8_t b = sCtrl->read();
        if (_ctrlEchoExpected > 0) {
            _ctrlEchoExpected--;
            continue;
        }

        _bytesFromCtrl++;
        _lastCtrlActivity = now;

        // Si no hemos detectado inicio de paquete 0x55 0xAA:
        if (_ctrlFramer.idx == 0) {
            if (b == 0x55) {
                _ctrlFramer.buf[_ctrlFramer.idx++] = b;
                _ctrlFramer.lastByteTime = now;
            } else {
                sDash->write(b);
                _dashEchoExpected++;
                _lastTxDashTime = micros();
            }
            continue;
        }

        if (_ctrlFramer.idx == 1) {
            if (b == 0xAA) {
                _ctrlFramer.buf[_ctrlFramer.idx++] = b;
                _ctrlFramer.lastByteTime = now;
            } else {
                sDash->write(_ctrlFramer.buf[0]);
                _dashEchoExpected++;
                sDash->write(b);
                _dashEchoExpected++;
                _lastTxDashTime = micros();
                _ctrlFramer.reset();
            }
            continue;
        }

        // Almacenar byte en buffer de trama
        _ctrlFramer.buf[_ctrlFramer.idx++] = b;
        _ctrlFramer.lastByteTime = now;

        uint8_t expectedTotal = _ctrlFramer.buf[2] + 6; // Total M365 = L + 6
        if (_ctrlFramer.idx == expectedTotal) {
            uint8_t* p = _ctrlFramer.buf;
            uint8_t totalLen = _ctrlFramer.idx;
            uint8_t reg = p[5];

            // Validar Checksum original
            uint16_t calcCk = M365Protocol::calcChecksum(&p[2], p[2] + 2);
            uint16_t pktCk  = (uint16_t)p[totalLen - 2] | ((uint16_t)p[totalLen - 1] << 8);

            if (calcCk == pktCk) {
                bool modified = false;



                // Trama 0xB0: Telemetría general ESC -> Dashboard
                // Var176 (err:2B, p[6..7]), Var177 (warn:2B, p[8..9]), Var178 (flags:2B, p[10..11]),
                // Var179 (mode:2B, p[12..13]), Var180 (batt%:2B, p[14..15])
                if (reg == 0xB0 && totalLen >= 16) {
                    if (p[14] < 80) { // Si viene a 0% o batería baja por firmware sin BMS
                        p[14] = 85;   // 85% = 4 LEDs blancos fijos encendidos
                        p[15] = 0;
                        modified = true;
                    }
                }
                // Trama 0xB4: Registro dedicado de porcentaje de batería (Var180)
                else if (reg == 0xB4 && totalLen >= 8) {
                    if (p[6] < 80) {
                        p[6] = 85;
                        p[7] = 0;
                        modified = true;
                    }
                }
                // Trama 0x31: BMS capacidad restante
                else if (reg == 0x31 && totalLen >= 8) {
                    if (p[6] < 80) {
                        p[6] = 85;
                        p[7] = 0;
                        modified = true;
                    }
                }

                if (modified) {
                    _pktsBattSpoofed++;
                    uint16_t newCk = M365Protocol::calcChecksum(&p[2], p[2] + 2);
                    p[totalLen - 2] = (uint8_t)(newCk & 0xFF);
                    p[totalLen - 1] = (uint8_t)((newCk >> 8) & 0xFF);
                }
            }

            // Enviar la trama (con batería saneada a 85% o intacta) a la pantalla
            for (uint8_t i = 0; i < totalLen; i++) {
                sDash->write(p[i]);
                _dashEchoExpected++;
            }
            _lastTxDashTime = micros();
            _ctrlFramer.reset();
        } else if (_ctrlFramer.idx >= sizeof(_ctrlFramer.buf)) {
            // Buffer lleno por anomalía de longitud: enviar directo
            for (uint8_t i = 0; i < _ctrlFramer.idx; i++) {
                sDash->write(_ctrlFramer.buf[i]);
                _dashEchoExpected++;
            }
            _lastTxDashTime = micros();
            _ctrlFramer.reset();
        }
    }
}

// ---------------------------------------------------------------------------
// sendToController() — Inyección directa (para comandos de la botonera)
// ---------------------------------------------------------------------------
void sendToController(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        sCtrl->write(buf[i]);
        _ctrlEchoExpected++;
    }
    _lastTxCtrlTime = micros();
}

// ---------------------------------------------------------------------------
// sendToDashboard() — Inyección directa
// ---------------------------------------------------------------------------
void sendToDashboard(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        sDash->write(buf[i]);
        _dashEchoExpected++;
    }
    _lastTxDashTime = micros();
}

// ---------------------------------------------------------------------------
// Estadísticas y estado del bus
// ---------------------------------------------------------------------------
uint32_t getBytesFromDash() { return _bytesFromDash; }
uint32_t getBytesFromCtrl() { return _bytesFromCtrl; }
bool isDashboardActive() {
    return (_lastDashActivity > 0 && (millis() - _lastDashActivity < 2000));
}
bool isControllerActive() {
    return (_lastCtrlActivity > 0 && (millis() - _lastCtrlActivity < 2000));
}
uint8_t getLastThrottlePercent() {
    if (millis() - _lastDashActivity > 1500) return 0;
    return _lastThrottlePct;
}
bool isBrakeActive() {
    if (millis() - _lastDashActivity > 1500) return false;
    return _lastBrakeActive;
}

} // namespace UartBridge
