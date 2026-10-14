// =============================================================================
// ui_manager.cpp — Implementación de la FSM de UI y Botones
// =============================================================================
//
// LÓGICA DE BOTONES LATCHING (fijos):
//   Al estar hundidos, su estado GPIO queda en LOW de forma permanente.
//   Por eso NO basta con leer el nivel: hay que detectar el FLANCO
//   (transición HIGH→LOW) usando el estado anterior guardado en memoria.
//   La acción se dispara una única vez por pulsada.
//
// LÓGICA DE BOTÓN MOMENTÁNEO (PITO):
//   Vuelve a HIGH al soltarse, así que leer el nivel LOW es suficiente.
//
// DEBOUNCE:
//   Se usa un debounce por tiempo: si el pin cambia, se espera DEBOUNCE_MS
//   antes de confirmar el nuevo estado. Evita rebotes mecánicos.
// =============================================================================

#include "ui_manager.h"
#include "led_controller.h"
#include "m365_protocol.h"
#include "uart_bridge.h"
#include "config.h"
#include <Arduino.h>

namespace UiManager {

// ---------------------------------------------------------------------------
// Estructura de estado para cada botón con debounce
// ---------------------------------------------------------------------------
struct ButtonState {
    uint8_t  pin;
    bool     lastStableState;    // Último estado confirmado (tras debounce)
    bool     pendingState;       // Estado que está esperando confirmarse
    uint32_t lastChangeTime;     // millis() del último cambio detectado
    bool     triggered;          // Flag de flanco procesado (latching)
};

// ---------------------------------------------------------------------------
// Instancias de botones
// ---------------------------------------------------------------------------
static ButtonState _btnIzq     = { BTN_IZQ,      false, false, 0, false };
static ButtonState _btnDer     = { BTN_DER,      false, false, 0, false };
static ButtonState _btnLuz     = { BTN_LUZ,      false, false, 0, false };
static ButtonState _btnPito    = { BTN_PITO,     false, false, 0, false };
static ButtonState _btnDashPwr = { PIN_DASH_PWR, false, false, 0, false };

// ---------------------------------------------------------------------------
// Estado de la aplicación
// ---------------------------------------------------------------------------
static DriveMode _currentMode       = MODE_ECO;
static bool      _luzActive         = false;
static bool      _pitoActive        = false;
static uint8_t   _benchThrottlePct  = 50; // 50% por defecto para pruebas seguras

// ---------------------------------------------------------------------------
// _updateLatchingSwitch() — Para interruptores con enclavamiento (Latching)
// Retorna true en CADA cambio de posición (tanto al hundir como al desclavar).
// Esto hace que un solo click físico dispare la acción de inmediato.
// ---------------------------------------------------------------------------
static bool _updateLatchingSwitch(ButtonState& btn) {
    bool rawState = (digitalRead(btn.pin) == LOW);
    uint32_t now = millis();

    if (rawState != btn.pendingState) {
        btn.pendingState   = rawState;
        btn.lastChangeTime = now;
    }

    if ((now - btn.lastChangeTime) >= DEBOUNCE_MS) {
        if (btn.pendingState != btn.lastStableState) {
            btn.lastStableState = btn.pendingState;
            return true; // Cambio de posición confirmado con 1 solo click
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// _updateButton() — Actualiza el estado de un botón momentáneo con debounce
// Retorna true si se detectó un FLANCO DESCENDENTE (HIGH→LOW) confirmado.
// ---------------------------------------------------------------------------
static bool _updateButton(ButtonState& btn) {
    bool rawState = (digitalRead(btn.pin) == LOW);

    uint32_t now = millis();

    if (rawState != btn.pendingState) {
        btn.pendingState   = rawState;
        btn.lastChangeTime = now;
        btn.triggered      = false;
    }

    if ((now - btn.lastChangeTime) >= DEBOUNCE_MS) {
        if (btn.pendingState != btn.lastStableState) {
            btn.lastStableState = btn.pendingState;

            if (btn.lastStableState && !btn.triggered) {
                btn.triggered = true;
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// init() — Configura pines de botones
// ---------------------------------------------------------------------------
void init() {
    pinMode(BTN_IZQ,      INPUT_PULLUP);
    pinMode(BTN_DER,      INPUT_PULLUP);
    pinMode(BTN_LUZ,      INPUT_PULLUP);
    pinMode(BTN_PITO,     INPUT_PULLUP);
    pinMode(PIN_DASH_PWR, INPUT_PULLUP); // D32: Botón pantalla original

    // Lee estado inicial y sincroniza para no disparar acciones falsas al arrancar
    _btnIzq.lastStableState     = (digitalRead(BTN_IZQ)      == LOW);
    _btnDer.lastStableState     = (digitalRead(BTN_DER)      == LOW);
    _btnLuz.lastStableState     = (digitalRead(BTN_LUZ)      == LOW);
    _btnPito.lastStableState    = (digitalRead(BTN_PITO)     == LOW);
    _btnDashPwr.lastStableState = (digitalRead(PIN_DASH_PWR) == LOW);

    _btnIzq.pendingState        = _btnIzq.lastStableState;
    _btnDer.pendingState        = _btnDer.lastStableState;
    _btnLuz.pendingState        = _btnLuz.lastStableState;
    _btnPito.pendingState       = _btnPito.lastStableState;
    _btnDashPwr.pendingState    = _btnDashPwr.lastStableState;

    Serial.println("[UI] ✓ Botonera Latching configurada (1 click por cambio de posición).");
    Serial.printf("[UI]   Modo inicial: ECO | Luz: OFF | Pito: OFF\n");
}

// Parámetros de temporización para UI
constexpr uint32_t MODE_CHANGE_COOLDOWN_MS = 400; // Cooldown mínimo entre cambios de modo
constexpr uint32_t LUZ_CHANGE_COOLDOWN_MS  = 400; // Cooldown mínimo para conmutar luz

static uint32_t _lastModeChangeTime = 0;
static uint32_t _lastLuzChangeTime  = 0;

static void _applyBenchThrottle() {
    uint8_t pct = _benchThrottlePct;
    constexpr uint16_t THROTTLE_SPAN = M365Protocol::M365_THROTTLE_MAX - M365Protocol::M365_THROTTLE_REST; // 172
    uint8_t rawByte = M365Protocol::M365_THROTTLE_REST + (uint8_t)((uint16_t)pct * THROTTLE_SPAN / 100);
    M365Protocol::setThrottle(rawByte);
}

// ---------------------------------------------------------------------------
// update() — FSM principal de UI (llamar en cada loop)
// ---------------------------------------------------------------------------
void update() {
    uint32_t now = millis();

    // --- BTN_DER: Sube modo de conducción (1 ECO -> 2 DRIVE -> 3 SPORT) a la de 1 click ---
    if (_updateLatchingSwitch(_btnDer)) {
        if (now - _lastModeChangeTime >= MODE_CHANGE_COOLDOWN_MS) {
            _lastModeChangeTime = now;
            if (_currentMode < MODE_SPORT) {
                _currentMode = (DriveMode)((int)_currentMode + 1);
                LedController::setMode(_currentMode);
                const char* names[] = { "1 [ECO - 55%]", "2 [DRIVE - 78%]", "3 [SPORT - 100%]" };
                Serial.printf("[EVENTO] ★ Modo cambiado a: %s\n", names[_currentMode]);
            }
        }
    }

    // --- BTN_IZQ: Baja modo de conducción (3 SPORT -> 2 DRIVE -> 1 ECO) a la de 1 click ---
    if (_updateLatchingSwitch(_btnIzq)) {
        if (now - _lastModeChangeTime >= MODE_CHANGE_COOLDOWN_MS) {
            _lastModeChangeTime = now;
            if (_currentMode > MODE_ECO) {
                _currentMode = (DriveMode)((int)_currentMode - 1);
                LedController::setMode(_currentMode);
                const char* names[] = { "1 [ECO - 55%]", "2 [DRIVE - 78%]", "3 [SPORT - 100%]" };
                Serial.printf("[EVENTO] ★ Modo cambiado a: %s\n", names[_currentMode]);
            }
        }
    }

    // --- BTN_LUZ: Toggle faro M365 a la de 1 click ---
    if (_updateLatchingSwitch(_btnLuz)) {
        if (now - _lastLuzChangeTime >= LUZ_CHANGE_COOLDOWN_MS) {
            _lastLuzChangeTime = now;
            _luzActive = !_luzActive;
            M365Protocol::sendLight(_luzActive);
            Serial.printf("[EVENTO] 💡 Luces: %s\n", _luzActive ? "ENCENDIDAS" : "APAGADAS");
        }
    }

    // --- BTN_PITO: Momentáneo — ACELERADOR DE PRUEBA (Porcentaje configurable por monitor serie) ---
    _updateButton(_btnPito);
    bool pitoPressed = _btnPito.lastStableState;

    if (pitoPressed != _pitoActive) {
        _pitoActive = pitoPressed;
        if (_pitoActive) {
            _applyBenchThrottle();
            uint8_t modeScale = (_currentMode == MODE_ECO) ? 55 : (_currentMode == MODE_DRIVE) ? 78 : 100;
            uint8_t effectivePct = (uint8_t)((uint16_t)_benchThrottlePct * modeScale / 100);
            Serial.printf("[EVENTO] ⚡ Botón Pito PULSADO → Set: %u%% | Modo aplica %u%% (Efectivo: ~%u%% con rampa suave)\n",
                          _benchThrottlePct, modeScale, effectivePct);
        } else {
            M365Protocol::setThrottle(M365Protocol::M365_THROTTLE_REST);
            Serial.println("[EVENTO] 🛑 Botón Pito SOLTADO → Corte instantáneo a reposo (0% / 0x26)");
        }
    }

    // --- BOTÓN PANTALLA ORIGINAL (D32) ---
    _updateButton(_btnDashPwr);
}

// ---------------------------------------------------------------------------
// setModeFromEsc() — Solo para diagnóstico (no sobreescribe los modos de la botonera)
// ---------------------------------------------------------------------------
void setModeFromEsc(DriveMode mode) {
    // Los 3 modos virtuales son controlados de forma soberana por la botonera
}

// ---------------------------------------------------------------------------
// Getters y Setters de estado
// ---------------------------------------------------------------------------
void setBenchThrottlePercent(uint8_t pct) {
    if (pct > 100) pct = 100;
    _benchThrottlePct = pct;
    if (_pitoActive) {
        _applyBenchThrottle();
    }
}

void setLuzActive(bool active) {
    _luzActive = active;
}

uint8_t   getBenchThrottlePercent() { return _benchThrottlePct; }
DriveMode getCurrentMode()          { return _currentMode; }
bool      isLuzActive()             { return _luzActive;   }
bool      isPitoActive()            { return _pitoActive;  }

} // namespace UiManager
