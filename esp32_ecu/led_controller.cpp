// =============================================================================
// led_controller.cpp — Implementación del Control de LEDs de la Botonera
// =============================================================================
// 100% No bloqueante (millis(), cero delay).
// Soporta:
//   1. BOOT: Animación Knight Rider suave (3 barridos de ida y vuelta a 80ms/paso).
//   2. IDLE: Ilumina de forma fija el LED del modo de conducción actual (1 ECO, 2 DRIVE, 3 SPORT).
// =============================================================================

#include "led_controller.h"
#include "config.h"
#include <Arduino.h>

namespace LedController {

// Pines de los LEDs indexados por DriveMode
static const uint8_t LED_PINS[MODE_COUNT] = { LED_ECO, LED_DRIVE, LED_SPORT };

// Parámetros de temporización
constexpr unsigned long BOOT_STEP_MS  = 80;   // Velocidad por paso Knight Rider
constexpr uint8_t       BOOT_REPEATS  = 3;    // Repeticiones Knight Rider

// Variables de estado
static LedState  _ledState    = BOOT;
static DriveMode _currentMode = MODE_ECO;

// Variables de animación Knight Rider
static uint8_t       _bootStep   = 0;
static uint8_t       _bootCount  = 0;
static unsigned long _bootStepMs = 0;

// ---------------------------------------------------------------------------
// Helpers para escritura directa de LEDs
// ---------------------------------------------------------------------------
static void setLeds(bool l1, bool l2, bool l3) {
    digitalWrite(LED_ECO,   l1 ? HIGH : LOW);
    digitalWrite(LED_DRIVE, l2 ? HIGH : LOW);
    digitalWrite(LED_SPORT, l3 ? HIGH : LOW);
}

void setAll(bool state) {
    setLeds(state, state, state);
}

// Aplica el patrón correspondiente al modo activo en estado IDLE
static void applyIdleLeds() {
    setLeds(_currentMode == MODE_ECO,
            _currentMode == MODE_DRIVE,
            _currentMode == MODE_SPORT);
}

// ---------------------------------------------------------------------------
// init() — Configura pines e inicia la animación Knight Rider no bloqueante
// ---------------------------------------------------------------------------
void init() {
    for (uint8_t i = 0; i < MODE_COUNT; i++) {
        pinMode(LED_PINS[i], OUTPUT);
        digitalWrite(LED_PINS[i], LOW);
    }

    _ledState    = BOOT;
    _bootStep    = 0;
    _bootCount   = 0;
    _bootStepMs  = millis();
    _currentMode = MODE_ECO;

    Serial.println("[LED] ✓ Botonera LED inicializada. Iniciando Knight Rider no bloqueante...");
}

// ---------------------------------------------------------------------------
// updateBoot() — Animación Knight Rider: L1 -> L2 -> L3 -> L2 (3 ciclos)
// ---------------------------------------------------------------------------
static void updateBoot() {
    unsigned long now = millis();
    if ((now - _bootStepMs) < BOOT_STEP_MS) return;
    _bootStepMs = now;

    static const bool seq[4][3] = {
        { true,  false, false },  // L1 (ECO)
        { false, true,  false },  // L2 (DRIVE)
        { false, false, true  },  // L3 (SPORT)
        { false, true,  false },  // L2 (Rebote)
    };

    setLeds(seq[_bootStep][0], seq[_bootStep][1], seq[_bootStep][2]);
    _bootStep++;

    if (_bootStep >= 4) {
        _bootStep = 0;
        _bootCount++;
        if (_bootCount >= BOOT_REPEATS) {
            _ledState = IDLE;
            applyIdleLeds();
        }
    }
}

// ---------------------------------------------------------------------------
// update() — Motor principal de animación (llamar en cada loop)
// ---------------------------------------------------------------------------
void update() {
    switch (_ledState) {
        case BOOT:
            updateBoot();
            break;
        case IDLE:
            applyIdleLeds();
            break;
    }
}

// ---------------------------------------------------------------------------
// setMode() — Cambia el modo activo de la botonera
// ---------------------------------------------------------------------------
void setMode(DriveMode mode) {
    _currentMode = mode;
    if (_ledState == IDLE) {
        applyIdleLeds();
    }
}

DriveMode getMode() {
    return _currentMode;
}

void blinkAll(uint8_t times, uint16_t periodMs) {
    // Para compatibilidad
    applyIdleLeds();
}

} // namespace LedController
