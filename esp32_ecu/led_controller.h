// =============================================================================
// led_controller.h — Control de LEDs de Estado (No Bloqueante)
// =============================================================================
// Gestiona los 3 LEDs de modo con soporte para animaciones suaves.
// Expone init(), update() y setMode().
// =============================================================================

#pragma once
#include <Arduino.h>
#include "config.h"

namespace LedController {
    enum LedState : uint8_t {
        BOOT, // Animación Knight Rider al encender (no bloqueante)
        IDLE  // Solo el LED del modo actual encendido (fijo)
    };

    void init();
    void update();                  // Motor de animación (llamar cada loop)
    void setMode(DriveMode mode);   // Cambia el modo y actualiza los LEDs
    DriveMode getMode();
    void setAll(bool state);
    void blinkAll(uint8_t times, uint16_t periodMs);
}
