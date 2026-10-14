// =============================================================================
// ui_manager.h — Máquina de Estados de la Interfaz de Usuario
// =============================================================================
// Gestiona:
//   - Lectura de botones con debounce robusto
//   - Detección de flancos para botones latching (IZQ, DER, LUZ)
//   - Lectura de nivel para botón momentáneo (PITO)
//   - FSM de modos de conducción
// =============================================================================

#pragma once
#include <Arduino.h>
#include "config.h"

namespace UiManager {
    void init();
    void update();                      // Debe llamarse cada loop()
    DriveMode getCurrentMode();
    void      setModeFromEsc(DriveMode mode); // Sincroniza el modo real recibido del ESC
    bool      isLuzActive();            // Estado del toggle de luces
    void      setLuzActive(bool active);// Sincroniza estado de luces desde comando serie
    bool      isPitoActive();           // Estado actual del claxon

    // Configuración dinámica del porcentaje del acelerador de prueba (botón pito)
    void    setBenchThrottlePercent(uint8_t pct);
    uint8_t getBenchThrottlePercent();
}
