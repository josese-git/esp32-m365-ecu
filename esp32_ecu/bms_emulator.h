// =============================================================================
// bms_emulator.h — Emulador BMS Xiaomi M365 (Basado en CamiAlfa M365_BMS_EMULATOR)
// =============================================================================
// Emula el protocolo de batería oficial de Xiaomi M365 para la controladora (ESC)
// en su puerto serie secundario (USART2 / conector blanco de 3 pines).
//
// CONEXIÓN FÍSICA:
//   - Cable BLANCO (ESC TX / Salida peticiones)  → ESP32 RX (GPIO 34 / D34)
//   - Cable AMARILLO (ESC RX / Entrada datos)    → ESP32 TX (GPIO 33 / D33)
//   - GND común entre ESP32 y Controladora
// =============================================================================

#pragma once
#include <Arduino.h>

namespace BmsEmulator {
    void init();
    void update();

    // Configuración dinámica de valores emulados
    void setBatteryVoltage(float volts);       // Ej: 36.1V
    void setBatteryPercentage(uint8_t percent); // Ej: 85%
    void setCapacity(uint16_t mah);            // Ej: 7800 mAh

    // Estadísticas
    uint32_t getQueriesReceived();
    uint32_t getRepliesSent();
    bool     isActive();
}
