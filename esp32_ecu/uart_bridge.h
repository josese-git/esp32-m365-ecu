// =============================================================================
// uart_bridge.h — Puente UART Bidireccional (Passthrough M365)
// =============================================================================
// Módulo responsable de:
//   1. Passthrough transparente: Pantalla <-> Controladora sin latencia.
//   2. (Fase 4) Intercepción de tramas 55 AA para inyección de throttle.
// =============================================================================

#pragma once
#include <Arduino.h>

namespace UartBridge {
    void init();    // Configura Serial1 (D13), Serial2 (D27) y pines de Power (D12, D14)
    void update();  // Passthrough UART bidireccional y puenteo de botón físico — llamar cada loop()

    // Inyección directa de tramas al ESC (Serial2 / D27)
    void sendToController(const uint8_t* buf, size_t len);

    // Inyección directa de tramas a la pantalla (Serial1 / D13)
    void sendToDashboard(const uint8_t* buf, size_t len);

    // Control hardware de la línea "P" de la controladora (D14)
    // pullToGnd = true  → D14 a LOW (GND) (simula pulsación de botón de encendido)
    // pullToGnd = false → D14 a HIGH / Hi-Z (botón liberado)
    void setControllerPowerPin(bool pullToGnd);

    // Simula un doble clic por hardware en el pin P del ESC (para cambio de modo físico)
    void simulateDoubleClick();

    // Lectura hardware de la línea "P" de la pantalla (D32)
    // Retorna true si el usuario tiene presionado el botón físico de la pantalla original
    bool isDashboardButtonPressed();

    // Estadísticas y estado del bus
    uint32_t getBytesFromDash();
    uint32_t getBytesFromCtrl();
    bool     isDashboardActive();
    bool     isControllerActive();
    uint8_t  getLastThrottlePercent();
    bool     isBrakeActive();
}
