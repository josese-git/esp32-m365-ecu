// =============================================================================
// m365_protocol.h — Protocolo UART Xiaomi M365
// =============================================================================
//
// Implementa la capa de protocolo para comunicarse con la controladora (ESC)
// del Xiaomi M365 desde el ESP32 actuando como master/dashboard.
//
// FORMATO DE TRAMA M365:
//   +------+------+-----+----+----+----+-----+------+------+
//   | 0x55 | 0xAA |  L  |  D |  T |  C | ... | CK0  | CK1  |
//   +------+------+-----+----+----+----+-----+------+------+
//
//   L   = longitud: número de bytes payload (...) + 2
//   D   = dirección: 0x20 master→ESC  | 0x23 ESC→master
//   T   = tipo:      0x01 lectura      | 0x03 escritura
//   C   = registro destino
//   ... = payload (little-endian, byte menos significativo primero)
//   CK  = checksum: suma de bytes desde L hasta fin de datos, XOR 0xFFFF
//         CK0 = byte bajo (LSB), CK1 = byte alto (MSB)
//
// MODOS DE VELOCIDAD (Registro 0x7B — uint16 LE, unidad: km/h × 1000):
//   MODE_ECO   → 1600  (0x0640) = 16.0 km/h
//   MODE_DRIVE → 2400  (0x0960) = 24.0 km/h  (límite legal EU)
//   MODE_SPORT → 3000  (0x0BB8) = 30.0 km/h
//
// ENCENDIDO / KEEPALIVE:
//   La controladora M365 se mantiene activa mientras recibe heartbeats del
//   master cada ~50 ms (registro 0x70, tipo lectura). Si la pantalla no está
//   conectada, el ESP32 puede simularlos activando el modo KeepAlive.
//   BTN_PITO → alterna KeepAlive ON/OFF.
// =============================================================================

#pragma once
#include <Arduino.h>
#include "config.h"

namespace M365Protocol {

    // -------------------------------------------------------------------------
    // API pública
    // -------------------------------------------------------------------------

    void init();    // Llamar una vez en setup()
    void update();  // Llamar en cada loop() — gestiona secuencia de encendido y heartbeat

    // Encendido y apagado completo (Hardware pulse en línea P + UART handshake)
    void powerOn();
    void powerOff();
    void triggerPowerPulse(uint32_t durationMs = 600);
    bool isPowerActive();

    // Inyecta el paquete de modo de conducción al ESC (Registro 0x7B)
    void sendSpeedMode(DriveMode mode);

    // Inyecta el comando de luces (Registro 0x7D)
    void sendLight(bool enable);

    // Constantes de acelerador y freno del protocolo M365:
    // Reposo / suelto (0%): 0x26 (38 dec). Menos de 0x26 provoca Error 14/15 en la controladora.
    // Máximo / a fondo (100%): 0xD2 (210 dec).
    constexpr uint8_t M365_THROTTLE_REST = 0x26;
    constexpr uint8_t M365_THROTTLE_MAX  = 0xD2;
    constexpr uint8_t M365_BRAKE_REST    = 0x26;
    constexpr uint8_t M365_BRAKE_MAX     = 0xD2;

    // Control y emulación de acelerador y freno (Trama 0x65 cada 25ms)
    void    setThrottle(uint8_t val);
    uint8_t getThrottle();
    void    setDirectThrottle(uint8_t val);
    bool    isDirectThrottleActive();
    void    setBrake(uint8_t val);
    uint8_t getBrake();
    void    sendThrottle(uint8_t throttle, uint8_t brake = M365_BRAKE_REST);

    // Activa/desactiva el heartbeat autónomo (simula la pantalla encendida).
    void setKeepAlive(bool enable);
    bool isKeepAliveActive();

    // -------------------------------------------------------------------------
    // Utilidades de protocolo
    // -------------------------------------------------------------------------

    // Calcula checksum M365: suma de 'len' bytes desde 'data', XOR 0xFFFF
    uint16_t calcChecksum(const uint8_t* data, uint8_t len);

    // Construye un paquete M365 completo en 'outBuf'.
    // Retorna el total de bytes escritos.
    uint8_t buildPacket(uint8_t*       outBuf,
                        uint8_t        dest,
                        uint8_t        type,
                        uint8_t        reg,
                        const uint8_t* payload,
                        uint8_t        payloadLen);

} // namespace M365Protocol
