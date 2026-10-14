// =============================================================================
// config.h — Configuración Central del Proyecto ESP32 ECU E-Bike
// =============================================================================
// Todos los pines, baudrates y parámetros ajustables en un único lugar.
// Modifica SOLO este archivo para adaptar el firmware a cambios de hardware.
// =============================================================================

#pragma once

// -----------------------------------------------------------------------------
// PANTALLA Y CONTROLADORA XIAOMI M365 (Líneas "T" UART + Líneas "P" Power)
// -----------------------------------------------------------------------------
#define BAUD_RATE           115200

// Pantalla Xiaomi M365 (Dashboard)
#define UART_DASH_PORT      1           // Serial1
#define PIN_DASH_DATA       13          // D13: "T" pantalla (UART Half-Duplex bidireccional)
#define PIN_DASH_PWR        32          // D32: "P" pantalla (Botón físico original, INPUT_PULLUP — D12 causaba fallo de arranque en Flash VDD_SDIO)

// Controladora de Motor Xiaomi M365 (ESC)
#define UART_CTRL_PORT      2           // Serial2
#define PIN_CTRL_DATA       27          // D27: "T" controladora (UART Half-Duplex bidireccional)
#define PIN_CTRL_PWR        14          // D14: "P" controladora (Línea encendido hardware a GND, OUTPUT)

// Tamaño del buffer de lectura UART (bytes por ciclo de loop)
#define UART_BUFFER_SIZE    64

// -----------------------------------------------------------------------------
// BOTONES — INPUT_PULLUP, activos en LOW
// -----------------------------------------------------------------------------

// Botones fijos (latching) — se detectan por FLANCO (cambio de estado)
#define BTN_IZQ             18          // Bajar modo
#define BTN_DER             19          // Subir modo
#define BTN_LUZ             21          // Toggle luces

// Botón momentáneo — se detecta por NIVEL
#define BTN_PITO            22          // Claxon

// Tiempo de debounce en milisegundos
#define DEBOUNCE_MS         50

// -----------------------------------------------------------------------------
// LEDs DE ESTADO — OUTPUT, activos en HIGH
// -----------------------------------------------------------------------------
#define LED_ECO             23          // Verde  — Modo Eco
#define LED_DRIVE           25          // Verde  — Modo Drive
#define LED_SPORT           26          // Azul   — Modo Sport

// -----------------------------------------------------------------------------
// EMULADOR BMS XIAOMI M365 (Puerto USART2 / conector 3 pines)
// -----------------------------------------------------------------------------
#define PIN_BMS_RX          34          // D34: Entrada desde cable AMARILLO de controladora (ESC TX)
#define PIN_BMS_TX          33          // D33: Salida hacia cable BLANCO de controladora (ESC RX)

// -----------------------------------------------------------------------------
// SENSORES — FASE FUTURA (pines TBD)
// -----------------------------------------------------------------------------
#define PIN_PAS             4           // Sensor PAS cadencia (FASE FUTURA)

// -----------------------------------------------------------------------------
// OTA / WiFi
// -----------------------------------------------------------------------------
#define OTA_HOSTNAME        "esp32-ebike-ecu"
#define WIFI_SSID           "esp32"
#define WIFI_PASSWORD       "12345678"
#define WIFI_TIMEOUT_MS     10000       // Timeout para conectar al arranque

// -----------------------------------------------------------------------------
// PROTOCOLO M365 — Timings y constantes de inyección UART
// -----------------------------------------------------------------------------
// Intervalo entre heartbeats al ESC cuando KeepAlive está activo (ms).
// La controladora tolera hasta ~300 ms sin paquetes antes de apagarse.
#define M365_HEARTBEAT_MS       50

// Retardo entre el cambio de modo en la UI y el envío del paquete de velocidad.
// Evita colisiones con el passthrough normal (ms).
#define M365_MODE_SEND_DELAY_MS 20

// -----------------------------------------------------------------------------
// MODOS DE CONDUCCIÓN
// -----------------------------------------------------------------------------
enum DriveMode {
    MODE_ECO   = 0,
    MODE_DRIVE = 1,
    MODE_SPORT = 2,
    MODE_COUNT = 3   // Total de modos (útil para wrapping)
};

// -----------------------------------------------------------------------------
// VERSIÓN DE FIRMWARE
// -----------------------------------------------------------------------------
#define FW_VERSION          "0.1.0-alpha"
#define PROJECT_NAME        "ESP32 E-Bike ECU"

// -----------------------------------------------------------------------------
// PUERTO SERIE DEBUG (PC USB)
// -----------------------------------------------------------------------------
// Serial nativo por Hardware UART0 (GPIO 1 TX / GPIO 3 RX a 115200 baud).

