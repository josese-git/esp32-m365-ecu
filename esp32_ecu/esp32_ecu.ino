// =============================================================================
// esp32_ecu.ino — Punto de Entrada Principal del Firmware ECU E-Bike
// =============================================================================
//  Proyecto : ESP32 E-Bike ECU (MiTM Xiaomi M365)
//  Versión  : 0.1.0-alpha
//  Autor    : (tu nombre aquí)
// =============================================================================

#include "config.h"
#include "ota_manager.h"
#include "uart_bridge.h"
#include "led_controller.h"
#include "ui_manager.h"
#include "m365_protocol.h"
#include "bms_emulator.h"

// =============================================================================

// Procesa comandos recibidos por el monitor serie
void processSerialCommand(const char* rawCmd) {
    // Limpiar caracteres de control / basura iniciales
    while (*rawCmd && ((unsigned char)*rawCmd <= ' ' || (unsigned char)*rawCmd >= 127)) rawCmd++;
    if (*rawCmd == '\0') return;

    // Comandos alfabéticos rápidos
    char c = *rawCmd;
    if ((c == 'p' || c == 'P') && !isdigit((unsigned char)rawCmd[1])) {
        Serial.println("[CMD] ► Pulso de encendido disparado por comando serie (600ms en D14 a GND)");
        LedController::blinkAll(2, 100);
        M365Protocol::triggerPowerPulse(600);
        return;
    } else if (c == 'm' || c == 'M') {
        Serial.println("[CMD] ► Doble clic simulado disparado por consola serie...");
        UartBridge::simulateDoubleClick();
        return;
    } else if (c == 'b' || c == 'B') {
        Serial.printf("[BMS_STATUS] Consultas ESC: %u | Respuestas: %u | Activo: %s\n",
                      BmsEmulator::getQueriesReceived(),
                      BmsEmulator::getRepliesSent(),
                      BmsEmulator::isActive() ? "SÍ" : "NO");
        return;
    } else if (c == 'l' || c == 'L') {
        bool newLight = !UiManager::isLuzActive();
        UiManager::setLuzActive(newLight);
        M365Protocol::sendLight(newLight);
        Serial.printf("[CMD] 💡 Luces conmutadas a: %s\n", newLight ? "ON" : "OFF");
        return;
    } else if (c == 'w' || c == 'W') {
        OtaManager::startWiFi();
        return;
    } else if (c == 'h' || c == 'H' || c == '?') {
        Serial.println("\n[AYUDA] Comandos disponibles por Serial:");
        Serial.printf("  0 .. 100 -> Configura el %% de gas del botón pito (actual: %u%%)\n", UiManager::getBenchThrottlePercent());
        Serial.println("  b        -> Estado detallado BMS CamiAlfa");
        Serial.println("  l        -> Conmutar luces (ON / OFF)");
        Serial.println("  w        -> Activar WiFi para OTA");
        Serial.println("  p        -> Pulso encendido D14 (600ms)");
        Serial.println("  m        -> Doble clic simulado D14");
        return;
    }

    // Si contiene algún número (ej: "75", "75%", "gas 80", "set 60")
    const char* p = rawCmd;
    while (*p && !isdigit((unsigned char)*p)) p++;
    if (*p) {
        int pct = atoi(p);
        if (pct < 0)   pct = 0;
        if (pct > 100) pct = 100;

        UiManager::setBenchThrottlePercent((uint8_t)pct);
        Serial.printf("[CMD] ✓ Gas del botón pito calibrado al %d%%. ¡Púlsalo para acelerar!\n", pct);
        return;
    }

    if (isprint((unsigned char)*rawCmd)) {
        Serial.printf("[CMD] ❓ Comando desconocido: '%s' (Introduce un número 0-100 para calibrar gas)\n", rawCmd);
    }
}

// =============================================================================
// setup() — Inicialización única al arranque
// =============================================================================
void setup() {
    pinMode(PIN_CTRL_PWR, INPUT);
    pinMode(PIN_DASH_PWR, INPUT_PULLUP);

    Serial.begin(115200);
    delay(200);

    Serial.println("\n============================================================");
    Serial.printf("  %s v%s — ECU E-Bike Xiaomi M365\n", PROJECT_NAME, FW_VERSION);
    Serial.println("============================================================");
    Serial.println("  [BMS Emulador] : UART Hardware (D34 RX / D33 TX)");
    Serial.println("  [Puente UART ] : Pantalla D13 <-> Controladora D27");
    Serial.println("  [Botonera    ] : 3 Modos (ECO/DRV/SPT) + Luces");
    Serial.printf("  [Acelerador  ] : Botón Pito activo (Escribe 1-100 para cambiar %%, actual: %u%%)\n", UiManager::getBenchThrottlePercent());
    Serial.println("============================================================");

    LedController::init();
    UiManager::init();
    UartBridge::init();
    M365Protocol::init();
    BmsEmulator::init();
    OtaManager::init();

    Serial.println("[SISTEMA] ✓ Inicialización completada. Iniciando dashboard...\n");
}

// =============================================================================
// loop() — Bucle principal NO BLOQUEANTE
// =============================================================================
void loop() {
    // ► Gestión OTA
    OtaManager::update();

    // ► Protocolo M365 (heartbeat y keepalive)
    M365Protocol::update();

    // ► Passthrough UART Pantalla <-> Controladora
    UartBridge::update();

    // ► Emulador BMS Xiaomi M365
    BmsEmulator::update();

    // ► Lectura de botones y FSM de modos
    UiManager::update();

    // ► Animaciones LED no bloqueantes
    LedController::update();

    // =========================================================================
    // ► DASHBOARD PERIÓDICO DE MONITORIZACIÓN (cada 1500 ms)
    // =========================================================================
    static uint32_t lastDashboardTime = 0;
    uint32_t now = millis();
    if (now - lastDashboardTime >= 1500) {
        lastDashboardTime = now;

        DriveMode m = UiManager::getCurrentMode();
        const char* modeStr = (m == MODE_ECO)   ? "1 [ECO 55%]  " :
                              (m == MODE_DRIVE) ? "2 [DRIVE 78%] " :
                                                  "3 [SPORT 100%]";

        const char* luzStr = UiManager::isLuzActive() ? "ON " : "OFF";

        uint8_t gasPct = UartBridge::getLastThrottlePercent();
        bool pitoAct = UiManager::isPitoActive();
        char gasBuf[22];
        if (pitoAct) {
            snprintf(gasBuf, sizeof(gasBuf), "%2u%% [PITO Set:%u]", gasPct, UiManager::getBenchThrottlePercent());
        } else {
            snprintf(gasBuf, sizeof(gasBuf), "%2u%% (Set:%u)", gasPct, UiManager::getBenchThrottlePercent());
        }

        const char* frenoStr = UartBridge::isBrakeActive() ? "FRENANDO" : "LIBRE   ";
        const char* dashStr  = UartBridge::isDashboardActive() ? "ONLINE " : "OFFLINE";
        const char* escStr   = UartBridge::isControllerActive() ? "ONLINE " : "OFFLINE";

        char bmsBuf[20];
        if (BmsEmulator::isActive()) {
            snprintf(bmsBuf, sizeof(bmsBuf), "OK (Q:%u)", BmsEmulator::getQueriesReceived());
        } else {
            snprintf(bmsBuf, sizeof(bmsBuf), "STANDBY");
        }

        Serial.printf("[DASHBOARD] Modo: %s | Luz: %s | Gas: %s | Freno: %s | Dash: %s | ESC: %s | BMS: %s\n",
                      modeStr, luzStr, gasBuf, frenoStr, dashStr, escStr, bmsBuf);
    }

    // =========================================================================
    // ► Lectura y buffer de entrada serie (Soporta Newline y 'No Line Ending')
    // =========================================================================
    static char serialRxBuf[32];
    static uint8_t serialRxIdx = 0;
    static uint32_t lastSerialRxTime = 0;

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        lastSerialRxTime = millis();
        if (c == '\r' || c == '\n') {
            if (serialRxIdx > 0) {
                serialRxBuf[serialRxIdx] = '\0';
                processSerialCommand(serialRxBuf);
                serialRxIdx = 0;
            }
        } else if ((unsigned char)c >= 32 && (unsigned char)c <= 126) {
            // Solo acumula caracteres ASCII imprimibles
            if (serialRxIdx < sizeof(serialRxBuf) - 1) {
                serialRxBuf[serialRxIdx++] = c;
                serialRxBuf[serialRxIdx] = '\0';
            }
        }
    }

    // Timeout de 120ms para procesar comandos cuando el monitor serie está en "No Line Ending"
    if (serialRxIdx > 0 && (millis() - lastSerialRxTime > 120)) {
        serialRxBuf[serialRxIdx] = '\0';
        processSerialCommand(serialRxBuf);
        serialRxIdx = 0;
    }
}
