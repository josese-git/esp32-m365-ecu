// =============================================================================
// ota_manager.h — Gestor OTA (Over-The-Air) y WiFi
// =============================================================================
// Módulo no bloqueante para conexión WiFi y actualizaciones OTA.
// Expone init() y update() únicamente.
// =============================================================================

#pragma once

namespace OtaManager {
    void init();    // Configura OTA (WiFi en reposo para no degradar el bus serie)
    void update();  // Debe llamarse cada loop() — maneja handles OTA
    void startWiFi();
    void stopWiFi();
    bool isConnected();
}
