// =============================================================================
// ota_manager.cpp — Implementación del Gestor OTA y WiFi
// =============================================================================

#include "ota_manager.h"
#include "config.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

namespace OtaManager {

// ---------------------------------------------------------------------------
// Estado interno
// ---------------------------------------------------------------------------
static bool _connected = false;

// ---------------------------------------------------------------------------
// init() — Conecta al WiFi y arranca el servidor OTA
// ---------------------------------------------------------------------------
void init() {
    // Apagar WiFi por defecto para no introducir interrupciones RF que degraden la UART serie
    WiFi.mode(WIFI_OFF);
    _connected = false;

    // --- Configuración ArduinoOTA ---
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] ► Inicio de actualización: " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] ✓ Actualización completada. Reiniciando...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progreso: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] ✗ Error[%u]: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });
}

void startWiFi() {
    Serial.printf("[OTA] Conectando WiFi: %s ...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void stopWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    _connected = false;
    Serial.println("[OTA] WiFi desactivado (Modo silencioso)");
}

// ---------------------------------------------------------------------------
// update() — Handler OTA no bloqueante (llámalo cada loop)
// ---------------------------------------------------------------------------
void update() {
    if (!_connected && WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        _connected = true;
        Serial.printf("[OTA] ✓ WiFi Conectado en segundo plano. IP: %s\n", WiFi.localIP().toString().c_str());
        ArduinoOTA.begin();
        Serial.printf("[OTA] Servidor listo en hostname: %s.local\n", OTA_HOSTNAME);
    }

    if (_connected) {
        ArduinoOTA.handle();
    }
}

// ---------------------------------------------------------------------------
// isConnected() — Estado de la conexión WiFi
// ---------------------------------------------------------------------------
bool isConnected() {
    return _connected && (WiFi.status() == WL_CONNECTED);
}

} // namespace OtaManager
