# ESP32 M365 E-Bike ECU & BMS Bridge (`esp32-m365-ecu`)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32%20DevKit%20V1-blue.svg)](https://www.espressif.com/)
[![Framework: Arduino](https://img.shields.io/badge/Framework-Arduino%20C%2B%2B-00979D.svg)](https://www.arduino.cc/)
[![BMS: CamiAlfa Emulated](https://img.shields.io/badge/BMS-CamiAlfa%20Emulated-success.svg)](#emulador-bms-xiaomi-m365)

Unidad de Control Electrónico (**ECU**) de alto rendimiento desarrollada en C++ para microcontroladores **ESP32 DevKit V1 (30 pines)**. Diseñada para integrar componentes de propulsión de patinetes **Xiaomi M365** (Controladora ESC + Pantalla Dashboard) en conversiones de **bicicletas eléctricas (E-Bike)** y vehículos ligeros personalizados alimentados por baterías genéricas (36V / 48V).

---

## 🚀 Características Principales

### 1. Puente UART Bidireccional con Saneamiento de Paquetes al Vuelo
- **Passthrough Half-Duplex (Monohilo)** entre Pantalla (**GPIO 13**) y Controladora (**GPIO 27**) a 115200 baudios.
- Filtro de eco integrado con cancelación de transmisiones locales.
- **Intercepción y Reescritura al Vuelo**: Parser de tramas M365 (`0x55 0xAA`) que escala dinámicamente la demanda de acelerador según el modo de conducción y recalcula el checksum XOR en tiempo real (< 2 ms de latencia).

### 2. Rampa Suave Anti-Tirones (2-Stage Soft-Start Slew-Rate Limiter)
- **Eliminación Total de Tirones**: Resuelve la brusquedad inicial en motores hub brushless de alta potencia mediante una rampa de aceleración de 2 etapas:
  - **Etapa 1 (38..58 raw)**: Incremento fino de `+1` por trama (~20 ms) garantizando un enganche suave, progresivo y sin sacudidas en la transmisión.
  - **Etapa 2 (59..210 raw)**: Progresión ágil de `+2` por trama para alcanzar aceleración plena sin sensación de retardo.
  - **Ajuste en Marcha (-3 / trama)**: Transición controlada al reducir de modo (ej. SPORT a ECO) con gas aplicado.
- **Prioridad Absoluta de Freno y Corte Instantáneo (0 ms)**: Al pulsar la maneta de freno (`p[8] > 50`) o soltar el gas, el acelerador cae inmediatamente a reposo (`0x26 = 38`), garantizando máxima seguridad activa.

### 3. Emulador de BMS Xiaomi M365 (CamiAlfa) — Adiós al Error 21
- Emulación completa del protocolo de batería Xiaomi por UART hardware secundaria (**GPIO 34 RX / GPIO 33 TX**).
- Responde automáticamente a las solicitudes de telemetría de la controladora ESC (voltaje total, corrientes, temperaturas, voltajes de celda individuales, capacidad y salud).
- **Permite usar cualquier pack de batería personalizado (10S, 12S, 13S, etc.)** sin necesidad de la placa BMS original de Xiaomi.

### 4. Botonera de Modos con Animación Knight Rider
- **3 Modos de Conducción**:
  - **ECO**: 55% de entrega máxima de acelerador.
  - **DRIVE**: 78% de entrega de acelerador.
  - **SPORT**: 100% de potencia y aceleración directa.
- **LEDs de Estado Exclusivos**: 3 LEDs (`LED_ECO`, `LED_DRIVE`, `LED_SPORT`) dedicados exclusivamente a reflejar el modo activo.
- **Arranque Knight Rider**: Animación no bloqueante estilo "Coche Fantástico" al encender el sistema.
- **FSM Latching con Cooldown**: Máquina de estados con antirrebote (debounce) y ventana de enfriamiento de 400 ms que evita saltos involuntarios de modo.
- **Control de Faro**: Conmutación de luz delantera M365 mediante botón físico o comando serie sin alterar los LEDs de modo.

### 5. Banco de Pruebas y Dashboard Serie en Tiempo Real
- Monitorización continua por Serial USB (`115200 baud`) cada 1500 ms con telemetría en vivo: Modo activo, Luces, Gas suavizado saliente, Estado de freno, Enlace Dash, Enlace ESC y estado del emulador BMS.
- **Botón de Pito como Acelerador de Banco**: Permite acelerar en pruebas estáticas con solo pulsar el botón de claxon.
- **Calibración Serie al Vuelo**: Envía cualquier número de `0` a `100` por el Monitor Serie para fijar el porcentaje de gas de prueba instantáneamente.

### 6. Actualizaciones Inalámbricas (OTA)
- Soporte para flasheo por WiFi mediante ArduinoOTA, facilitando el mantenimiento sin cables.

---

## 📌 Asignación de Pines (ESP32 DevKit V1 30-Pin)

| Periférico / Señal | Pin GPIO | Dirección / Modo | Descripción |
| :--- | :---: | :---: | :--- |
| **Línea "T" Pantalla** | `GPIO 13` | UART Half-Duplex | Serial1 bidireccional (Pantalla M365) |
| **Línea "P" Pantalla** | `GPIO 32` | `INPUT_PULLUP` | Detección botón físico original (evita strappings) |
| **Línea "T" Controladora** | `GPIO 27` | UART Half-Duplex | Serial2 bidireccional (ESC M365) |
| **Línea "P" Controladora** | `GPIO 14` | `OUTPUT` | Disparo de encendido hardware (pulso a GND) |
| **BMS ESC TX (Amarillo)** | `GPIO 34` | UART Input | RX Emulador BMS (desde cable amarillo ESC) |
| **BMS ESC RX (Blanco)** | `GPIO 33` | UART Output | TX Emulador BMS (hacia cable blanco ESC) |
| **Botón Subir Modo (DER)** | `GPIO 19` | `INPUT_PULLUP` | Conmutador Latching (ECO → DRIVE → SPORT) |
| **Botón Bajar Modo (IZQ)** | `GPIO 18` | `INPUT_PULLUP` | Conmutador Latching (SPORT → DRIVE → ECO) |
| **Botón Luces** | `GPIO 21` | `INPUT_PULLUP` | Conmutador Latching (Toggle Faro M365) |
| **Botón Pito / Claxon** | `GPIO 22` | `INPUT_PULLUP` | Pulsador momentáneo (Acelerador en banco) |
| **LED Verde (ECO)** | `GPIO 23` | `OUTPUT` | Indicador Modo 1 ECO |
| **LED Verde (DRIVE)** | `GPIO 25` | `OUTPUT` | Indicador Modo 2 DRIVE |
| **LED Azul (SPORT)** | `GPIO 26` | `OUTPUT` | Indicador Modo 3 SPORT |
| **Sensor PAS** | `GPIO 4` | `INPUT_PULLUP` | Sensor de Pedaleo Asistido (Fase futura) |

---

## 📂 Estructura del Repositorio

```text
esp32-m365-ecu/
├── esp32_ecu/                # Firmware principal para Arduino IDE / arduino-cli
│   ├── esp32_ecu.ino         # Loop principal, setup y parser de comandos serie
│   ├── config.h              # Configuración global, pines y parámetros ajustables
│   ├── uart_bridge.h/.cpp    # Puente UART Half-Duplex y rampa suave 2-etapas
│   ├── ui_manager.h/.cpp     # Máquina de estados de botones (Latching + Debounce)
│   ├── led_controller.h/.cpp # Controlador de LEDs y animación Knight Rider
│   ├── bms_emulator.h/.cpp   # Emulador CamiAlfa BMS (Hardware UART)
│   ├── m365_protocol.h/.cpp  # Constantes, tramas y cálculo de checksums M365
│   └── ota_manager.h/.cpp    # Gestor de WiFi y actualizaciones OTA
├── botonera_test/            # Sketch de prueba aislado para calibración de hardware
│   └── botonera_test.ino
├── docs/                     # Documentación técnica adicional y mapa de registros
│   └── m365_register_map.txt
├── .gitignore
├── LICENSE
└── README.md
```

---

## 🛠️ Compilación y Flasheo

### Requisitos
1. [Arduino CLI](https://arduino.github.io/arduino-cli/) o [Arduino IDE](https://www.arduino.cc/en/software) (v2.x recomendado).
2. Paquete de placas **esp32 by Espressif Systems** (versión 2.0.x o superior).

### Usando Arduino CLI
```bash
# Compilar el proyecto con FlashMode DIO
arduino-cli compile --fqbn esp32:esp32:esp32:FlashMode=dio esp32_ecu

# Subir a la placa (sustituir COMx por el puerto correspondiente)
arduino-cli upload -p COM6 --fqbn esp32:esp32:esp32:FlashMode=dio esp32_ecu
```

---

## 💻 Consola Serie y Teclas Rápidas

Abre el Monitor Serie a **115200 baudios**:

```text
============================================================
  ESP32 E-Bike ECU v0.1.0-alpha — ECU E-Bike Xiaomi M365
============================================================
  [BMS Emulador] : UART Hardware (D34 RX / D33 TX)
  [Puente UART ] : Pantalla D13 <-> Controladora D27
  [Botonera    ] : 3 Modos (ECO/DRV/SPT) + Luces
  [Acelerador  ] : Botón Pito activo (Escribe 1-100 para cambiar %, actual: 50%)
============================================================
```

### Comandos Serie Disponibles:
- `0` .. `100`: Calibra el porcentaje de gas para las pruebas de banco (ej. introduce `75` y pulsa Enter).
- `l` o `L`: Conmuta las luces encendidas / apagadas.
- `b` o `B`: Muestra el estado del emulador de batería BMS (consultas y respuestas).
- `p` o `P`: Envía un pulso de encendido por hardware de 600 ms en GPIO 14 (línea P).
- `m` o `M`: Simula doble clic en la línea P para alternar modos en la controladora.
- `w` o `W`: Inicia el punto de acceso WiFi para flasheo OTA.
- `h` o `?`: Muestra el menú de ayuda.

---

## 🛡️ Licencia

Distribuido bajo la Licencia **MIT**. Consulta el archivo [`LICENSE`](LICENSE) para más detalles.
Agradecimientos especiales a **Camilo Ruiz (CamiAlfa)** por la ingeniería inversa del protocolo BMS de Ninebot/Xiaomi.
