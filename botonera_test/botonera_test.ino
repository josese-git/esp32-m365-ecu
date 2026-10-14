// =============================================================================
//  FIRMWARE - BOTONERA E-BIKE
//  Target: ESP32 | Framework: Arduino
//  Descripción: Botonera de 4 botones + 3 LEDs con máquina de estados,
//               animación de inicio Knight Rider, modo luces y modo pito.
//               100% NO BLOQUEANTE (millis(), cero delay()).
//  Baudios: 115200
// =============================================================================
//
//  MAPA DE PINES:
//    BOTONES (activos LOW - INPUT_PULLUP):
//      GPIO 18  -> BTN_IZQ   (interruptor fijo / toggle) - Baja modo
//      GPIO 19  -> BTN_DER   (interruptor fijo / toggle) - Sube modo
//      GPIO 21  -> BTN_LUZ   (interruptor fijo / toggle) - Modo nocturno
//      GPIO 22  -> BTN_PITO  (pulsador momentaneo)       - Estroboscopico
//    LEDs (activos HIGH - OUTPUT):
//      GPIO 23  -> LED_1
//      GPIO 25  -> LED_2
//      GPIO 26  -> LED_3
//
// =============================================================================

// ---------------------------------------------------------------------------
// Definicion de pines
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_BTN_IZQ  = 18;
constexpr uint8_t PIN_BTN_DER  = 19;
constexpr uint8_t PIN_BTN_LUZ  = 21;
constexpr uint8_t PIN_BTN_PITO = 22;

constexpr uint8_t PIN_LED_1 = 23;
constexpr uint8_t PIN_LED_2 = 25;
constexpr uint8_t PIN_LED_3 = 26;

// ---------------------------------------------------------------------------
// Parametros de temporizacion
// ---------------------------------------------------------------------------
constexpr unsigned long DEBOUNCE_MS   = 50;   // Filtro antirrebote
constexpr unsigned long BOOT_STEP_MS  = 80;   // Velocidad Knight Rider por paso
constexpr unsigned long HORN_BLINK_MS = 40;   // Flash pito (ON u OFF)
constexpr uint8_t       BOOT_REPEATS  = 3;    // Repeticiones Knight Rider

// ---------------------------------------------------------------------------
// Maquina de estados para los LEDs
// ---------------------------------------------------------------------------
enum LedState : uint8_t {
  BOOT,         // Animacion de inicio Knight Rider
  IDLE,         // Solo el LED del modo actual encendido
  LIGHT_MODE,   // Luces nocturnas: 3 LEDs encendidos fijos
  HORN_STROBE   // Pito activo: parpadeo ultrarapido (prioridad maxima)
};

// ---------------------------------------------------------------------------
// Estado de la aplicacion
// ---------------------------------------------------------------------------
LedState ledState   = BOOT;
uint8_t  modoActual = 1;       // Modos 1, 2, 3
bool     luzActiva  = false;   // Estado logico del modo luz
LedState prevState  = IDLE;    // Estado anterior al pito (para restaurar)

// Variables de animacion - Boot (Knight Rider)
uint8_t       bootStep   = 0;
uint8_t       bootCount  = 0;
unsigned long bootStepMs = 0;

// Variables de animacion - Pito (strobe)
bool          hornLedOn   = false;
unsigned long hornBlinkMs = 0;

// ---------------------------------------------------------------------------
// Estructura generica para manejar un boton con debounce no bloqueante
// ---------------------------------------------------------------------------
struct Button {
  uint8_t       pin;
  bool          isToggle;       // true = fijo (toggle), false = momentaneo

  int           lastRawState;
  unsigned long lastChangeMs;
  int           stableState;

  bool          logicState;     // Estado logico actual
};

// ---------------------------------------------------------------------------
// Array de botones
// ---------------------------------------------------------------------------
Button buttons[4] = {
  { PIN_BTN_IZQ,  true,  HIGH, 0, HIGH, false },  // [0] IDX_IZQ
  { PIN_BTN_DER,  true,  HIGH, 0, HIGH, false },  // [1] IDX_DER
  { PIN_BTN_LUZ,  true,  HIGH, 0, HIGH, false },  // [2] IDX_LUZ
  { PIN_BTN_PITO, false, HIGH, 0, HIGH, false },  // [3] IDX_PITO
};

constexpr uint8_t IDX_IZQ  = 0;
constexpr uint8_t IDX_DER  = 1;
constexpr uint8_t IDX_LUZ  = 2;
constexpr uint8_t IDX_PITO = 3;

// ---------------------------------------------------------------------------
// Helpers: escritura de LEDs
// ---------------------------------------------------------------------------
void setLeds(bool l1, bool l2, bool l3) {
  digitalWrite(PIN_LED_1, l1 ? HIGH : LOW);
  digitalWrite(PIN_LED_2, l2 ? HIGH : LOW);
  digitalWrite(PIN_LED_3, l3 ? HIGH : LOW);
}

void ledsOff() { setLeds(false, false, false); }
void ledsAll() { setLeds(true,  true,  true);  }

// Enciende solo el LED correspondiente al modo actual
void applyIdleLeds() {
  setLeds(modoActual == 1, modoActual == 2, modoActual == 3);
}

// ---------------------------------------------------------------------------
// Actualiza el debounce de un boton.
// Devuelve true si el estado logico cambio en este ciclo.
// ---------------------------------------------------------------------------
bool updateButton(Button& btn) {
  const int rawNow        = digitalRead(btn.pin);
  const unsigned long now = millis();

  if (rawNow != btn.lastRawState) {
    btn.lastRawState = rawNow;
    btn.lastChangeMs = now;
  }

  if ((now - btn.lastChangeMs) >= DEBOUNCE_MS) {
    if (rawNow != btn.stableState) {
      btn.stableState = rawNow;

      if (btn.isToggle) {
        // Solo actua en flanco de PRESION (LOW)
        if (rawNow == LOW) {
          btn.logicState = !btn.logicState;
          return true;
        }
      } else {
        // Momentaneo: refleja el nivel fisico
        btn.logicState = (rawNow == LOW);
        return true;
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Transicion de estado LED con log opcional
// ---------------------------------------------------------------------------
void transitionTo(LedState newState, const char* reason = nullptr) {
  if (reason) {
    Serial.print(F("[ESTADO] -> "));
    Serial.println(reason);
  }
  ledState = newState;
}

// ---------------------------------------------------------------------------
// Animacion BOOT: Knight Rider (no bloqueante)
// ---------------------------------------------------------------------------
void updateBoot() {
  const unsigned long now = millis();
  if ((now - bootStepMs) < BOOT_STEP_MS) return;
  bootStepMs = now;

  // Secuencia: L1 -> L2 -> L3 -> L2 (4 pasos por ciclo)
  static const bool seq[4][3] = {
    { true,  false, false },  // L1
    { false, true,  false },  // L2
    { false, false, true  },  // L3
    { false, true,  false },  // L2 (rebote)
  };

  setLeds(seq[bootStep][0], seq[bootStep][1], seq[bootStep][2]);
  bootStep++;

  if (bootStep >= 4) {
    bootStep = 0;
    bootCount++;
    if (bootCount >= BOOT_REPEATS) {
      ledsOff();
      Serial.println(F("[BOOT] Animacion Knight Rider terminada."));
      Serial.print(F("[MODO] Modo inicial: "));
      Serial.println(modoActual);
      transitionTo(luzActiva ? LIGHT_MODE : IDLE);
      applyIdleLeds();
    }
  }
}

// ---------------------------------------------------------------------------
// Animacion PITO: strobe ultrarapido (no bloqueante)
// ---------------------------------------------------------------------------
void updateHornStrobe() {
  const unsigned long now = millis();
  if ((now - hornBlinkMs) < HORN_BLINK_MS) return;
  hornBlinkMs = now;

  hornLedOn = !hornLedOn;
  setLeds(hornLedOn, hornLedOn, hornLedOn);
}

// ---------------------------------------------------------------------------
// Procesa la logica de botones y actualiza el estado
// ---------------------------------------------------------------------------
void processButtons() {
  bool anyChange = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (updateButton(buttons[i])) anyChange = true;
  }
  if (!anyChange) return;

  // ── PITO (prioridad maxima) ───────────────────────────────────────────────
  bool pitoActivo = buttons[IDX_PITO].logicState;

  if (pitoActivo && ledState != HORN_STROBE) {
    prevState   = ledState;
    hornLedOn   = true;
    hornBlinkMs = millis();
    transitionTo(HORN_STROBE, "PITO ACTIVADO - Estroboscopico");
    setLeds(true, true, true);
    return;
  }

  if (!pitoActivo && ledState == HORN_STROBE) {
    if (prevState == LIGHT_MODE) {
      transitionTo(LIGHT_MODE, "Pito liberado -> MODO LUZ");
      ledsAll();
    } else {
      transitionTo(IDLE, "Pito liberado -> IDLE");
      applyIdleLeds();
    }
    return;
  }

  // No procesar IZQ/DER/LUZ si el pito sigue activo
  if (ledState == HORN_STROBE) return;

  // ── BTN_LUZ (toggle modo nocturno) ────────────────────────────────────────
  if (buttons[IDX_LUZ].logicState != luzActiva) {
    luzActiva = buttons[IDX_LUZ].logicState;
    if (luzActiva) {
      Serial.println(F("[LUZ] Modo nocturno ACTIVADO - 3 LEDs encendidos."));
      transitionTo(LIGHT_MODE);
      ledsAll();
    } else {
      Serial.println(F("[LUZ] Modo nocturno DESACTIVADO."));
      transitionTo(IDLE);
      applyIdleLeds();
    }
  }

  // ── BTN_DER sube modo ─────────────────────────────────────────────────────
  if (buttons[IDX_DER].logicState) {
    buttons[IDX_DER].logicState = false;  // Consume el evento
    if (modoActual < 3) {
      modoActual++;
      Serial.print(F("[MODO] Subido a modo: "));
      Serial.println(modoActual);
      if (ledState == IDLE) applyIdleLeds();
    } else {
      Serial.println(F("[MODO] Ya en modo maximo (3)."));
    }
  }

  // ── BTN_IZQ baja modo ─────────────────────────────────────────────────────
  if (buttons[IDX_IZQ].logicState) {
    buttons[IDX_IZQ].logicState = false;  // Consume el evento
    if (modoActual > 1) {
      modoActual--;
      Serial.print(F("[MODO] Bajado a modo: "));
      Serial.println(modoActual);
      if (ledState == IDLE) applyIdleLeds();
    } else {
      Serial.println(F("[MODO] Ya en modo minimo (1)."));
    }
  }
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Pines de botones
  pinMode(PIN_BTN_IZQ,  INPUT_PULLUP);
  pinMode(PIN_BTN_DER,  INPUT_PULLUP);
  pinMode(PIN_BTN_LUZ,  INPUT_PULLUP);
  pinMode(PIN_BTN_PITO, INPUT_PULLUP);

  // Pines de LEDs
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  ledsOff();

  // Banner
  Serial.println();
  Serial.println(F("============================================================="));
  Serial.println(F("  FIRMWARE BOTONERA E-BIKE                                  "));
  Serial.println(F("  BTN: IZQ=18 | DER=19 | LUZ=21 | PITO=22                  "));
  Serial.println(F("  LED: L1=23  | L2=25  | L3=26                              "));
  Serial.println(F("  Debounce: 50ms | Knight Rider: 80ms/paso x3               "));
  Serial.println(F("  100% No bloqueante - millis() puro                        "));
  Serial.println(F("============================================================="));
  Serial.println(F("[BOOT] Iniciando animacion Knight Rider..."));

  ledState   = BOOT;
  bootStep   = 0;
  bootCount  = 0;
  bootStepMs = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // 1. Leer botones SIEMPRE, incluso durante BOOT
  if (ledState != BOOT) {
    // Procesamiento completo fuera del boot
    processButtons();
  } else {
    // Durante el boot solo monitoreamos el pito (prioridad absoluta)
    if (updateButton(buttons[IDX_PITO])) {
      if (buttons[IDX_PITO].logicState) {
        Serial.println(F("[PITO] Presionado durante BOOT."));
        prevState   = IDLE;
        hornLedOn   = true;
        hornBlinkMs = millis();
        transitionTo(HORN_STROBE, "PITO activado desde BOOT");
      }
    }
  }

  // 2. Ejecutar la animacion del estado actual
  switch (ledState) {
    case BOOT:        updateBoot();        break;
    case IDLE:        /* estatico */       break;
    case LIGHT_MODE:  /* estatico */       break;
    case HORN_STROBE: updateHornStrobe();  break;
  }
}
