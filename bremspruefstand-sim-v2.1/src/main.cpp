// Rollenpruefstand Wirbelstrombremse
// Simulationsaufbau:
//   Poti 1     (GPIO36) -> Drehzahl-Sollwert 0-6000 RPM
//   Poti 2     (GPIO39) -> Bremsmoment-Sollwert 0-10 Nm
//   LCD 16x2 I2C 0x27   -> Anzeige
//   LEDs       25/26/27 -> Brake / RPM / Warnung
//   Wokwi-Sim  intern   -> Drehzahl / Kraft / Moment / Temperatur

#include <Arduino.h>

#ifndef WOKWI_SIM
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#endif

#define HX711_DT_PIN       35
#define HX711_SCK_PIN      33
#define HX711_SCALE     2280.0f

#define PIN_PULSE          34
#define PIN_LED_BRAKE      25
#define PIN_LED_RPM        26
#define PIN_LED_WARN       27
#define PIN_PWM_MAGNET     32
#define PIN_PWM_MOTOR       4
#define PIN_POT_RPM        36
#define PIN_POT_BRAKE      39
#define LCD_ADDR         0x27

#define CH_LED_BRAKE        0
#define CH_LED_RPM          1
#define CH_LED_WARN         2
#define CH_PWM_MAGNET       3
#define CH_PWM_MOTOR        4

const float TEMP_WARN = 80.0f;
const float TEMP_CRIT = 120.0f;
const float TEMP_AMBIENT = 25.0f;
float HEBELARM_MM = 250.0f;

#ifndef WOKWI_SIM
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
HX711 scale;
#else
class SimLcd {
 public:
  explicit SimLcd(uint8_t, uint8_t, uint8_t) {}

  void init() {}
  void backlight() {}
  void clear() {}
  void setCursor(uint8_t, uint8_t) {}

  size_t print(const char *text) {
    last_line_ = text ? text : "";
    return last_line_.length();
  }

 private:
  String last_line_;
};

SimLcd lcd(LCD_ADDR, 16, 2);
#endif

volatile uint32_t pulse_count = 0;
#ifndef WOKWI_SIM
void IRAM_ATTR onPulse() { pulse_count++; }
#endif

float sim_rpm_state = 0.0f;
float drehzahl_rpm = 0.0f;
float drehzahl_hz = 0.0f;
float moment_Nm = 0.0f;
float kraft_N = 0.0f;
float temp_scheibe = TEMP_AMBIENT;
int brake_mv = 0;

float lokSollMom = 0.0f;
float lokSollRpm = 0.0f;
bool lokFreigabe = false;
bool usePotInputs = true;

void pwmSetup(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution) {
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
}

void messen() {
#ifdef WOKWI_SIM
  const float enable = lokFreigabe ? 1.0f : 0.0f;
  const float rpm_target = lokSollRpm * enable;
  const float brake_target = constrain(brake_mv / 10000.0f * 10.0f, 0.0f, 10.0f);
  const float rpm_error = rpm_target - sim_rpm_state;
  const float accel = rpm_error * 0.18f - brake_target * (55.0f + sim_rpm_state / 180.0f);

  sim_rpm_state += accel;
  if (sim_rpm_state < 0.0f) sim_rpm_state = 0.0f;
  if (!lokFreigabe && sim_rpm_state < 2.0f) sim_rpm_state = 0.0f;

  drehzahl_rpm = sim_rpm_state;
  drehzahl_hz = drehzahl_rpm / 60.0f;

  const float rpm_factor = constrain(drehzahl_rpm / 6000.0f, 0.0f, 1.0f);
  moment_Nm = brake_target * (0.35f + 0.65f * rpm_factor);
  kraft_N = moment_Nm / max(HEBELARM_MM / 1000.0f, 0.05f);
#else
  static uint32_t last_cnt = 0;
  const uint32_t cnt = pulse_count;
  const uint32_t pulses = cnt - last_cnt;
  last_cnt = cnt;

  drehzahl_rpm = pulses * (60000.0f / 200.0f);
  drehzahl_hz = drehzahl_rpm / 60.0f;

  if (scale.is_ready()) {
    const float g = scale.get_units(3);
    kraft_N = g * 9.81f / 1000.0f;
    moment_Nm = kraft_N * (HEBELARM_MM / 1000.0f);
  }
#endif
}

void temperatur_update(float dt_s) {
  const float brake_ratio = brake_mv / 10000.0f;
  const float rpm_ratio = constrain(drehzahl_rpm / 6000.0f, 0.0f, 1.0f);
  const float heating = brake_ratio * rpm_ratio * 45.0f;
  const float cooling = (temp_scheibe - TEMP_AMBIENT) * 0.12f;
  temp_scheibe += (heating - cooling) * dt_s;
  if (temp_scheibe < TEMP_AMBIENT) temp_scheibe = TEMP_AMBIENT;
}

void poti_update() {
  if (!usePotInputs) return;

  int rr = analogRead(PIN_POT_RPM);
  int rb = analogRead(PIN_POT_BRAKE);
  if (rr < 82) rr = 0;
  if (rb < 82) rb = 0;

  lokSollRpm = rr / 4095.0f * 6000.0f;
  lokSollMom = rb / 4095.0f * 10.0f;
  lokFreigabe = (lokSollRpm > 5.0f || lokSollMom > 0.05f);
  ledcWrite(CH_PWM_MOTOR, static_cast<uint8_t>(lokSollRpm / 6000.0f * 255.0f));
}

void regelung() {
  const float soll = lokFreigabe ? lokSollMom : 0.0f;
  brake_mv = static_cast<int>(constrain(soll / 10.0f * 10000.0f, 0, 10000));
  ledcWrite(CH_PWM_MAGNET, static_cast<uint8_t>(constrain(brake_mv / 10000.0f * 255.0f, 0, 255)));
}

void lcd_update() {
  char line1[17];
  char line2[17];
  lcd.clear();
  lcd.setCursor(0, 0);
  snprintf(line1, sizeof(line1), "%4drpm %3.0fC", static_cast<int>(drehzahl_rpm), temp_scheibe);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  snprintf(line2, sizeof(line2), "%4.1fN %4.2fNm", constrain(kraft_N, 0.0f, 99.9f), moment_Nm);
  lcd.print(line2);
}

void printCommandHelp() {
  Serial.println("Cmds: t  l<mm>  m<Nm>  n<RPM>  on  off  pot  remote  r  s");
}

void serial_cmd(String &cmd) {
  cmd.trim();

  if (cmd == "t") {
#ifdef WOKWI_SIM
    sim_rpm_state = 0.0f;
    Serial.println("[SIM] Reset/Tara OK");
#else
    scale.tare();
    Serial.println("[HX711] Tara OK");
#endif
  } else if (cmd.startsWith("l")) {
    const float v = cmd.substring(1).toFloat();
    if (v >= 100 && v <= 500) {
      HEBELARM_MM = v;
      Serial.printf("Hebelarm: %.0fmm\n", v);
    }
  } else if (cmd.startsWith("m")) {
    usePotInputs = false;
    lokSollMom = constrain(cmd.substring(1).toFloat(), 0, 10);
    Serial.printf("Sollmoment seriell: %.2fNm\n", lokSollMom);
  } else if (cmd.startsWith("n")) {
    usePotInputs = false;
    lokSollRpm = constrain(cmd.substring(1).toFloat(), 0, 6000);
    Serial.printf("Sollrpm seriell: %.0f\n", lokSollRpm);
  } else if (cmd == "on") {
    usePotInputs = false;
    lokFreigabe = true;
    Serial.println("Freigabe EIN");
  } else if (cmd == "off") {
    usePotInputs = false;
    lokFreigabe = false;
    Serial.println("Freigabe AUS");
  } else if (cmd == "pot") {
    usePotInputs = true;
    Serial.println("Steuerquelle: Potis");
  } else if (cmd == "remote") {
    usePotInputs = false;
    Serial.println("Steuerquelle: Seriell");
  } else if (cmd == "r") {
    lokSollMom = 0;
    lokSollRpm = 0;
    lokFreigabe = false;
    brake_mv = 0;
    temp_scheibe = TEMP_AMBIENT;
    ledcWrite(CH_PWM_MAGNET, 0);
    ledcWrite(CH_PWM_MOTOR, 0);
    Serial.println("Reset OK");
  } else if (cmd == "s") {
    Serial.printf("%.0frpm  F:%.1fN  M:%.2fNm  T:%.1fC  U:%dmV  src:%s  en:%d\n",
                  drehzahl_rpm, kraft_N, moment_Nm, temp_scheibe, brake_mv,
                  usePotInputs ? "pot" : "serial", lokFreigabe ? 1 : 0);
  } else {
    printCommandHelp();
  }
}

// cppcheck-suppress unusedFunction
void setup() {
  Serial.begin(115200);
  delay(300);

#ifndef WOKWI_SIM
  Wire.begin(21, 22);
  delay(100);

  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);
  scale.set_scale(HX711_SCALE);
  scale.tare();
  Serial.println("[HW] Waegezelle 5kg bereit");

  pinMode(PIN_PULSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PULSE), onPulse, RISING);
#else
  Serial.println("[SIM] Interne Rollenpruefstand-Simulation aktiv");
#endif

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bremspr. v3");
  lcd.setCursor(0, 1);
  lcd.print("Init...");
  delay(800);

  pwmSetup(PIN_LED_BRAKE, CH_LED_BRAKE, 1000, 8);
  pwmSetup(PIN_LED_RPM, CH_LED_RPM, 1000, 8);
  pwmSetup(PIN_LED_WARN, CH_LED_WARN, 2000, 8);
  pwmSetup(PIN_PWM_MAGNET, CH_PWM_MAGNET, 1000, 8);
  pwmSetup(PIN_PWM_MOTOR, CH_PWM_MOTOR, 1000, 8);
  ledcWrite(CH_PWM_MAGNET, 0);
  ledcWrite(CH_PWM_MOTOR, 0);

  pinMode(PIN_POT_RPM, INPUT);
  pinMode(PIN_POT_BRAKE, INPUT);

  Serial.println("=== Rollenpruefstand bereit ===");
  printCommandHelp();
}

// cppcheck-suppress unusedFunction
void loop() {
  static unsigned long t_meas = 0;
  static unsigned long t_lcd = 0;
  static unsigned long t_print = 0;
  static unsigned long t_poti = 0;
  const unsigned long now = millis();

  if (now - t_poti >= 50) {
    t_poti = now;
    poti_update();
  }

  if (now - t_meas >= 200) {
    t_meas = now;
    messen();
    regelung();
    temperatur_update(0.2f);
    ledcWrite(CH_LED_RPM, static_cast<int>(constrain(drehzahl_rpm / 6000.0f * 255.0f, 0, 255)));
    ledcWrite(CH_LED_BRAKE, static_cast<int>(constrain(brake_mv / 10000.0f * 255.0f, 0, 255)));
    ledcWrite(CH_LED_WARN, temp_scheibe >= TEMP_WARN ? (temp_scheibe >= TEMP_CRIT ? 255 : 96) : 0);
  }

  if (now - t_lcd >= 500) {
    t_lcd = now;
    lcd_update();
  }

  if (now - t_print >= 1000) {
    t_print = now;
    Serial.printf("[%.1fs] %.0frpm %.2fHz  F:%.1fN  M:%.2fNm  T:%.1fC  U:%dmV  Soll:%.0f/%.2f  src:%s\n",
                  millis() / 1000.0f, drehzahl_rpm, drehzahl_hz, kraft_N, moment_Nm,
                  temp_scheibe, brake_mv, lokSollRpm, lokSollMom, usePotInputs ? "pot" : "serial");
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    serial_cmd(cmd);
  }

  delay(5);
}
