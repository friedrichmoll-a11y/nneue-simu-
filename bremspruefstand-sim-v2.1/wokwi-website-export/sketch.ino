// Wokwi Website Export for the brake test bench
// Browser-only version: internal simulation, no external custom chips required.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PIN_LED_BRAKE    25
#define PIN_LED_RPM      26
#define PIN_LED_WARN     27
#define PIN_PWM_MAGNET   32
#define PIN_PWM_MOTOR     4
#define PIN_POT_RPM      36
#define PIN_POT_BRAKE    39
#define LCD_ADDR       0x27

#define CH_LED_BRAKE      0
#define CH_LED_RPM        1
#define CH_LED_WARN       2
#define CH_PWM_MAGNET     3
#define CH_PWM_MOTOR      4

const float TEMP_WARN = 80.0f;
const float TEMP_CRIT = 120.0f;
const float TEMP_AMBIENT = 25.0f;

float HEBELARM_MM = 250.0f;
float MOTOR_MAX_NM = 6.0f;
float MOTOR_MAX_RPM = 6000.0f;
float MAGNET_MAX_NM = 10.0f;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

float sim_rpm_state = 0.0f;
float drehzahl_rpm = 0.0f;
float drehzahl_hz = 0.0f;
float moment_Nm = 0.0f;
float motor_Nm = 0.0f;
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
  const float enable = lokFreigabe ? 1.0f : 0.0f;
  const float rpm_target = lokSollRpm * enable;
  const float throttle = constrain(rpm_target / max(MOTOR_MAX_RPM, 100.0f), 0.0f, 1.0f);
  const float brake_target = constrain(brake_mv / 10000.0f * MAGNET_MAX_NM, 0.0f, MAGNET_MAX_NM);
  const float rpm_ratio = constrain(sim_rpm_state / max(MOTOR_MAX_RPM, 100.0f), 0.0f, 1.5f);

  motor_Nm = MOTOR_MAX_NM * throttle * max(0.0f, 1.0f - rpm_ratio);
  const float brake_dynamic = brake_target * (0.25f + 0.75f * constrain(sim_rpm_state / 3000.0f, 0.0f, 1.5f));
  const float accel = (motor_Nm - brake_dynamic - 0.08f * max(1.0f, sim_rpm_state / 800.0f)) * 18.0f;

  sim_rpm_state += accel;
  if (sim_rpm_state < 0.0f) sim_rpm_state = 0.0f;
  if (!lokFreigabe && sim_rpm_state < 2.0f) sim_rpm_state = 0.0f;

  drehzahl_rpm = sim_rpm_state;
  drehzahl_hz = drehzahl_rpm / 60.0f;
  moment_Nm = brake_dynamic;
  kraft_N = moment_Nm / max(HEBELARM_MM / 1000.0f, 0.05f);
}

void temperatur_update(float dt_s) {
  const float brake_ratio = constrain(moment_Nm / max(MAGNET_MAX_NM, 0.5f), 0.0f, 1.5f);
  const float rpm_ratio = constrain(drehzahl_rpm / max(MOTOR_MAX_RPM, 500.0f), 0.0f, 1.5f);
  const float heating = brake_ratio * rpm_ratio * 52.0f;
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

  lokSollRpm = rr / 4095.0f * MOTOR_MAX_RPM;
  lokSollMom = rb / 4095.0f * MAGNET_MAX_NM;
  lokFreigabe = (lokSollRpm > 5.0f || lokSollMom > 0.05f);
  ledcWrite(CH_PWM_MOTOR, static_cast<uint8_t>(constrain(lokSollRpm / max(MOTOR_MAX_RPM, 100.0f) * 255.0f, 0, 255)));
}

void regelung() {
  const float soll = lokFreigabe ? lokSollMom : 0.0f;
  brake_mv = static_cast<int>(constrain(soll / max(MAGNET_MAX_NM, 0.5f) * 10000.0f, 0, 10000));
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
  Serial.println("Cmds:");
  Serial.println("  l<mm>  -> Hebelarm 100..500");
  Serial.println("  m<Nm>  -> Magnet-Sollmoment");
  Serial.println("  n<RPM> -> Drehzahl-Sollwert");
  Serial.println("  mm<Nm> -> Motor-Maxmoment");
  Serial.println("  mr<RPM>-> Motor-Max-RPM");
  Serial.println("  mx<Nm> -> Magnet-Maxmoment");
  Serial.println("  on/off pot remote r s");
}

void serial_cmd(String &cmd) {
  cmd.trim();

  if (cmd.startsWith("mm")) {
    const float v = cmd.substring(2).toFloat();
    if (v >= 0.5f && v <= 40.0f) {
      MOTOR_MAX_NM = v;
      Serial.printf("Motor-Maxmoment: %.2fNm\n", MOTOR_MAX_NM);
    }
  } else if (cmd.startsWith("mr")) {
    const float v = cmd.substring(2).toFloat();
    if (v >= 500.0f && v <= 12000.0f) {
      MOTOR_MAX_RPM = v;
      Serial.printf("Motor-Max-RPM: %.0f\n", MOTOR_MAX_RPM);
    }
  } else if (cmd.startsWith("mx")) {
    const float v = cmd.substring(2).toFloat();
    if (v >= 0.5f && v <= 20.0f) {
      MAGNET_MAX_NM = v;
      Serial.printf("Magnet-Maxmoment: %.2fNm\n", MAGNET_MAX_NM);
    }
  } else if (cmd.startsWith("l")) {
    const float v = cmd.substring(1).toFloat();
    if (v >= 100.0f && v <= 500.0f) {
      HEBELARM_MM = v;
      Serial.printf("Hebelarm: %.0fmm\n", HEBELARM_MM);
    }
  } else if (cmd.startsWith("m")) {
    usePotInputs = false;
    lokSollMom = constrain(cmd.substring(1).toFloat(), 0.0f, MAGNET_MAX_NM);
    Serial.printf("Sollmoment seriell: %.2fNm\n", lokSollMom);
  } else if (cmd.startsWith("n")) {
    usePotInputs = false;
    lokSollRpm = constrain(cmd.substring(1).toFloat(), 0.0f, MOTOR_MAX_RPM);
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
    lokSollMom = 0.0f;
    lokSollRpm = 0.0f;
    lokFreigabe = false;
    brake_mv = 0;
    temp_scheibe = TEMP_AMBIENT;
    sim_rpm_state = 0.0f;
    ledcWrite(CH_PWM_MAGNET, 0);
    ledcWrite(CH_PWM_MOTOR, 0);
    Serial.println("Reset OK");
  } else if (cmd == "s") {
    Serial.printf("%.0frpm  F:%.1fN  M:%.2fNm  Mm:%.2fNm  T:%.1fC  H:%.0fmm  src:%s  en:%d\n",
                  drehzahl_rpm, kraft_N, moment_Nm, motor_Nm, temp_scheibe, HEBELARM_MM,
                  usePotInputs ? "pot" : "serial", lokFreigabe ? 1 : 0);
  } else {
    printCommandHelp();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(21, 22);
  delay(100);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Brake Bench");
  lcd.setCursor(0, 1);
  lcd.print("Wokwi Web");
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

  Serial.println("=== Rollenpruefstand Wokwi Web bereit ===");
  printCommandHelp();
}

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
    ledcWrite(CH_LED_RPM, static_cast<int>(constrain(drehzahl_rpm / max(MOTOR_MAX_RPM, 100.0f) * 255.0f, 0, 255)));
    ledcWrite(CH_LED_BRAKE, static_cast<int>(constrain(moment_Nm / max(MAGNET_MAX_NM, 0.5f) * 255.0f, 0, 255)));
    ledcWrite(CH_LED_WARN, temp_scheibe >= TEMP_WARN ? (temp_scheibe >= TEMP_CRIT ? 255 : 96) : 0);
  }

  if (now - t_lcd >= 500) {
    t_lcd = now;
    lcd_update();
  }

  if (now - t_print >= 1000) {
    t_print = now;
    Serial.printf("[%.1fs] %.0frpm %.2fHz  F:%.1fN  M:%.2fNm  Mm:%.2fNm  T:%.1fC  Soll:%.0f/%.2f\n",
                  millis() / 1000.0f, drehzahl_rpm, drehzahl_hz, kraft_N, moment_Nm,
                  motor_Nm, temp_scheibe, lokSollRpm, lokSollMom);
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    serial_cmd(cmd);
  }

  delay(5);
}
