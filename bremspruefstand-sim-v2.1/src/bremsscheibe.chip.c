#include "wokwi-api.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  pin_t pin;
  bool level_high;
  uint64_t last_rise_ns;
  uint64_t last_edge_ns;
  uint32_t period_ns;
  float duty_cycle;
} pwm_input_t;

typedef struct {
  pin_t motor_ref_pin;
  pin_t torque_in_pin;
  pin_t pulse_pin;
  pwm_input_t motor_ref_pwm;
  pwm_input_t torque_pwm;
  timer_t timer;
  uint32_t max_rpm_attr;
  uint32_t inertia_attr;
  uint32_t motor_gain_attr;
  uint32_t brake_gain_attr;
  float rpm;
  float pulse_phase_rev;
  uint64_t pulse_high_until_ns;
  bool pulse_is_high;
} disc_t;

static const uint32_t DISC_TICK_US = 100;
static const uint32_t PULSE_WIDTH_NS = 100000;

static float clampf_local(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void pwm_input_init(pwm_input_t *input, const char *pin_name) {
  input->pin = pin_init(pin_name, INPUT);
  input->level_high = false;
  input->last_rise_ns = 0;
  input->last_edge_ns = get_sim_nanos();
  input->period_ns = 1000000;
  input->duty_cycle = 0.0f;
}

static void pwm_input_edge(void *user_data, pin_t pin, uint32_t value) {
  (void)pin;
  pwm_input_t *input = user_data;
  uint64_t now = get_sim_nanos();

  if (value == HIGH) {
    if (input->last_rise_ns != 0) {
      uint64_t period = now - input->last_rise_ns;
      if (period > 0 && period < 1000000000ULL) {
        input->period_ns = (uint32_t)period;
      }
    }
    input->last_rise_ns = now;
    input->last_edge_ns = now;
    input->level_high = true;
  } else {
    if (input->level_high && input->last_rise_ns != 0 && input->period_ns > 0) {
      uint64_t high_time = now - input->last_rise_ns;
      float duty = (float)high_time / (float)input->period_ns;
      input->duty_cycle = clampf_local(duty, 0.0f, 1.0f);
    } else {
      input->duty_cycle = 0.0f;
    }
    input->last_edge_ns = now;
    input->level_high = false;
  }
}

static float pwm_input_duty(const pwm_input_t *input) {
  if (input->level_high) {
    return input->period_ns > 0 ? 1.0f : 0.0f;
  }
  return clampf_local(input->duty_cycle, 0.0f, 1.0f);
}

static void disc_tick(void *user_data) {
  disc_t *disc = user_data;
  const uint64_t now_ns = get_sim_nanos();
  const float dt_s = (float)DISC_TICK_US / 1000000.0f;

  const float max_rpm = (float)attr_read(disc->max_rpm_attr);
  const float inertia = clampf_local(attr_read_float(disc->inertia_attr), 0.05f, 10.0f);
  const float motor_gain = clampf_local(attr_read_float(disc->motor_gain_attr), 0.1f, 20.0f);
  const float brake_gain = clampf_local(attr_read_float(disc->brake_gain_attr), 0.1f, 20.0f);

  const float motor_duty = pwm_input_duty(&disc->motor_ref_pwm);
  const float brake_duty = pwm_input_duty(&disc->torque_pwm);

  const float target_rpm = motor_duty * max_rpm;
  const float motor_term = (target_rpm - disc->rpm) * (motor_gain / inertia);
  const float brake_term = brake_duty * brake_gain * 4500.0f / inertia;
  const float drag_term = disc->rpm * 0.18f;

  disc->rpm += (motor_term - brake_term - drag_term) * dt_s;
  disc->rpm = clampf_local(disc->rpm, 0.0f, max_rpm);

  if (disc->pulse_is_high && now_ns >= disc->pulse_high_until_ns) {
    pin_write(disc->pulse_pin, LOW);
    disc->pulse_is_high = false;
  }

  disc->pulse_phase_rev += (disc->rpm / 60.0f) * dt_s;
  if (!disc->pulse_is_high && disc->pulse_phase_rev >= 1.0f) {
    disc->pulse_phase_rev -= floorf(disc->pulse_phase_rev);
    pin_write(disc->pulse_pin, HIGH);
    disc->pulse_is_high = true;
    disc->pulse_high_until_ns = now_ns + PULSE_WIDTH_NS;
  }
}

void chip_init(void) {
  disc_t *disc = malloc(sizeof(disc_t));
  memset(disc, 0, sizeof(*disc));

  pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  disc->pulse_pin = pin_init("PULSE", OUTPUT_LOW);

  pwm_input_init(&disc->motor_ref_pwm, "MOTOR_REF");
  pwm_input_init(&disc->torque_pwm, "TORQUE_IN");
  disc->motor_ref_pin = disc->motor_ref_pwm.pin;
  disc->torque_in_pin = disc->torque_pwm.pin;

  disc->max_rpm_attr = attr_init("maxRpm", 6000);
  disc->inertia_attr = attr_init_float("inertia", 1.2f);
  disc->motor_gain_attr = attr_init_float("motorGain", 3.2f);
  disc->brake_gain_attr = attr_init_float("brakeGain", 2.2f);

  const pin_watch_config_t motor_watch = {
    .edge = BOTH,
    .pin_change = pwm_input_edge,
    .user_data = &disc->motor_ref_pwm,
  };
  pin_watch(disc->motor_ref_pin, &motor_watch);

  const pin_watch_config_t torque_watch = {
    .edge = BOTH,
    .pin_change = pwm_input_edge,
    .user_data = &disc->torque_pwm,
  };
  pin_watch(disc->torque_in_pin, &torque_watch);

  const timer_config_t timer_config = {
    .callback = disc_tick,
    .user_data = disc,
  };
  disc->timer = timer_init(&timer_config);
  timer_start(disc->timer, DISC_TICK_US, true);

  printf("[Bremsscheibe] bereit\n");
}
