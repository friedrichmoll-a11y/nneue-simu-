#include "wokwi-api.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  pin_t pin;
  bool level_high;
  uint64_t last_rise_ns;
  uint32_t period_ns;
  float duty_cycle;
} pwm_input_t;

typedef struct {
  pin_t ctrl_in_pin;
  pin_t brake_out_pin;
  pin_t status_led_pin;
  pwm_input_t ctrl_pwm;
  timer_t timer;
  uint32_t magnet_strength_attr;
  uint32_t response_ms_attr;
  float output_duty;
  uint32_t pwm_phase_us;
  bool brake_out_level;
} magnet_t;

static const uint32_t MAGNET_TICK_US = 50;
static const uint32_t OUTPUT_PWM_PERIOD_US = 1000;

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
  input->period_ns = OUTPUT_PWM_PERIOD_US * 1000;
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
    input->level_high = true;
  } else {
    if (input->level_high && input->last_rise_ns != 0 && input->period_ns > 0) {
      uint64_t high_time = now - input->last_rise_ns;
      input->duty_cycle = clampf_local((float)high_time / (float)input->period_ns, 0.0f, 1.0f);
    } else {
      input->duty_cycle = 0.0f;
    }
    input->level_high = false;
  }
}

static float pwm_input_duty(const pwm_input_t *input) {
  if (input->level_high) {
    return input->period_ns > 0 ? 1.0f : 0.0f;
  }
  return clampf_local(input->duty_cycle, 0.0f, 1.0f);
}

static void magnet_tick(void *user_data) {
  magnet_t *magnet = user_data;
  const float dt_s = (float)MAGNET_TICK_US / 1000000.0f;
  const float response_ms = clampf_local((float)attr_read(magnet->response_ms_attr), 1.0f, 1000.0f);
  const float response = dt_s / (response_ms / 1000.0f);
  const float strength = clampf_local((float)attr_read(magnet->magnet_strength_attr) / 10.0f, 0.0f, 2.0f);
  const float target_duty = clampf_local(pwm_input_duty(&magnet->ctrl_pwm) * strength, 0.0f, 1.0f);

  magnet->output_duty += (target_duty - magnet->output_duty) * clampf_local(response, 0.0f, 1.0f);
  magnet->output_duty = clampf_local(magnet->output_duty, 0.0f, 1.0f);

  magnet->pwm_phase_us += MAGNET_TICK_US;
  if (magnet->pwm_phase_us >= OUTPUT_PWM_PERIOD_US) {
    magnet->pwm_phase_us -= OUTPUT_PWM_PERIOD_US;
  }

  const uint32_t high_time_us = (uint32_t)(magnet->output_duty * OUTPUT_PWM_PERIOD_US);
  const bool brake_high = high_time_us > 0 && magnet->pwm_phase_us < high_time_us;
  if (brake_high != magnet->brake_out_level) {
    pin_write(magnet->brake_out_pin, brake_high ? HIGH : LOW);
    magnet->brake_out_level = brake_high;
  }

  pin_write(magnet->status_led_pin, magnet->output_duty > 0.03f ? HIGH : LOW);
}

void chip_init(void) {
  magnet_t *magnet = malloc(sizeof(magnet_t));
  memset(magnet, 0, sizeof(*magnet));

  pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  pwm_input_init(&magnet->ctrl_pwm, "CTRL_IN");
  magnet->ctrl_in_pin = magnet->ctrl_pwm.pin;
  magnet->brake_out_pin = pin_init("BRAKE_OUT", OUTPUT_LOW);
  magnet->status_led_pin = pin_init("STATUS_LED", OUTPUT_LOW);

  magnet->magnet_strength_attr = attr_init("magnetStrength", 10);
  magnet->response_ms_attr = attr_init("responseMs", 60);

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = pwm_input_edge,
    .user_data = &magnet->ctrl_pwm,
  };
  pin_watch(magnet->ctrl_in_pin, &watch_config);

  const timer_config_t timer_config = {
    .callback = magnet_tick,
    .user_data = magnet,
  };
  magnet->timer = timer_init(&timer_config);
  timer_start(magnet->timer, MAGNET_TICK_US, true);

  printf("[Magnet] bereit\n");
}
