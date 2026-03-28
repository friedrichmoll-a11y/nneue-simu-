#include "wokwi-api.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  pin_t pwm_in_pin;
  pin_t pwm_out_pin;
  pin_t active_pin;
  uint32_t duty_attr;
  timer_t timer;
  uint64_t last_rise_ns;
  uint32_t period_ns;
  float duty_cycle;
  uint32_t phase_ns;
  bool output_high;
} pwm_driver_t;

static const uint32_t DRIVER_TICK_US = 50;
static const uint32_t DEFAULT_PERIOD_NS = 1000000;

static float clampf_local(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void pwm_driver_edge(void *user_data, pin_t pin, uint32_t value) {
  (void)pin;
  pwm_driver_t *driver = user_data;
  uint64_t now = get_sim_nanos();

  if (value == HIGH) {
    if (driver->last_rise_ns != 0) {
      uint64_t period = now - driver->last_rise_ns;
      if (period > 0 && period < 200000000ULL) {
        driver->period_ns = (uint32_t)period;
      }
    }
    driver->last_rise_ns = now;
  } else if (driver->last_rise_ns != 0 && driver->period_ns > 0) {
    uint64_t high_time = now - driver->last_rise_ns;
    driver->duty_cycle = clampf_local((float)high_time / (float)driver->period_ns, 0.0f, 1.0f);
  }
}

static void pwm_driver_tick(void *user_data) {
  pwm_driver_t *driver = user_data;
  uint32_t period_ns = driver->period_ns;
  if (period_ns == 0 || period_ns > 200000000U) {
    period_ns = DEFAULT_PERIOD_NS;
  }

  driver->phase_ns += DRIVER_TICK_US * 1000;
  if (driver->phase_ns >= period_ns) {
    driver->phase_ns %= period_ns;
  }

  const uint32_t high_time_ns = (uint32_t)(driver->duty_cycle * period_ns);
  const bool high = driver->duty_cycle > 0.01f && driver->phase_ns < high_time_ns;
  if (high != driver->output_high) {
    pin_write(driver->pwm_out_pin, high ? HIGH : LOW);
    pin_write(driver->active_pin, high ? HIGH : LOW);
    driver->output_high = high;
  }
}

void chip_init(void) {
  static pwm_driver_t driver;
  memset(&driver, 0, sizeof(driver));

  pin_init("VCC", INPUT);
  pin_init("GND", INPUT);
  pin_init("V24", INPUT);
  driver.pwm_in_pin = pin_init("PWM_IN", INPUT);
  driver.pwm_out_pin = pin_init("PWM_24V", OUTPUT_LOW);
  driver.active_pin = pin_init("ACTIVE", OUTPUT_LOW);
  driver.duty_attr = attr_init("duty_display", 0);
  driver.period_ns = DEFAULT_PERIOD_NS;

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = pwm_driver_edge,
    .user_data = &driver,
  };
  pin_watch(driver.pwm_in_pin, &watch_config);

  const timer_config_t timer_config = {
    .callback = pwm_driver_tick,
    .user_data = &driver,
  };
  driver.timer = timer_init(&timer_config);
  timer_start(driver.timer, DRIVER_TICK_US, true);

  printf("[PWM-Driver] bereit\n");
}
