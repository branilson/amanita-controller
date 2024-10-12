/*
 * Copyright (c) 2024 Branilson Luiz
 *
 * Inspired on the L298N Arduino Library from Andrea Lombardo.
 * available at https://github.com/AndreaLombardo/L298N
 *
 */

#ifndef __HBRIDGE__H
#define __HBRIDGE__H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>

enum Direction
{
  STOP = -1,
  FORWARD = 0,
  BACKWARD = 1
};

struct Hbridge
{
  const struct pwm_dt_spec *pwm_spec;
  const struct gpio_dt_spec *forward_pin;
  const struct gpio_dt_spec *backward_pin;
  bool is_moving;
  bool is_init;
  enum Direction dir;
  int pwm_period;
  int min_pwm;
  int max_pwm;
};

int hbridge_init(struct Hbridge *motor, const struct pwm_dt_spec *pwm_spec,
                 const struct gpio_dt_spec *pin_forward, const struct gpio_dt_spec *pin_backward, 
                 int pwm_min, int pwm_max, int period);
int run(struct Hbridge *motor, int pwm);
int forward(struct Hbridge *motor, int pwm);
int backward(struct Hbridge *motor, int pwm);
void stop(struct Hbridge *motor);

#endif /* __HBRIDGE__H */