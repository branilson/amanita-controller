/*
 * Copyright (c) 2024 Branilson Luiz
 *
 * Library inspired on the L298N Arduino Library from Andrea Lombardo.
 * available at https://github.com/AndreaLombardo/L298N
 *
 */

#include "hbridge.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(amanita_controller);

int hbridge_init(struct Hbridge *motor, const struct pwm_dt_spec *pwm_spec,
                 const struct gpio_dt_spec *pin_forward, const struct gpio_dt_spec *pin_backward, 
                 int pwm_min, int pwm_max, int period)
{
    // if (!pwm_is_ready_dt(pwm_spec))
    // {
    //     LOG_ERR("Error: PWM device %s is not ready\n", pwm_spec.dev->name);
    //     return -1;
    // }
    motor->pwm_spec = pwm_spec;
    motor->max_pwm = pwm_max;
    motor->min_pwm = pwm_min;

    if (!device_is_ready(pin_forward->port))
    {
        LOG_ERR("Error: GPIO device %s is not ready\n", pin_forward->port->name);
        return -2;
    }
    motor->forward_pin = pin_forward;

    if (!device_is_ready(pin_backward->port))
    {
        LOG_ERR("Error: GPIO device %s is not ready\n", pin_backward->port->name);
        return -2;
    }
    motor->backward_pin = pin_backward;

    int ret;
    ret = gpio_pin_configure_dt(motor->forward_pin, GPIO_OUTPUT_INACTIVE);
    ret += gpio_pin_configure_dt(motor->backward_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
    {
        LOG_ERR("Error: GPIO direction device configuration is not possible\n");
        return ret;
    }

    ret = gpio_pin_set(pin_forward->port, pin_forward->pin, 0);
    ret += gpio_pin_set(pin_backward->port, pin_backward->pin, 0);
    if (ret < 0)
    {
        LOG_ERR("Error: GPIO direction definition is not possible\n");
        return ret;
    }

    motor->pwm_period = period;
    stop(motor);
    motor->is_moving = false;
    motor->is_init = true;
    return 0;
}

int run(struct Hbridge *motor, int pwm)
{
    if (motor->is_init && pwm >= 0)
    {
        forward(motor, pwm);
        return 0;
    }
    if (motor->is_init && pwm < 0)
    {
        backward(motor, -pwm);
        return 0;
    }
    else
    {
        LOG_ERR("Error: motor not initiated. Run hbridge_init() before.\n");
        return -3;
    }
}

int forward(struct Hbridge *motor, int pwm)
{
    int ret;
    if (pwm < motor->min_pwm || pwm > motor->max_pwm)
    {
        LOG_ERR("Error: pwm value must be within %d and %d\n", motor->min_pwm, motor->max_pwm);
        return -4;
    }
    else
    {
        motor->dir = FORWARD;
        motor->is_moving = true;
        pwm_set_dt(motor->pwm_spec, motor->pwm_period, pwm * motor->pwm_period / 1000);
        ret = gpio_pin_set(motor->backward_pin->port, motor->backward_pin->pin, 0);
        ret += gpio_pin_set(motor->forward_pin->port, motor->forward_pin->pin, 1);
        if (ret < 0)
        {
            LOG_ERR("Error: GPIO direction definition is not possible\n");
        }
    }
    return ret;
}
int backward(struct Hbridge *motor, int pwm)
{
    int ret;
    if (pwm < motor->min_pwm || pwm > motor->max_pwm)
    {
        LOG_ERR("Error: pwm value must be within %d and %d\n", motor->min_pwm, motor->max_pwm);
        return -4;
    }
    else
    {
        motor->dir = BACKWARD;
        motor->is_moving = true;
        pwm_set_dt(motor->pwm_spec, motor->pwm_period, pwm * motor->pwm_period / 1000);
        ret = gpio_pin_set(motor->forward_pin->port, motor->forward_pin->pin, 0);
        ret += gpio_pin_set(motor->backward_pin->port, motor->backward_pin->pin, 1);
        if (ret < 0)
        {
            LOG_ERR("Error: GPIO direction definition is not possible\n");
        }
    }
    return ret;
}
void stop(struct Hbridge *motor)
{
    int ret;
    motor->dir = STOP;
    motor->is_moving = false;
    pwm_set_dt(motor->pwm_spec, motor->pwm_period, 0);
    ret = gpio_pin_set(motor->forward_pin->port, motor->forward_pin->pin, 0);
    ret += gpio_pin_set(motor->backward_pin->port, motor->backward_pin->pin, 0);
    if (ret < 0)
    {
        LOG_ERR("Error: GPIO direction definition is not possible\n");
    }
}
