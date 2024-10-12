/*
 * Copyright (c) 2024 Branilson Luiz
 */

/**
 * @file
 * @brief Dual motor controller for small robots using DC brushed motors.
 *
 * This controller uses a protocol based on simple ASCII messags exchanged via USB USB CDC ACM class driver.
 * The received data is processed and responses are returned to the serial port.
 * See the runCommand() function for communication details.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sample_usbd.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/cbprintf.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
#include <zephyr/console/console.h>
#include "qdenc.h"
#include "hbridge.h"
#include "Pid.h"

#define PWM_PERIOD_NSEC 50000U	/* Period = 50us ==> Frequency = 20 kHz*/
#define CONTROL_PERIOD_MSEC 20U /* in milliseconds*/
#define HZ (1000U / CONTROL_PERIOD_MSEC)
#define STACK_SIZE 500U
#define PRIORITY 5U
#define SHORT_SLEEP_TIME_MS 50U
#define SLEEP_TIME_MS 1950U
#define MAX_PWM 1000
#define MIN_PWM (-1000)
#define KP 0.012F
#define KI 0.008F
#define KD 0.004F
enum motors
{
	M1 = 0,
	M2
};

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static inline void print_baudrate(const struct device *dev);
void runCommand(char *args);
int init_encoder_irqs();
void blink_led();
void update_motors(struct k_timer *tim);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec enc1a_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(enc1a), gpios, {0});
static const struct gpio_dt_spec enc1b_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(enc1b), gpios, {0});
static const struct gpio_dt_spec enc2a_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(enc2a), gpios, {0});
static const struct gpio_dt_spec enc2b_spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(enc2b), gpios, {0});
static struct qdencoder enc1, enc2;
static struct gpio_callback enc1a_cb, enc1b_cb, enc2a_cb, enc2b_cb;
static const struct gpio_dt_spec m1_forward_spec = GPIO_DT_SPEC_GET(DT_ALIAS(m1fwd), gpios);
static const struct gpio_dt_spec m1_backward_spec = GPIO_DT_SPEC_GET(DT_ALIAS(m1bck), gpios);
static const struct gpio_dt_spec m2_forward_spec = GPIO_DT_SPEC_GET(DT_ALIAS(m2fwd), gpios);
static const struct gpio_dt_spec m2_backward_spec = GPIO_DT_SPEC_GET(DT_ALIAS(m2bck), gpios);
static struct Hbridge motor1, motor2;
static const struct pwm_dt_spec pwm1_dt_spec = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));
static const struct pwm_dt_spec pwm2_dt_spec = PWM_DT_SPEC_GET(DT_ALIAS(pwm2));
const struct device *const ina_dev = DEVICE_DT_GET_ONE(ti_ina226);
struct sensor_value v_bus, power, current;
pidData_t control1, control2;
bool is_control = false, speed_debug = false;
int reverse[] = {-1, 1}; /* -1 to reverse / 1 to direct */
int pwm_values[2], speeds[2], last_counts[2];
struct k_timer control_loop;

K_THREAD_DEFINE(blink_led_thread, STACK_SIZE, blink_led, NULL, NULL, NULL,
				7, 0, 0);
K_TIMER_DEFINE(control_loop, update_motors, NULL);

int main(void)
{
	int ret;

	if (!device_is_ready(uart_dev))
	{
		LOG_ERR("CDC ACM device not ready");
		return 0;
	}

	ret = usb_enable(NULL);

	if (ret != 0)
	{
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	if (!pwm_is_ready_dt(&pwm1_dt_spec))
	{
		printk("Error: PWM device %s is not ready\n",
			   pwm1_dt_spec.dev->name);
	}

	if (!pwm_is_ready_dt(&pwm2_dt_spec))
	{
		printk("Error: PWM device %s is not ready\n",
			   pwm2_dt_spec.dev->name);
	}

	if (hbridge_init(&motor1, &pwm1_dt_spec, &m1_forward_spec, &m1_backward_spec,
					 0, MAX_PWM, PWM_PERIOD_NSEC) < 0)
	{
		LOG_ERR("Failed to enable H Bridge #1");
	}

	if (hbridge_init(&motor2, &pwm2_dt_spec, &m2_forward_spec, &m2_backward_spec,
					 0, MAX_PWM, PWM_PERIOD_NSEC) < 0)
	{
		LOG_ERR("Failed to enable H Bridge #2");
	}

	if (!device_is_ready(ina_dev))
	{
		printf("Device %s is not ready.\n", ina_dev->name);
	}

	init_encoder(&enc1, 1);
	init_encoder(&enc2, 1);
	init_encoder_irqs();
	Pid_Init(&control1, KP, KI, KD, PID_DIRECT, CONTROL_PERIOD_MSEC);
	Pid_Init(&control2, KP, KI, KD, PID_DIRECT, CONTROL_PERIOD_MSEC);
	Pid_SetOutputLimits(&control1, (double)MIN_PWM, (double)MAX_PWM);
	Pid_SetOutputLimits(&control2, (double)MIN_PWM, (double)MAX_PWM);
	k_timer_start(&control_loop, K_MSEC(CONTROL_PERIOD_MSEC), K_MSEC(CONTROL_PERIOD_MSEC));

	while (true)
	{
		uint32_t dtr = 0U;

		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr)
		{
			break;
		}
		else
		{
			k_sleep(K_MSEC(100)); /* Give CPU resources to low priority threads. */
		}
	}

	printk("\n***  BL-Labs Amanita Controller running on %s ***\n\n", CONFIG_BOARD);
	print_baudrate(uart_dev);
	printk("\nEnter a command and press enter or h <enter> for help.\n");

	console_getline_init();
	while (1)
	{
		char *argv = console_getline();
		runCommand(argv);
	}
	return 0;
}

void runCommand(char *args)
{
	char *argv = args;
	bool valid = true;
	int argc = 0;
	char *cmd, *arg1, *arg2, *arg3;
	cmd = arg1 = arg2 = arg3 = NULL;
	char *token;

	while ((token = strtok_r(argv, " ", &argv)))
	{
		if (argc == 0)
		{
			cmd = token;
			argc++;
		}
		else if (cmd[0] == 'o' || cmd[0] == 'm')
		{
			if (argc == 1)
				arg1 = token;
			else if (argc == 2)
				arg2 = token;
			else if (argc > 2)
				break;
			argc++;
		}
		else if (cmd[0] == 'u')
		{
			if (argc == 1)
				arg1 = token;
			else if (argc == 2)
				arg2 = token;
			else if (argc == 3)
				arg3 = token;
			else if (argc > 3)
				break;
			argc++;
		}
		else if (strpbrk(cmd, "hbersgdp"))
			break;
		else
		{
			valid = false;
			break;
		}
	}
	if ((cmd[0] == 'o' || cmd[0] == 'm') && argc < 3)
		valid = false;
	if (cmd[0] == 'u' && argc < 4)
		valid = false;

	if (!valid)
	{
		printk("ERR\n");
		return;
	}

	switch (cmd[0])
	{
	case 'h':
		printk("\t h <enter> to print this help.\n");
		printk("\t b <enter> to get baudrate.\n");
		printk("\t e <enter> to get encoder counts/s.\n");
		printk("\t r <enter> to reset encoder counts.\n");
		printk("\t s <enter> to get motor speeds in ticks/s.\n");
		printk("\t g <enter> to get PID controller gains.\n");
		printk("\t d <enter> to toggle speed debug flag.\n");
		printk("\t p <enter> to get power(W), Voltage(V), and Current(A).\n");
		printk("\t m nn nn <enter> to set control speeds in ticks/s.\n");
		printk("\t o nn nn <enter> to set pwm speeds.\n");
		printk("\t u nn nn nn <enter> to update PID gains.\n");
		printk("\t nn is an integer number.\n");
		break;
	case 'b':
		print_baudrate(uart_dev);
		break;
	case 'e':
		printk("e %lli %lli\n", reverse[M1] * enc1.count, reverse[M2] * enc2.count);
		break;
	case 'r':
		init_encoder(&enc1, 1);
		init_encoder(&enc2, 1);
		printk("e %lli %lli\n", reverse[M1] * enc1.count, reverse[M2] * enc2.count);
		break;
	case 's':
		printk("s %i %i\n", speeds[M1], speeds[M2]);
		break;
	case 'd':
		speed_debug = !speed_debug;
		if (!speed_debug)
			printk("Speed debug disabled\n");
		break;
	case 'g':
		printk("g1 %f %f %f\n", Pid_GetKp(&control1), Pid_GetKi(&control1), Pid_GetKd(&control1));
		printk("g2 %f %f %f\n", Pid_GetKp(&control2), Pid_GetKi(&control2), Pid_GetKd(&control2));
		break;
	case 'm':
		int m1 = atoi(arg1), m2 = atoi(arg2);
		Pid_SetSetPoint(&control1, m1);
		Pid_SetSetPoint(&control2, m2);
		is_control = true;
		printk("m %i %i\n", m1, m2);
		break;
	case 'o':
		int o1 = atoi(arg1), o2 = atoi(arg2);
		pwm_values[M1] = o1;
		pwm_values[M2] = o2;
		printk("o %i %i\n", o1, o2);
		is_control = false;
		break;
	case 'p':
		if (sensor_sample_fetch(ina_dev)) printk("Could not fetch sensor data.\n");
		sensor_channel_get(ina_dev, SENSOR_CHAN_VOLTAGE, &v_bus);
		sensor_channel_get(ina_dev, SENSOR_CHAN_POWER, &power);
		sensor_channel_get(ina_dev, SENSOR_CHAN_CURRENT, &current);
		printf("p %f %f %f\n", sensor_value_to_double(&v_bus),
			   sensor_value_to_double(&power),
			   sensor_value_to_double(&current));
		break;
	case 'u':
		double kp = atof(arg1), ki = atof(arg2), kd = atof(arg3);
		Pid_SetTunings(&control1, kp, ki, kd);
		Pid_SetTunings(&control2, kp, ki, kd);
		printk("u %f %f %f\n", kp, ki, kd);
		break;
	default:
		printk("Invalid Command\r\n"); /* It shall never gets here. */
		break;
	}
}

static inline void print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
	{
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	}
	else
	{
		LOG_INF("Baudrate %u", baudrate);
	}
}

void update_motors(struct k_timer *tim)
{
	speeds[M1] = reverse[M1] * (enc1.count - last_counts[M1]) * HZ; /* speeds in ticks/sec */
	speeds[M2] = reverse[M2] * (enc2.count - last_counts[M2]) * HZ;
	last_counts[M1] = enc1.count;
	last_counts[M2] = enc2.count;
	if (speed_debug)
		printk("s %i %i\n", speeds[M1], speeds[M2]);

	if (is_control)
	{
		Pid_Run(&control1, (float)speeds[M1]);
		Pid_Run(&control2, (float)speeds[M2]);
		run(&motor1, reverse[M1] * (int)control1.output);
		run(&motor2, reverse[M2] * (int)control2.output);
	}
	else
	{
		run(&motor1, reverse[M1] * pwm_values[M1]);
		run(&motor2, reverse[M2] * pwm_values[M2]);
	}
}

void blink_led()
{
	int ret;

	if (!device_is_ready(led.port))
	{
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return;
	}

	bool led_is_on = false;

	while (1)
	{
		ret = gpio_pin_set(led.port, led.pin, (int)led_is_on);
		led_is_on = !led_is_on;
		if (led_is_on)
		{
			k_msleep(SLEEP_TIME_MS);
		}
		else
		{
			k_msleep(SHORT_SLEEP_TIME_MS);
		}
	}
}

void enc1_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t mask)
{
	compute_encoder(&enc1);
}

void enc2_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t mask)
{
	compute_encoder(&enc2);
}

int init_encoder_irqs()
{
	int ret;
	enc1.channel_a = &enc1a_spec;
	enc1.channel_b = &enc1b_spec;
	enc2.channel_a = &enc2a_spec;
	enc2.channel_b = &enc2b_spec;

	ret = gpio_pin_configure_dt(&enc1a_spec, GPIO_INPUT | GPIO_PULL_UP);
	ret += gpio_pin_configure_dt(&enc1b_spec, GPIO_INPUT | GPIO_PULL_UP);
	ret += gpio_pin_configure_dt(&enc2a_spec, GPIO_INPUT | GPIO_PULL_UP);
	ret += gpio_pin_configure_dt(&enc2b_spec, GPIO_INPUT | GPIO_PULL_UP);
	ret += gpio_pin_interrupt_configure_dt(&enc1a_spec, GPIO_INT_EDGE_BOTH);
	ret += gpio_pin_interrupt_configure_dt(&enc1b_spec, GPIO_INT_EDGE_BOTH);
	ret += gpio_pin_interrupt_configure_dt(&enc2a_spec, GPIO_INT_EDGE_BOTH);
	ret += gpio_pin_interrupt_configure_dt(&enc2b_spec, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&enc1a_cb, enc1_callback, BIT(enc1a_spec.pin));
	gpio_init_callback(&enc1b_cb, enc1_callback, BIT(enc1b_spec.pin));
	gpio_init_callback(&enc2a_cb, enc2_callback, BIT(enc2a_spec.pin));
	gpio_init_callback(&enc2b_cb, enc2_callback, BIT(enc2b_spec.pin));
	ret += gpio_add_callback(enc1a_spec.port, &enc1a_cb);
	ret += gpio_add_callback(enc1b_spec.port, &enc1b_cb);
	ret += gpio_add_callback(enc2a_spec.port, &enc2a_cb);
	ret += gpio_add_callback(enc2b_spec.port, &enc2b_cb);
	return ret;
}
