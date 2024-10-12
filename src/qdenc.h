/*
 * Copyright (c) 2024 Branilson Luiz
 * 
 * Library inspired in the tutorial "How to use a quadrature encoder" from sparkfun.com.
 * 
 */

#ifndef __QDENC__H
#define __QDENC__H

#include <zephyr/drivers/gpio.h>
#include <stdint.h>

struct qdencoder
{
  const struct gpio_dt_spec *channel_a;
  const struct gpio_dt_spec *channel_b;
  volatile int state;
  volatile int old_state;
  volatile int64_t last_time;
  volatile int64_t this_time;
  int64_t tick_displacement;
  volatile int direction;
  volatile int64_t count;
};

void compute_encoder(struct qdencoder *enc);
void init_encoder(struct qdencoder *enc, int64_t displacement);
int64_t get_displacement(struct qdencoder *enc);
int64_t get_count(struct qdencoder *enc);

#endif /* __QDENC__H */