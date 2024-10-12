/*
 * Copyright (c) 2024 Branilson Luiz
 *
 */

#include "qdenc.h"
#include <zephyr/kernel.h>

void compute_encoder(struct qdencoder *enc)
{
  static const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};
  enc->old_state = enc->state;
  enc->last_time = enc->this_time;
  enc->this_time = k_uptime_get();
  enc->state = 2 * gpio_pin_get(enc->channel_b->port, enc->channel_b->pin) +
                   gpio_pin_get(enc->channel_a->port, enc->channel_a->pin);
  enc->direction = QEM[4 * enc->old_state + enc->state];

  if (enc->direction == 1 || enc->direction == -1) 
  {
    enc->count = enc->count + enc->direction;
  }
}

void init_encoder(struct qdencoder *enc, int64_t displacement)
{
    enc->tick_displacement = displacement;
    enc->count = 0;
}

int64_t get_count(struct qdencoder *enc)
{
    return enc->count;
}

int64_t get_displacement(struct qdencoder *enc)
{
    return (enc->count * enc->tick_displacement);
}
