#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "monitor.h"

static uint8_t temperature_cpy;

bool heaton_guard(void) {
  return ((((float)(temperature_cpy)) * (((float)(150.0)) / ((float)(255.0)))) - ((float)(50.0))) < ((float)(18.0));
}

float heaton_arg0(void) {
  return (((float)(temperature_cpy)) * (((float)(150.0)) / ((float)(255.0)))) - ((float)(50.0));
}

bool heatoff_guard(void) {
  return ((((float)(temperature_cpy)) * (((float)(150.0)) / ((float)(255.0)))) - ((float)(50.0))) > ((float)(21.0));
}

float heatoff_arg0(void) {
  return (((float)(temperature_cpy)) * (((float)(150.0)) / ((float)(255.0)))) - ((float)(50.0));
}

void step(void) {
  (temperature_cpy) = (temperature);
  if ((heaton_guard)()) {
    {(func_on)((()));}
  };
  if ((heatoff_guard)()) {
    {(func_off)((()));}
  };
}
