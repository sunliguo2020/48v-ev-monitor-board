#ifndef GIF_H
#define GIF_H

#include <Arduino.h>
#include <pgmspace.h>

#define GIF_LENGTH 33

extern const uint8_t gif[GIF_LENGTH][512] PROGMEM;

extern const int gif_length;

#endif