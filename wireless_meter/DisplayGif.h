#ifndef DISPLAYGIF_H
#define DISPLAYGIF_H

#include <Arduino.h>
#include <U8g2lib.h>

/*
 * 引用主程序中的OLED对象
 *
 * 实际定义：
 * U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(...)
 *
 * 在 wireless_meter.ino 中
 */
extern U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2;

/*
 * GIF图片数组
 *
 * GIF.h中定义：
 *
 * static const unsigned char gif[][512]
 *
 * 每一帧：
 * 64 × 64 像素
 *
 * 64×64 / 8 = 512 字节
 */
extern const unsigned char gif[][512];

extern const int gif_length;

// 显示一帧GIF
void show_gif(int i);

// 播放完整GIF动画
void displaygif();

#endif