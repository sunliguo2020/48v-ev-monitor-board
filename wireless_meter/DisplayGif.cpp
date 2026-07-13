#include "DisplayGif.h"
#include "Gif.h"

/**
 * @brief 显示GIF单帧
 *
 * @param i 当前帧编号
 */
void show_gif(int i)
{
    uint8_t frame[512];

    memcpy_P(
        frame,
        gif[i],
        512);

    u8g2.setDrawColor(1);

    u8g2.drawBox(0, 0, 128, 64);

    u8g2.setDrawColor(0);

    u8g2.drawXBM(
        32,
        0,
        64,
        64,
        frame);

    u8g2.setDrawColor(1);

    u8g2.sendBuffer();
}

/**
 * @brief GIF动画播放函数
 *
 * 顺序播放33帧动画
 */
void displaygif()
{

    for (int i = 0; i < gif_length; i++)
    {

        show_gif(i);

        // 动画速度
        delay(10);
    }
}