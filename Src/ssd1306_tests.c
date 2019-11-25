#include "ssd1306.h"
#include <string.h>
#include <stdio.h>

void ssd1306_TestBorder(SSD1306_t* ssd) {
    ssd1306_Fill(ssd, Black);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    uint8_t x = 0;
    uint8_t y = 0;
    do {
        ssd1306_DrawPixel(ssd, x, y, Black);

        if((y == 0) && (x < 127))
            x++;
        else if((x == 127) && (y < 63))
            y++;
        else if((y == 63) && (x > 0)) 
            x--;
        else
            y--;

        ssd1306_DrawPixel(ssd, x, y, White);
        ssd1306_UpdateScreen(ssd);
    
        HAL_Delay(5);
        end = HAL_GetTick();
    } while((end - start) < 8000);
   
    HAL_Delay(1000);
}

void ssd1306_TestFonts(SSD1306_t* ssd) {
    ssd1306_Fill(ssd, Black);
    ssd1306_SetCursor(ssd, 2, 0);
    ssd1306_WriteString(ssd, "Font 16x26", Font_16x26, White);
    ssd1306_SetCursor(ssd, 2, 26);
    ssd1306_WriteString(ssd, "Font 11x18", Font_11x18, White);
    ssd1306_SetCursor(ssd, 2, 26+18);
    ssd1306_WriteString(ssd, "Font 7x10", Font_7x10, White);
    ssd1306_UpdateScreen(ssd);
}

void ssd1306_TestFPS(SSD1306_t* ssd) {
    ssd1306_Fill(ssd, White);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    int fps = 0;
    char message[] = "ABCDEFGHIJK";
   
    ssd1306_SetCursor(ssd, 2,0);
    ssd1306_WriteString(ssd, "Testing...", Font_11x18, Black);
   
    do {
        ssd1306_SetCursor(ssd, 2, 18);
        ssd1306_WriteString(ssd, message, Font_11x18, Black);
        ssd1306_UpdateScreen(ssd);
       
        char ch = message[0];
        memmove(message, message+1, sizeof(message)-2);
        message[sizeof(message)-2] = ch;

        fps++;
        end = HAL_GetTick();
    } while((end - start) < 5000);
   
    HAL_Delay(1000);

    char buff[64];
    fps = (float)fps / ((end - start) / 1000.0);
    snprintf(buff, sizeof(buff), "~%d FPS", fps);
   
    ssd1306_Fill(ssd, White);
    ssd1306_SetCursor(ssd, 2, 18);
    ssd1306_WriteString(ssd, buff, Font_11x18, Black);
    ssd1306_UpdateScreen(ssd);
}

void ssd1306_TestAll(SSD1306_t* ssd) {
    ssd1306_Init(ssd);
    ssd1306_TestFPS(ssd);
    HAL_Delay(3000);
    ssd1306_TestBorder(ssd);
    ssd1306_TestFonts(ssd);
}
