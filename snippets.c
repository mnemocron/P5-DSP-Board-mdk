int enc_val_1 = (65535 - (TIM3->CNT)) /2;  // change rotation direction: CW/CCW - count up/down
int enc_val_2 = (        (TIM4->CNT)) /2;

HAL_Delay(1000);

// printf("ENC: 1:%d \t 2:%d\n", enc_val_1, enc_val_2);

char lcd_buf[18];
sprintf(lcd_buf, "  %d    ", enc_val_1);
ssd1306_SetCursor(&holedR, 10, 40);
ssd1306_WriteString(&holedR, lcd_buf, Font_11x18, White);
ssd1306_UpdateScreen(&holedR);

sprintf(lcd_buf, "   %d    ", enc_val_2);
ssd1306_SetCursor(&holedL, 10, 40);
ssd1306_WriteString(&holedL, lcd_buf, Font_11x18, White);
ssd1306_UpdateScreen(&holedL);






