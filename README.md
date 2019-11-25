# P5-DSP-Board-mdk
C Firmware for FHNW P5 DSP Board based on STM32F412 / TLV320aic

---

### SSD1306 OLED

[mnemocron/stm32-ssd1306](https://github.com/mnemocron/stm32-ssd1306)

```c
SSD1306_t holed1;

holed1.hi2cx = &hi2c1;

ssd1306_Init(&holed1);
ssd1306_Fill(&holed1, Black);
ssd1306_SetCursor(&holed1, 2, 0);
ssd1306_WriteString(&holed1, "Hello,", Font_11x18, White);
ssd1306_UpdateScreen(&holed1);

```

---

### License

Copyright (c) 2019 Simon Burkhardt, Mischa Studer

No license applies! You may not reuse the provided code in any form wthout permission.




