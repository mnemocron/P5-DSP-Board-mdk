#include "ssd1306.h"

#if defined(SSD1306_USE_I2C)

extern SSD1306_t holed1;

void ssd1306_Reset(SSD1306_t *ssd) {
	/* for I2C - do nothing */
}

// Send a byte to the command register
void ssd1306_WriteCommand(SSD1306_t *ssd, uint8_t byte) {
	HAL_I2C_Mem_Write(ssd->hi2cx, ssd->address, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// Send data
void ssd1306_WriteData(SSD1306_t *ssd, uint8_t* buffer, size_t buff_size) {
	HAL_I2C_Mem_Write(ssd->hi2cx, ssd->address, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

#elif defined(SSD1306_USE_SPI)

void ssd1306_Reset(SSD1306_t *ssd) {
	// CS = High (not selected)
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET);

	// Reset the OLED
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

// Send a byte to the command register
void ssd1306_WriteCommand(ssd, SSD1306_t *ssd, uint8_t byte) {
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_RESET); // command
    HAL_SPI_Transmit(&SSD1306_SPI_PORT, (uint8_t *) &byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
}

// Send data
void ssd1306_WriteData(SSD1306_t *ssd, uint8_t* buffer, size_t buff_size) {
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_SET); // data
    HAL_SPI_Transmit(&SSD1306_SPI_PORT, buffer, buff_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
}

#else
#error "You should define SSD1306_USE_SPI or SSD1306_USE_I2C macro"
#endif


// Screenbuffer
// static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Screen object
static SSD1306_t SSD1306;

// Initialize the oled screen
void ssd1306_Init(SSD1306_t *ssd) {
    // Reset OLED
    ssd1306_Reset(ssd);
	  ssd->address = SSD1306_I2C_ADDR;

    // Wait for the screen to boot
    HAL_Delay(100);
    
    // Init OLED
    ssd1306_WriteCommand(ssd, 0xAE); //display off

    ssd1306_WriteCommand(ssd, 0x20); //Set Memory Addressing Mode   
    ssd1306_WriteCommand(ssd, 0x10); // 00,Horizontal Addressing Mode; 01,Vertical Addressing Mode;
                                // 10,Page Addressing Mode (RESET); 11,Invalid

    ssd1306_WriteCommand(ssd, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
    ssd1306_WriteCommand(ssd, 0xC0); // Mirror vertically
#else
    ssd1306_WriteCommand(ssd, 0xC8); //Set COM Output Scan Direction
#endif

    ssd1306_WriteCommand(ssd, 0x00); //---set low column address
    ssd1306_WriteCommand(ssd, 0x10); //---set high column address

    ssd1306_WriteCommand(ssd, 0x40); //--set start line address - CHECK

    ssd1306_WriteCommand(ssd, 0x81); //--set contrast control register - CHECK
    ssd1306_WriteCommand(ssd, 0xFF);

#ifdef SSD1306_MIRROR_HORIZ
    ssd1306_WriteCommand(ssd, 0xA0); // Mirror horizontally
#else
    ssd1306_WriteCommand(ssd, 0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
    ssd1306_WriteCommand(ssd, 0xA7); //--set inverse color
#else
    ssd1306_WriteCommand(ssd, 0xA6); //--set normal color
#endif

    ssd1306_WriteCommand(ssd, 0xA8); //--set multiplex ratio(1 to 64) - CHECK
    ssd1306_WriteCommand(ssd, 0x3F); //

    ssd1306_WriteCommand(ssd, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssd1306_WriteCommand(ssd, 0xD3); //-set display offset - CHECK
    ssd1306_WriteCommand(ssd, 0x00); //-not offset

    ssd1306_WriteCommand(ssd, 0xD5); //--set display clock divide ratio/oscillator frequency
    ssd1306_WriteCommand(ssd, 0xF0); //--set divide ratio

    ssd1306_WriteCommand(ssd, 0xD9); //--set pre-charge period
    ssd1306_WriteCommand(ssd, 0x22); //

    ssd1306_WriteCommand(ssd, 0xDA); //--set com pins hardware configuration - CHECK
    ssd1306_WriteCommand(ssd, 0x12);

    ssd1306_WriteCommand(ssd, 0xDB); //--set vcomh
    ssd1306_WriteCommand(ssd, 0x20); //0x20,0.77xVcc

    ssd1306_WriteCommand(ssd, 0x8D); //--set DC-DC enable
    ssd1306_WriteCommand(ssd, 0x14); //
    ssd1306_WriteCommand(ssd, 0xAF); //--turn on SSD1306 panel

    // Clear screen
    ssd1306_Fill(ssd, Black);
    
    // Flush buffer to screen
    ssd1306_UpdateScreen(ssd);
    
    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;
    
    SSD1306.Initialized = 1;
}

// Fill the whole screen with the given color
void ssd1306_Fill(SSD1306_t *ssd, SSD1306_COLOR color) {
    /* Set memory */
    uint32_t i;

    for(i = 0; i < sizeof(ssd->pixels); i++) {
        ssd->pixels[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

// Write the screenbuffer with changed to the screen
void ssd1306_UpdateScreen(SSD1306_t *ssd) {
    uint8_t i;
    for(i = 0; i < 8; i++) {
        ssd1306_WriteCommand(ssd, 0xB0 + i);
        ssd1306_WriteCommand(ssd, 0x00);
        ssd1306_WriteCommand(ssd, 0x10);
        ssd1306_WriteData(ssd, &ssd->pixels[SSD1306_WIDTH*i],SSD1306_WIDTH);
    }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void ssd1306_DrawPixel(SSD1306_t *ssd, uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        // Don't write outside the buffer
        return;
    }
    
    // Check if pixel should be inverted
    if(SSD1306.Inverted) {
        color = (SSD1306_COLOR)!color;
    }
    
    // Draw in the right color
    if(color == White) {
        ssd->pixels[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else { 
        ssd->pixels[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

// Draw 1 char to the screen buffer
// ch         => char om weg te schrijven
// Font     => Font waarmee we gaan schrijven
// color     => Black or White
char ssd1306_WriteChar(SSD1306_t *ssd, char ch, FontDef Font, SSD1306_COLOR color) {
    uint32_t i, b, j;
    
    // Check remaining space on current line
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }
    
    // Use the font to write
    for(i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for(j = 0; j < Font.FontWidth; j++) {
            if((b << j) & 0x8000)  {
                ssd1306_DrawPixel(ssd, SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
            } else {
                ssd1306_DrawPixel(ssd, SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }
    
    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;
    
    // Return written char for validation
    return ch;
}

// Write full string to screenbuffer
char ssd1306_WriteString(SSD1306_t *ssd, char* str, FontDef Font, SSD1306_COLOR color) {
    // Write until null-byte
    while (*str) {
        if (ssd1306_WriteChar(ssd, *str, Font, color) != *str) {
            // Char could not be written
            return *str;
        }
        
        // Next char
        str++;
    }
    
    // Everything ok
    return *str;
}

// Position the cursor
void ssd1306_SetCursor(SSD1306_t *ssd, uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
