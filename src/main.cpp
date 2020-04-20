/*--------------------------------------------------------------------
Copyright 2020 fukuen

Mic rec/play demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <Sipeed_ST7789.h>
#include <SD.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sysctl.h"
#include "plic.h"
#include "uarths.h"
#include "i2s.h"
#include "fpioa.h"
#include "utility/Button.h"
#include "gpio.h"

#define WIDTH 320
#define HEIGHT 240
#define FRAME_LEN 512
#define SAMPLING 16000
#define FILE_NAME "/RECDATA.WAV"
uint16_t rx_buf[FRAME_LEN * 2];
uint16_t tx_buf[FRAME_LEN];
uint32_t g_rx_dma_buf[FRAME_LEN * 2 * 2];
uint32_t g_tx_dma_buf[FRAME_LEN * 2 * 2];
uint16_t g_tx_dma_buf2[FRAME_LEN * 2 * 2];

volatile uint32_t g_index;
volatile uint8_t uart_rec_flag;
volatile uint32_t receive_char;
volatile uint8_t i2s_rec_flag;
volatile uint8_t i2s_start_flag = 0;

SPIClass spi_0(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_0);

#define DEBOUNCE_MS 10
#ifdef PIN_KEY_UP // Maix-go
#define WIFI_EN_PIN 8
//#define AUDIO_PA_EN_PIN None  // Bit Dock and old MaixGo
#define AUDIO_PA_EN_PIN 32    // Maix Go(version 2.20)
Button BtnBOOT = Button(KEY0, true, DEBOUNCE_MS);
Button BtnA = Button(PIN_KEY_UP, true, DEBOUNCE_MS);
Button BtnB = Button(PIN_KEY_PRESS, true, DEBOUNCE_MS);
Button BtnC = Button(PIN_KEY_DOWN, true, DEBOUNCE_MS);
#else // Maixduino
#define WIFI_EN_PIN 8
#define AUDIO_PA_EN_PIN 2
Button BtnBOOT = Button(KEY0, true, DEBOUNCE_MS);
Button BtnA = Button(7, true, DEBOUNCE_MS);
Button BtnB = Button(8, true, DEBOUNCE_MS);
Button BtnC = Button(9, true, DEBOUNCE_MS);
#endif

typedef enum _rec_play_mode
{
    MODE_PLAY = 0,
    MODE_REC = 1,
    MODE_STOP = 2,
    MODE_PLAYING = 3,
    MODE_RECORDING = 4
} rec_play_mode_t;

rec_play_mode_t rec_play_mode = MODE_STOP;

File file;

int i2s_dma_irq(void *ctx) {
    uint32_t i;
    if (i2s_start_flag) {
        int16_t s_tmp;
        if (g_index) {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[g_index], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = 0;
            for (i = 0; i < FRAME_LEN; i++) {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff); //g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 1;
        } else {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = FRAME_LEN * 2;
            for (i = FRAME_LEN; i < FRAME_LEN * 2; i++) {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff);//g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 2;
        }
    } else {
        i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
        g_index = FRAME_LEN * 2;
    }
    return 0;
}

int pio_init() {
    // disable WiFi
    fpioa_set_function(WIFI_EN_PIN, FUNC_GPIO0);
    gpio_set_drive_mode(0, GPIO_DM_OUTPUT);
    gpio_set_pin(0, GPIO_PV_LOW);

    // disable audio PA
    fpioa_set_function(AUDIO_PA_EN_PIN, FUNC_GPIO1);
    gpio_set_drive_mode(1, GPIO_DM_OUTPUT);
    gpio_set_pin(1, GPIO_PV_LOW);

    fpioa_set_function(MIC_DAT3, FUNC_I2S0_IN_D0);
    fpioa_set_function(MIC_WS, FUNC_I2S0_WS);
    fpioa_set_function(MIC_BCK, FUNC_I2S0_SCLK);

    fpioa_set_function(I2S_DA, FUNC_I2S2_OUT_D1);
    fpioa_set_function(I2S_WS, FUNC_I2S2_WS);	
    fpioa_set_function(I2S_BCK, FUNC_I2S2_SCLK);

    //i2s init
    i2s_init(I2S_DEVICE_0, I2S_RECEIVER, 0x03);
    i2s_init(I2S_DEVICE_2, I2S_TRANSMITTER, 0xC);

    i2s_rx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
            RESOLUTION_16_BIT, SCLK_CYCLES_32,
            TRIGGER_LEVEL_4, STANDARD_MODE);
    i2s_tx_channel_config(I2S_DEVICE_2, I2S_CHANNEL_1,
            RESOLUTION_16_BIT, SCLK_CYCLES_32,
            TRIGGER_LEVEL_4, RIGHT_JUSTIFYING_MODE);

    uint32_t ret = i2s_set_sample_rate(I2S_DEVICE_0, SAMPLING);
    printf("actual rate %ul\n", ret);
    i2s_set_sample_rate(I2S_DEVICE_2, SAMPLING);

//    plic_init();
    dmac_init();
    dmac_set_irq(DMAC_CHANNEL0, i2s_dma_irq, NULL, 1);
    i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);

    /* Enable the machine interrupt */
    sysctl_enable_irq();
    return 0;
}

void drawMenu() {
    lcd.fillScreen(COLOR_BLACK);
    lcd.fillRect(0, 0, 320, 20, COLOR_CYAN);
    lcd.setCursor(1, 1);
    lcd.setTextSize(2);
    lcd.setTextColor(COLOR_BLACK);
    lcd.println("REC/PLAY demo v0.1");
    lcd.setCursor(0, 16);
    lcd.println("");
    lcd.setTextColor(COLOR_WHITE);
    for (int i = 0; i < 10; i++) {
        lcd.println("");
    }
    lcd.println("UP: PLAY/STOP");
    lcd.println("DOWN: REC/STOP");
    lcd.setTextColor(COLOR_GREENYELLOW);
    lcd.print("Copyright 2020 fukuen");
}

void drawClear() {
    lcd.fillRect(0, 32, 320, 144, COLOR_BLACK);
    lcd.setCursor(120, 96);
    lcd.setTextSize(2);
    lcd.setTextColor(COLOR_WHITE);
}

void drawStop() {
    drawClear();
    lcd.println("STOP");
}

void drawPlaying() {
    drawClear();
    lcd.println("PLAYING...");
}

void drawRecording() {
    drawClear();
    lcd.println("RECORDING...");
}

void writeSD(int offset) {
    file.write((uint8_t *)&rx_buf[offset], FRAME_LEN * 2);
}

void setup() {
    pll_init();
    plic_init();
//    dmac_init();
    uarths_init();

    lcd.begin(15000000, COLOR_BLACK);
//    Serial.begin(115200);   // initialize serial for debugging

    if (!SD.begin()){
        printf( "SD mount failed.\n" );
        lcd.println("SD mount failed.");
        while (true) {}
    }

    pio_init();

    g_index = 0;
    i2s_rec_flag = 0;
    i2s_start_flag = 1;

    drawMenu();
}

void loop() {
    BtnBOOT.read();
    BtnA.read();
    BtnB.read();
    BtnC.read();

    if (BtnBOOT.wasPressed()) {
        printf("Button BOOT pressed.\n");
    }

    if (BtnA.wasPressed()) {
        printf("Button A pressed.\n");
        if (rec_play_mode == MODE_STOP) {
            // stop -> playing
            file = SD.open(FILE_NAME);
            if (file) {
                printf("File open success.\n");
                rec_play_mode = MODE_PLAYING;
                drawPlaying();
                // enable audio PA
                gpio_set_pin(1, GPIO_PV_HIGH);
            } else {
                printf("File open failed.\n");
            }
        } else if (rec_play_mode == MODE_PLAYING) {
            // playing -> stop
            file.close();
            rec_play_mode = MODE_STOP;
            drawStop();
            // disable audio PA
            gpio_set_pin(1, GPIO_PV_LOW);
        }
    }

    if (BtnC.wasPressed()) {
        printf("Button C pressed.\n");
        if (rec_play_mode == MODE_STOP) {
            // stop -> recording
            try {
                SD.remove(FILE_NAME);
            } catch (char *str) {
                printf("%s\n", str);
            }
            file = SD.open(FILE_NAME, FILE_WRITE);
            if (file) {
                printf("File open success.\n");
                drawRecording();
                rec_play_mode = MODE_RECORDING;
            } else {
                printf("File open failed.\n");
            }
        } else if (rec_play_mode == MODE_RECORDING) {
            // recording -> stop
            file.close();
            rec_play_mode = MODE_STOP;
            drawStop();
        }
    }

    if (rec_play_mode == MODE_RECORDING) {
        if (i2s_rec_flag == 0) {
            //
        } else if (i2s_rec_flag == 1) {
            writeSD(0);
            i2s_rec_flag = 0;
        } else {
            writeSD(FRAME_LEN);
            i2s_rec_flag = 0;
        }
    } else if (rec_play_mode == MODE_PLAYING) {
        size_t len = file.readBytes((uint8_t *)tx_buf, FRAME_LEN * 2);
        if (len > 0) {
/*
            for (int i = 0; i < len /  2; i++) {
                int16_t tmp = (tx_buf[i] - 32768) * 16;
                g_tx_dma_buf[2 * i] = (uint32_t) tmp;
                g_tx_dma_buf[2 * i + 1] = (uint32_t) tmp;
            }
            dmac_wait_done(DMAC_CHANNEL1);
            i2s_send_data_dma(I2S_DEVICE_2, (uint8_t *)g_tx_dma_buf, len, DMAC_CHANNEL1);
*/
            for (int i = 0; i < len /  2; i++) {
                int16_t tmp = (tx_buf[i] - 32768) * 16;
                g_tx_dma_buf2[2 * i] = (uint16_t) tmp;
                g_tx_dma_buf2[2 * i + 1] = (uint16_t) tmp;
            }
            dmac_wait_done(DMAC_CHANNEL1);
            i2s_play(I2S_DEVICE_2, DMAC_CHANNEL1, (uint8_t *)g_tx_dma_buf2, len * 2,
                    FRAME_LEN, 16, 2);
        } else {
            file.close();
            rec_play_mode = MODE_STOP;
            drawStop();
            // disable audio PA
            gpio_set_pin(1, GPIO_PV_LOW);
        }
    }
}
