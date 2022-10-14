#ifndef CARD_READER_H
#define CARD_READER_H

#include <Arduino.h>
#include "config.h"

#ifdef SDCARD_MMC
#include "sdmmc.h"
#define SD_DRV SD_MMC_EX
#if defined(ARDUINO_ARCH_ESP32)
#define SD_CMD    15
#define SD_DATA_0 2
#define SD_DATA_1 4
#define SD_DATA_2 12
#define SD_DATA_3 13
#define SD_CLK    5
#endif
#else
#include <SD.h>
#define SD_DRV SD
#endif

extern bool sdcard_ready;
bool initSdCard();
bool sd_read_block(void * dst, uint32_t blknum);

#endif
