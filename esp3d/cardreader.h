#ifndef CARD_READER_H
#define CARD_READER_H

// #ifdef SDCARD_MMC
#include <SD_MMC.h>
#define SD_DRV SD_MMC
// #if defined(ARDUINO_ARCH_ESP32)
#define SD_CMD    15
#define SD_DATA_0 2
#define SD_DATA_1 4
#define SD_DATA_2 12
#define SD_DATA_3 13
#define SD_CLK    5
// #endif
// #else
// #include <SD.h>
// #define SD_DRV SD
// #endif

extern bool sdcard_ready;
bool initSdCard();

#endif
