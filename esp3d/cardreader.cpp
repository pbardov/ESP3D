#include "cardreader.h"
#include "SdInfo.h"

#ifdef SDCARD_MMC
#include <esp_system.h>
#include <sdmmc_cmd.h>
#endif

bool sdcard_ready = false;

bool initSdCard() {
#if defined(SDCARD_MMC)
#ifdef SD_CMD
    pinMode(SD_CMD, INPUT_PULLUP);
#endif
#ifdef SD_DATA_0
    pinMode(SD_DATA_0, INPUT_PULLUP);
#endif
#ifdef SD_DATA_1
    pinMode(SD_DATA_1, INPUT_PULLUP);
#endif
#ifdef SD_DATA_2
    pinMode(SD_DATA_2, INPUT_PULLUP);
#endif
#ifdef SD_DATA_3
    pinMode(SD_DATA_3, INPUT_PULLUP);
#endif
    // sdcard_ready = SD_DRV.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT, 5); // newest library version
    sdcard_ready = SD_DRV.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT);
#else
    sdcard_ready = SD_DRV.begin(); // NOT TESTED YET
#endif
    if (sdcard_ready) {
        uint8_t card_type = SD_DRV.cardType();
        LOG ("Direct SD type:");
        switch (card_type) {
            case CARD_SD:
            case CARD_SDHC:
            case CARD_MMC:
            LOG (CONFIG::intTostr(card_type));
            break;
            default:
            LOG ("None/Unknown");
            sdcard_ready = false;
            break;
        }
        LOG ("\n");
        if (sdcard_ready) {
            uint64_t sdcard_size = SD_DRV.cardSize();
            LOG ("Direct SD size:");
            LOG (CONFIG::intTostr(sdcard_size));
            if (sdcard_size <= 0) {
                sdcard_ready = false;
            }
            LOG ("\n");
            LOG ("Sector size: ");
            LOG (CONFIG::intTostr(SD_DRV._card->csd.sector_size));
            LOG ("\n");
        }
    } else {
        LOG ("Direct SD card init fail!\n");
    }
    CONFIG::is_direct_sd = sdcard_ready;
    return sdcard_ready;
}

#ifdef SDCARD_MMC
bool sd_read_block(void * dst, uint32_t blknum) {
    esp_err_t res = sdmmc_read_sectors(SD_DRV._card, dst, blknum, 1);
    return res == ESP_OK;
}
#else
bool sd_read_block(void * dst, uint32_t blknum) {
    esp_err_t res = SD_DRV.readRAW(dst, blknum);
    return res == ESP_OK;
}
#endif
