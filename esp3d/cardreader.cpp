#include "cardreader.h"
#include "config.h"

bool sdcard_ready = false;

bool initSdCard() {
    sdcard_ready = SD_DRV.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT, 5);
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
            uint64_t card_size = SD_DRV.cardSize();
            LOG ("Direct SD size:");
            LOG (CONFIG::intTostr(card_size));
            if (card_size <= 0) {
                sdcard_ready = false;
            }
            LOG ("\n");
        }
    } else {
        LOG ("Direct SD card init fail!\n");
    }
    CONFIG::is_direct_sd = sdcard_ready;
    return sdcard_ready;
}
