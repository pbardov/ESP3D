#include "CardReader.h"

#ifdef SDCARD_MMC
#include <SD_MMC.h>
#define SD_DRV SD_MMC
#else
#include <SD.h>
#define SD_DRV SD
#endif

CardReader cardReader;

CardReader::CardReader() {
    //
}

CardReader::~CardReader() {
    //
}

void CardReader::begin() {
    initSdCard();
}

bool CardReader::isSdReady() {
    return _sd_ready && _card_type != CARD_NONE && _card_type != CARD_UNKNOWN;
}

uint64_t CardReader::getCardSize(bool refresh) {
    if (refresh && isSdReady()) {
        _card_size = SD_DRV.cardSize();
    }
    return _card_size;
}

bool CardReader::initSdCard() {
    if ((_sd_ready = SD_DRV.begin())) {
        _card_type = SD_DRV.cardType();
        if (!isSdReady()) {
            return false;
        }
        _card_size = SD_DRV.cardSize();
    }
    return isSdReady();
}
