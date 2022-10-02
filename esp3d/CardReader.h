#ifndef _CARD_READER_H_
#define _CARD_READER_H_

#include "config.h"
#include <stdint.h>

class CardReader {
    public:
        CardReader();
        virtual ~CardReader();
        void begin();
        bool isSdReady();
        uint64_t getCardSize(bool refresh = false);
    private:
        bool initSdCard();
        bool _sd_ready;
        uint8_t _card_type;
        uint64_t _card_size;
};

extern CardReader cardReader;

#endif
