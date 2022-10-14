#ifndef SDPROXY_H
#define SDPROXY_H

#include <ESP32SPISlave.h>
#include "SdInfo.h"

static constexpr size_t SDPROXY_BUFSIZE {544};

class SDProxy {
    public:
        SDProxy();
        bool init();
        bool isReady() const { return _is_ready; }
        void handle();
    protected:
        ESP32SPISlave _spi;
        bool _is_ready;
        bool _check_crc;
        bool _hcs;
        bool _cmd55;
        uint8_t _send_reg;
        uint8_t _r2;
        cid_t _cid;
        csd_t _csd;
        uint64_t _size;
        uint64_t _capacity;
        uint8_t _rx_buf[SDPROXY_BUFSIZE];
        uint8_t _tx_buf[SDPROXY_BUFSIZE];
        uint32_t _blknum;
        size_t _read_count;
};

extern SDProxy sdproxy;

#endif
