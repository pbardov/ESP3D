#include "sdproxy.h"
#include "cardreader.h"
#include "sd_diskio_crc.h"

void sdproxy_spi_handler(void * pvParams);
void sdproxy_cmd_handler(void * pvParams);
void sdproxy_blk_handler(void * pvParams);

SDProxy sdproxy;

const cid_t default_cid = {
    .mid = 0x000001, // Panasonic
    .oid = {'P','A'},
    .pnm = {'E', 'S', 'P', '3', 'D'},
    .prv_m = 0x1,
    .prv_n = 0x0,
    .psn = 0x12345678,
    .mdt_year_high = 0x1,
    .reserved = 0x0,
    .mdt_month = 10,
    .mdt_year_low = 0x6,
    .always1 = 1,
    .crc = 0x0
};

const csd_t default_csd = {
    .v2 = {
        .reserved1 = 0x0,
        .csd_ver = 0x1,
        .taac = 0x0E,
        .nsac = 0x0,
        .tran_speed = 0x32,
        .ccc_high = 0x5b,
        .read_bl_len = 0x09,
        .ccc_low = 0x5,
        .reserved2 = 0x0,
        .dsr_imp = 0x0,
        .read_blk_misalign = 0x0,
        .write_blk_misalign =  0x0,
        .read_bl_partial = 0x0,
        .reserved3 = 0x0,
        .c_size_high = 0x0,
        .c_size_mid = 0x0,
        .c_size_low = 0x0,
        .sector_size_high = 0x3f,
        .erase_blk_en = 0x1,
        .reserved4 = 0x0,
        .wp_grp_size = 0x0,
        .sector_size_low = 0x1,
        .write_bl_len_high = 0x2, // 0x9 >> 2
        .r2w_factor = 0x2,
        .reserved5 = 0x0,
        .wp_grp_enable = 0x0,
        .reserved6 = 0x0,
        .write_partial = 0x0,
        .write_bl_len_low = 0x1, // 0x9 & 0b011
        .reserved7 = 0x0,
        .file_format = 0x0,
        .tmp_write_protect = 0x0,
        .perm_write_protect = 0x0,
        .copy = 0x0,
        .file_format_grp =  0x0,
        .always1 = 0x1,
        .crc = 0x0
    }
};

static constexpr uint16_t SDPROXY_SPI_DELAY {100};
static constexpr uint8_t SDPROXY_CORE_SPI {0};
static constexpr uint8_t SDPROXY_CORE_CMD {0}; // {tskNO_AFFINITY};
static constexpr uint8_t SDPROXY_CORE_BLK {0}; // {tskNO_AFFINITY};

static constexpr uint8_t SDPROXY_SEND_NONE {0x0};
static constexpr uint8_t SDPROXY_SEND_CID {0x1};
static constexpr uint8_t SDPROXY_SEND_CSD {0x2};
static constexpr uint8_t SDPROXY_SEND_BLK {0x3};

static constexpr size_t SDPROXY_DUMMY_BYTES {2};

static constexpr uint8_t SDPROXY_MSK_IS_CMD {0xC0};
static constexpr uint8_t SDPROXY_VAL_IS_CMD {0x40};
static constexpr uint8_t SDPROXY_MSK_CMD {0x3F};

typedef union {
    uint8_t b[6];
    struct Fields {
        uint8_t start : 2;
        uint8_t cmd : 6;
        uint32_t arg : 32;
        uint8_t crc : 8;
    } f;
} SDCommand;

SDProxy::SDProxy() {
    memcpy(&_cid, &default_cid, sizeof(cid_t));
    memcpy(&_csd, &default_csd, sizeof(csd_t));
    // Clear buffers
    _clearTxBuf();
    _clearRxBuf();
    memset(_tx_dummy, 0xff, SDPROXY_RXBUF_SIZE);
    // Setup Tasks
    xTaskCreatePinnedToCore(sdproxy_spi_handler, "sdproxy_spi_handler", 2048, this,  2 | portPRIVILEGE_BIT, &_tsk_spi_handler, SDPROXY_CORE_SPI);
    xTaskCreatePinnedToCore(sdproxy_cmd_handler, "sdproxy_cmd_handler", 2048, this, 2, &_tsk_cmd_handler, SDPROXY_CORE_CMD);
    xTaskCreatePinnedToCore(sdproxy_blk_handler, "sdproxy_blk_handler", 2048, this, 2, &_tsk_blk_handler, SDPROXY_CORE_BLK);
}

SDProxy::~SDProxy() {
    //
}

bool SDProxy::init(spi_host_device_t spi_host, const int8_t sck, const int8_t miso, const int8_t mosi, const int8_t ss) {
    // Set SPI Bus configuration
    _spi_bus_cfg.sclk_io_num = sck;
    _spi_bus_cfg.miso_io_num = miso;
    _spi_bus_cfg.mosi_io_num = mosi;
    _spi_ifc_cfg.spics_io_num = ss;
    _spi_host = spi_host;

    _check_crc = false;
    _hcs = false;
    _cmd55 = false;
    _mode = SDPROXY_SEND_NONE;
    _read_count = 0;
    _r2 = R2_READY_STATE;
    _clearTxBuf();
    if (sdcard_ready) {
        pinMode(SPI_PROXY_SCK, INPUT_PULLDOWN);
        pinMode(SPI_PROXY_MOSI, INPUT_PULLUP);
        pinMode(SPI_PROXY_SS, INPUT_PULLUP);
        pinMode(SPI_PROXY_MISO, INPUT_PULLUP);

        _is_ready = spi_slave_initialize(_spi_host, &_spi_bus_cfg, &_spi_ifc_cfg, SPI_DMA_DISABLED) == ESP_OK;

        // SET_PERI_REG_BITS(SPI_CTRL2_REG(HSPI), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);		// set to 0
        // SET_PERI_REG_BITS(SPI_CTRL2_REG(HSPI), SPI_MISO_DELAY_NUM, 0, SPI_MISO_DELAY_NUM_S);		// set to 0
        // SET_PERI_REG_BITS(SPI_CTRL2_REG(HSPI), SPI_MOSI_DELAY_MODE, 2, SPI_MOSI_DELAY_MODE_S);		// set to 2
        // SET_PERI_REG_BITS(SPI_CTRL2_REG(HSPI), SPI_MOSI_DELAY_NUM, 2, SPI_MOSI_DELAY_NUM_S);		// set to 2

        _size = SD_DRV.cardSize();
        _capacity = _size / 512;
        _csd.v2.c_size_low = _capacity & 0xff;
        _csd.v2.c_size_mid = (_capacity >> 8) & 0xff;
        _csd.v2.c_size_high = (_capacity >> 16) & 0x1f;
        _csd.v2.crc = CRC7(&_csd, 15);

        _cid.crc = CRC7(&_cid, 15);
    } else {
        if (_is_ready) spi_slave_free(_spi_host);
        _is_ready = false;
    }
    LOG ("SD Proxy RDY = ");
    LOG (CONFIG::intTostr(_is_ready));
    LOG ("\n");
    return _is_ready;
}

bool SDProxy::start() {
    if (_is_ready) {
        if (!_is_running) xTaskNotifyGive(_tsk_spi_handler);
        while (!_is_running) vTaskDelay(100);
    }
    return _is_running && !_is_stopped;
}

bool SDProxy::stop() {
    bool was_running = _is_running && !_is_stopped;
    _is_running = false;
    while (!_is_stopped) vTaskDelay(100);
    return was_running;
}

void SDProxy::_spiComm() {
    _is_running = true;
    _is_stopped = false;
    _rx_pos = 0;
    while (_is_running) {
        size_t tlen = _spiNext(_rx_buf, _tx_size > 0 ? _tx_buf + _tx_pos : NULL);
        // check if command start sequence recieved
        if (_tx_size > 0) {
            _tx_size = tlen < _tx_size ? _tx_size - tlen : 0;
            _tx_pos += tlen;
        }
        for (int n = 0; n < tlen; ++n) {
            uint8_t rx_byte = _rx_buf[n];
            if ((rx_byte & 0xC0) == 0x40) {
                _cmd_pos = n;
                _processing = true;
                xTaskNotifyGive(_tsk_cmd_handler);
                break;
            }
        }

        // check if we can take next block
        if (_mode != MODE_IDLE && _tx_size == 0 && !_processing) {
            _processing = true;
            xTaskNotifyGive(_tsk_blk_handler);
        }
    }
    _is_stopped = true;
}

void SDProxy::_readData() {
    _tx_size = 0;
    _tx_pos  = 0;
    if (_mode == MODE_CID || _mode == MODE_CSD) {
        const void * reg = _mode == MODE_CID ? (const void *)&_cid : (const void *)&_csd;
        _tx_buf[0]       = 0xff;
        _tx_buf[1]       = DATA_START_BLOCK;
        memcpy(_tx_buf + 2, reg, 16);
        *reinterpret_cast<uint16_t *>(_tx_buf + 18) = CRC16(_tx_buf + 2, 16);
        size_t dummy = SDPROXY_SPI_SIZE - 20;
        memset(_tx_buf + 20, 0xff, dummy);
        _tx_size = SDPROXY_SPI_SIZE;
        _mode    = MODE_IDLE;
    } else if (_mode == MODE_BLK) {
        if (_read_count > 0) {
            --_read_count;
            if (_blknum > _capacity) {
                // block number out of range
                _r2         = R2_OUT_OF_RANGE;
                _read_count = 0; // stop reading
                _mode       = MODE_IDLE;
            } else if (!sd_read_block(_tx_buf + 2, _blknum)) {
                LOG ("SD read block ");
                LOG (CONFIG::intTostr(_blknum));
                LOG (" failed\n\r");
                _r2         = R2_ERROR;
                _read_count = 0;
                _mode       = MODE_IDLE;
            } else if (_mode == MODE_BLK) { // We still able to read block ?
                _r2        = R2_READY_STATE;
                _tx_buf[0] = 0xff;
                _tx_buf[1] = DATA_START_BLOCK;
                *reinterpret_cast<uint16_t *>(_tx_buf + 514) = CRC16(_tx_buf + 2, 512);
                size_t dummy = SDPROXY_SPI_SIZE - (516 % SDPROXY_SPI_SIZE);
                if (dummy > 0) {
                    memset(_tx_buf + 516, 0xff, dummy);
                }
                _tx_size   = 516 + dummy;
            } else {
                // We not able to read
                _tx_size = 0;
                _tx_pos  = 0;
            }
        } else {
            _mode = MODE_IDLE;
        }
    }
    _processing = false;
}

void SDProxy::_execCommand() {
    uint8_t  cmd  = _rx_buf[_cmd_pos] & 0x3f;                              // get command number from buffer
    uint32_t arg  = *reinterpret_cast<uint32_t *>(_rx_buf + _cmd_pos + 1); // get command argument
    uint8_t  crc  = _rx_buf[_cmd_pos + 5];                                 // command crc
    // reset rx_pos after command read complete
    _rx_pos = 0;

    // Compute valid CRC
    uint8_t valid_crc     =  CRC7(_rx_buf + _cmd_pos, 5);
    bool    is_crc_valid  =  crc == valid_crc;

    // Debug loggin
#ifdef DEBUG_ESP3D
    {
        char pbuf[128];
        sniprintf(pbuf, sizeof(pbuf), "%d CMD%d %08X \n\r", is_crc_valid, cmd, arg);
        LOG (pbuf);
    }
#endif

    if (!is_crc_valid && _mode != MODE_IDLE) {
        // skip wrong crc commands when transmission mode
        _processing = false;
        return;
    }

    // reset current transmission
    _tx_size = 0;
    _tx_pos  = 0;

    // set transmission buffer pointer
    _tx_buf[0] = 0xff;
    _tx_buf[1] = 0xff;
    uint8_t * tx      =  _tx_buf + 2;
    size_t    tx_size =  0;

    if (is_crc_valid) {
        // Process command if CRC valid
        if (!_cmd55) {
            switch (cmd) {
                case CMD0: // GO_IDLE_STATE - init card in spi mode
                {
                    _r2     = R2_READY_STATE;
                    tx[0]   = R1_IDLE_STATE;
                    tx_size = 1;
                    break;
                }
                case CMD8: // SEND_IF_COND - verify SD Memory Card interface operating condition
                {
                    _r2              = R2_READY_STATE;
                    uint8_t result[] = {0x0, 0x0, 0xf0, static_cast<uint8_t>((arg >> 24) & 0xff)};
                    tx[0]            = R1_READY_STATE;
                    memcpy(tx + 1, result, sizeof(result));
                    tx_size = 5;
                    break;
                }
                case CMD9: // SEND_CSD - read the Card Specific Data (CSD register)
                {
                    _mode       = MODE_CSD;
                    _read_count = 0;
                    _r2         = R2_READY_STATE;
                    tx[0]       = R1_READY_STATE;
                    tx_size     = 1;
                    break;
                }
                case CMD10: // SEND_CID - read the card identification information (CID register)
                {
                    _mode       = MODE_CID;
                    _read_count = 0;
                    _r2         = R2_READY_STATE;
                    tx[0]       = R1_READY_STATE;
                    tx_size     = 1;
                    break;
                }
                case CMD12: // STOP_TRANSMISSION - end multiple block read sequence
                {
                    _mode       = MODE_IDLE;
                    _read_count = 0;
                    _r2         = R2_READY_STATE;
                    tx[0]       = R1_READY_STATE;
                    tx_size     = 1;
                    break;
                }
                case CMD13: // SEND_STATUS - read the card status register
                {
                    tx[0]   = R1_READY_STATE;
                    tx[1]   = _r2;
                    tx_size = 2;
                    break;
                }
                case CMD17: // READ_SINGLE_BLOCK - read a single data block from the card
                case CMD18: // READ_MULTIPLE_BLOCK - read a multiple data blocks from the card
                {
                    _mode       = MODE_BLK;
                    _r2         = R2_READY_STATE;
                    _blknum     = arg;
                    _read_count = cmd == CMD17 ? 1 : 0xffffffff;
                    if (_blknum >= _capacity) {
                        _mode       = MODE_IDLE;
                        _read_count = 0;
                        _r2        |= R2_OUT_OF_RANGE;
                        tx[0]       = R1_PARAM_ERROR;
                    } else {
                        tx[0] = R1_READY_STATE;
                    }
                    tx_size = 1;
                    break;
                }
                case CMD24: // WRITE_BLOCK - write a single data block to the card
                case CMD25: // WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION
                case CMD32: // ERASE_WR_BLK_START - sets the address of the first block to be erased
                case CMD33: // ERASE_WR_BLK_END - sets the address of the last block of the continuous range to be erased
                case CMD38: // ERASE - erase all previously selected blocks
                {
                    // Act as read only SD card
                    _r2     = R2_WRITE_PROTECT | R2_WP_VIOLATION;
                    tx[0]   = R1_ERASE_RESET;
                    tx_size = 1;
                    break;
                }
                case CMD55: // APP_CMD - escape for application specific command
                {
                    _cmd55  = true;
                    tx[0]   = R1_READY_STATE;
                    tx_size = 1;
                    break;
                }
                case CMD58: // READ_OCR - read the OCR register of a card
                {
                    tx[0]                                 = R1_READY_STATE;
                    *reinterpret_cast<uint32_t *>(tx + 1) = 0xffffff01; // CCS bit is set, power up is set too
                    tx_size = 5;
                    break;
                }
                case CMD59: // CRC_ON_OFF - enable or disable CRC checking
                {
                    _check_crc = arg != 0;
                    tx[0]      = R1_READY_STATE;
                    tx_size    = 1;
                    break;
                }
                default:
                {
                    tx[0]   = R1_ILLEGAL_COMMAND;
                    tx_size = 1;
                    break;
                }
            }
        } else {
            _cmd55 = false; // reset _cmd55
            switch (cmd) {
                case ACMD23: // SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be pre-erased before writing
                {
                    _r2     = R2_WP_VIOLATION | R2_WRITE_PROTECT;
                    tx[0]   = R1_ERASE_RESET;
                    tx_size = 1;
                    break;
                }
                case ACMD41: // SD_SEND_OP_COMD - Sends host capacity support information and activates the card's initialization process
                {
                    _r2  = R2_READY_STATE;
                    _hcs = arg & 0x40; // Set HCS
                    break;
                }
                default:
                {
                    tx[0]   = R1_ILLEGAL_COMMAND;
                    tx_size = 1;
                    break;
                }
            }
        }
    } else {
        // CRC invalid
        tx[0]   = R1_CRC_ERROR;
        tx_size = 1;
    }

    size_t dummy = SDPROXY_SPI_SIZE - (tx_size + 2);
    if (dummy > 0 && dummy < SDPROXY_SPI_SIZE) {
        memset(_tx_buf + 2 + tx_size, 0xff, dummy);
    }

    _tx_pos     = 0;
    _tx_size    = 2 + tx_size + dummy;
    _processing = false;
}

void sdproxy_spi_handler(void * pvParams) {
    SDProxy * sdp = reinterpret_cast<SDProxy *>(pvParams);
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sdp->_spiComm();
    }
}

void sdproxy_cmd_handler(void * pvParams) {
    SDProxy * sdp = reinterpret_cast<SDProxy *>(pvParams);
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sdp->_execCommand();
    }
}

void sdproxy_blk_handler(void * pvParams) {
    SDProxy * sdp = reinterpret_cast<SDProxy *>(pvParams);
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sdp->_readData();
    }
}

void spi_slave_setup_done(spi_slave_transaction_t* trans) {
    //
}

inline void spi_slave_trans_done(spi_slave_transaction_t* trans) {
    //
}
