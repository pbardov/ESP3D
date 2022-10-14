#include "sdproxy.h"
#include "cardreader.h"
#include "sd_diskio_crc.h"
#include "sdproxy_handler.h"

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

static constexpr uint8_t SDPROXY_SEND_NONE {0x0};
static constexpr uint8_t SDPROXY_SEND_CID {0x1};
static constexpr uint8_t SDPROXY_SEND_CSD {0x2};
static constexpr size_t SDPROXY_SPI_SIZE {64};

SDProxy::SDProxy() :
    _is_ready(false),
    _check_crc(false),
    _hcs(false),
    _cmd55(false),
    _r2(R2_READY_STATE),
    _send_reg(SDPROXY_SEND_NONE),
    _read_count(0),
    _rx_pos(0),
    _tx_pos(0),
    _tx_size(0) {
    memcpy(&_cid, &default_cid, sizeof(cid_t));
    memcpy(&_csd, &default_csd, sizeof(csd_t));
}

bool SDProxy::init() {
    _rx_pos = 0;
    _tx_pos = 0;
    _tx_size = 0;
    _check_crc = false;
    _hcs = false;
    _cmd55 = false;
    _send_reg = SDPROXY_SEND_NONE;
    _read_count = 0;
    _r2 = R2_READY_STATE;
    if (sdcard_ready) {
        pinMode(SPI_PROXY_SCK, INPUT_PULLDOWN);
        pinMode(SPI_PROXY_MOSI, INPUT_PULLDOWN);
        pinMode(SPI_PROXY_SS, INPUT_PULLUP);
        pinMode(SPI_PROXY_MISO, INPUT_PULLUP);

        _spi.setDataMode(SPI_MODE1);
        _is_ready = _spi.begin(VSPI, SPI_PROXY_SCK, SPI_PROXY_MISO, SPI_PROXY_MOSI, SPI_PROXY_SS);
        _size = SD_DRV.cardSize();
        _capacity = _size / 512;
        _csd.v2.c_size_low = _capacity & 0xff;
        _csd.v2.c_size_mid = (_capacity >> 8) & 0xff;
        _csd.v2.c_size_high = (_capacity >> 16) & 0x1f;
        _csd.v2.crc = CRC7(&_csd, 15);

        _cid.crc = CRC7(&_cid, 15);
    } else {
        if (_is_ready) _spi.end();
        _is_ready = false;
    }
    LOG ("SD Proxy RDY = ");
    LOG (CONFIG::intTostr(_is_ready));
    LOG ("\n");
    return _is_ready;
}

void SDProxy::handle() {
    if (_is_ready) {
        if (_spi.remained() == 0) {
            if (_tx_pos >= _tx_size) {
                _tx_pos = 0;
                _tx_size = 0;
                memset(_tx_buf, 0xff, SDPROXY_BUFSIZE);
            }
            _spi.queue(_rx_buf, _tx_buf + _tx_pos, SDPROXY_SPI_SIZE);
            _tx_pos += SDPROXY_SPI_SIZE;
        }

        if ((_rx_size = _spi.available())) {
#ifdef DEBUG_ESP3D
            LOG ("SD_SPI <-[");
            LOG (CONFIG::intTostr(_rx_size));
            LOG ("] ");
            char pbuf[] = "  ";
            for (int n = 0; n < _rx_size; ++n) {
                snprintf(pbuf, sizeof(pbuf), "%02X", _rx_buf[n]);
                LOG (pbuf);
            }
            LOG ("\n\r");
#endif
            bool bad_req = false;
            for (_rx_pos = 0; _rx_pos < _rx_size; ++_rx_pos) {
                uint8_t cmd = _rx_buf[_rx_pos];
                if (cmd == 0xff) continue;
                if ((cmd & 0xC0) == 0x40) {
                    uint32_t arg = _rx_pos + 5 <= _rx_size ? *reinterpret_cast<uint32_t *>(_rx_buf + _rx_pos + 1) : 0;
                    _tx_size = _execCmd(cmd & 0x3F, arg);
                } else {
                    bad_req = true;
                    break;
                }
            }

            if (_tx_size == 0 && !bad_req) {
                if (_send_reg) {
                    const void * reg = _send_reg == SDPROXY_SEND_CID ? reinterpret_cast<const void *>(&_cid) : reinterpret_cast<const void *>(&_csd);
                    _tx_buf[0] = DATA_START_BLOCK;
                    memcpy(_tx_buf + 1, reg, 16);
                    *reinterpret_cast<uint16_t *>(_tx_buf + 17) = CRC16(_tx_buf + 1, 16);
                    _tx_size = 19;
                    _send_reg = SDPROXY_SEND_NONE;
                } else if (_read_count > 0) {
                    _tx_buf[0] = DATA_START_BLOCK;
                    if (_blknum >= _capacity) {
                        _r2 = R2_OUT_OF_RANGE;
                        _read_count = 0;
                        _tx_size = 0;
                    } else if (!sd_read_block(_tx_buf + 1, _blknum)) {
                        LOG ("SD read block ");
                        LOG (CONFIG::intTostr(_blknum));
                        LOG (" failed\n\r");
                        _r2 = R2_ERROR;
                        _read_count = 0;
                        _tx_size = 0;
                    } else {
                        _r2 = R2_READY_STATE;
                        *reinterpret_cast<uint16_t *>(_tx_buf + 513) = CRC16(_tx_buf + 1, 512);
                        _tx_size = 515;
                        --_read_count;
                        ++_blknum;
                    }
                }
            }

            _spi.pop();
        }
    }
}

size_t SDProxy::_execCmd(uint8_t cmd, uint32_t arg) {
#ifdef DEBUG_ESP3D
    if (!_cmd55) { LOG ("SD CMD"); }
    else { LOG ("SD ACMD"); }
    LOG (CONFIG::intTostr(cmd));
    LOG ("\n\r");
#endif
    if (!_cmd55) {
        if (cmd == CMD0) {                   // GO_IDLE_STATE - init card in spi mode
            _r2 = R2_READY_STATE;
            _tx_buf[0] = R1_IDLE_STATE;
            return 1;
        }
        if (cmd == CMD8) {                   // SEND_IF_COND - verify SD Memory Card interface operating condition
            _r2 = R2_READY_STATE;
            _tx_buf[0] = R1_READY_STATE;
            _tx_buf[4] = arg & 0xff; // echo-back
            _tx_buf[3] = 0xf0;       // All voltages
            _tx_buf[2] = 0x0;
            _tx_buf[1] = 0x0;        // 0 - command version
            return 5;
        }
        if (cmd == CMD9) {                   // SEND_CSD - read the Card Specific Data (CSD register)
            _send_reg = SDPROXY_SEND_CSD;
            _read_count = 0;
            _r2 = R2_READY_STATE;
            _tx_buf[0] = R1_READY_STATE;
            return 1;
        }
        if (cmd == CMD10) {                  // SEND_CID - read the card identification information (CID register)
            _send_reg = SDPROXY_SEND_CID;
            _read_count = 0;
            _r2 = R2_READY_STATE;
            _tx_buf[0] = R1_READY_STATE;
            return 1;
        }
        if (cmd == CMD12) {                  // STOP_TRANSMISSION - end multiple block read sequence
            _send_reg = SDPROXY_SEND_NONE;
            _read_count = 0;
            _tx_buf[0] = R1_READY_STATE;
            return 1;
        }
        if (cmd == CMD13) {                  // SEND_STATUS - read the card status register
            _tx_buf[0] = R1_READY_STATE;
            _tx_buf[1] = _r2;
            return 2;
        }
        if (cmd == CMD17) {                  // READ_SINGLE_BLOCK - read a single data block from the card
            _r2 = R2_READY_STATE;
            _blknum = arg;
            _read_count = 1;
            if (_blknum >= _capacity) {
                _read_count = 0;
                _r2 |= R2_OUT_OF_RANGE;
                _tx_buf[0] = R1_ADDRESS_ERROR;
            } else {
                _tx_buf[0] = R1_READY_STATE;
            }
            return 1;
        }
        if (cmd == CMD18) {                  // READ_MULTIPLE_BLOCK - read a multiple data blocks from the card
            _r2 = R2_READY_STATE;
            _blknum = arg;
            _read_count = 0xFFFFFFFF;
            if (_blknum >= _capacity) {
                _read_count = 0;
                _r2 |= R2_OUT_OF_RANGE;
                _tx_buf[0] = R1_ADDRESS_ERROR;
            } else {
                _tx_buf[0] = R1_READY_STATE;
            }
            return 1;
        }
        if (cmd == CMD24 || cmd == CMD25 || cmd == CMD32 || cmd == CMD33 || cmd == CMD38) { // ALL Write and erase command
            _r2 = R2_WRITE_PROTECT;
            _tx_buf[0] = R1_ERASE_RESET;
            return 1;
        }
        if (cmd == CMD55) {                  // APP_CMD - escape for application specific command
            _cmd55 = true;
            _tx_buf[0] = R1_READY_STATE;
            return 1;
        }
        if (cmd == CMD59) {                  // CRC_ON_OFF - enable or disable CRC checking
            _check_crc = arg != 0;
            _tx_buf[0] = R1_IDLE_STATE;
            return 1;
        }
    } else {
        _cmd55 = false;
        if (cmd == ACMD23) {                 // SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be pre-erased before writing
            _r2 = R2_WRITE_PROTECT;
            _tx_buf[0] = R1_ERASE_RESET;
            return 1;
        }
        if (cmd == ACMD41) {                 // SD_SEND_OP_COMD - Sends host capacity support information and activates the card's initialization process
            _r2 = R2_READY_STATE;
            _hcs = _rx_buf[1] & 0x40;
            _tx_buf[0] = R1_READY_STATE;
            return 1;
        }
    }

    _r2 = R2_READY_STATE;
    _tx_buf[0] = R1_ILLEGAL_COMMAND;
    return 1;
}

void handleSDProxy() {
    sdproxy.handle();
}
