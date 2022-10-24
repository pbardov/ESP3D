#ifndef SDPROXY_H
#define SDPROXY_H

#include <driver/spi_slave.h>
#include "config.h"
#include "SdInfo.h"

#ifndef SPI_PROXY_HOST
#define SPI_PROXY_HOST HSPI_HOST
#endif

#ifndef SPI_PROXY_MODE
#define SPI_PROXY_MODE SPI_MODE1
#endif

static constexpr size_t SDPROXY_SPI_SIZE {32};
static constexpr size_t SDPROXY_RXBUF_SIZE {SDPROXY_SPI_SIZE * 2};
static constexpr size_t SDPROXY_TXBUF_SIZE {576};

void spi_slave_setup_done(spi_slave_transaction_t* trans);
void spi_slave_trans_done(spi_slave_transaction_t* trans);

class SDProxy {
    public:
        SDProxy();
        virtual ~SDProxy();
        bool init(spi_host_device_t spi_host = SPI_PROXY_HOST, const int8_t sck = SPI_PROXY_SCK, const int8_t miso = SPI_PROXY_MISO, const int8_t mosi = SPI_PROXY_MOSI, const int8_t ss = SPI_PROXY_SS);
        inline bool isReady() const { return _is_ready; }
        bool start();
        bool stop();
    protected:
        friend void spi_slave_setup_done(spi_slave_transaction_t* trans);
        friend void spi_slave_trans_done(spi_slave_transaction_t* trans);
        friend void sdproxy_spi_handler(void * pvParams);
        friend void sdproxy_cmd_handler(void * pvParams);
        friend void sdproxy_blk_handler(void * pvParams);

        // internal variables
        volatile bool _is_ready {false};   // is SDProxy ready
        volatile bool _is_running {false}; // is SDProxy communication running
        volatile bool _is_stopped {true};  // is SDProxy comm loop finished
        volatile bool _processing {false}; // true - data fetching now by sdproxy_blk_handler

        // SPI Bus
        spi_bus_config_t _spi_bus_cfg {
            .mosi_io_num = SPI_PROXY_MOSI,
            .miso_io_num = SPI_PROXY_MISO,
            .sclk_io_num = SPI_PROXY_SCK,
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
            .flags = SPICOMMON_BUSFLAG_SLAVE,
        };
        spi_slave_interface_config_t _spi_ifc_cfg {
            .spics_io_num = SPI_PROXY_SS,
            .queue_size = 6,
            .mode = SPI_PROXY_MODE,
            .post_setup_cb = spi_slave_setup_done,
            .post_trans_cb = spi_slave_trans_done
        };
        spi_host_device_t       _spi_host {SPI_PROXY_HOST};
        // SPI transaction
        spi_slave_transaction_t _spi_t;

        // Dummy transmit buffer
        uint8_t _tx_dummy[SDPROXY_RXBUF_SIZE];

        // Communication buffers
        uint8_t _rx_buf[SDPROXY_RXBUF_SIZE];
        uint8_t _tx_buf[SDPROXY_TXBUF_SIZE];
        // tx/rx positions
        volatile int    _rx_pos {0};
        volatile int    _tx_pos {0};
        volatile size_t _rx_size {0};
        volatile size_t _tx_size {0};
        // command position in tx_buf
        volatile int    _cmd_pos {0};

        // Task specific variables
        TaskHandle_t _tsk_spi_handler {0};
        TaskHandle_t _tsk_cmd_handler {0};
        TaskHandle_t _tsk_blk_handler {0};

        // SPI communication methods
        size_t       _spiNext(void * rx_buf, const void * tx_buf = NULL);
        void         _spiComm();

        void _execCommand();
        void _readData();

        inline void _clearRxBuf() { memset(_rx_buf, 0x00, SDPROXY_RXBUF_SIZE); }
        inline void _clearTxBuf() { memset(_tx_buf, 0xff, SDPROXY_TXBUF_SIZE); }

        // Internal virtual SD Card state members
        bool     _check_crc;    // is CRC on/off (changed by CMD59)
        bool     _hcs;          // HCS on/off (changed by ACMD41)
        bool     _cmd55;        // next command is ACMD
        // Working mode
        uint8_t _mode {SDProxy::MODE_IDLE}; // current mode
        static constexpr uint8_t MODE_IDLE  {0};
        static constexpr uint8_t MODE_CID   {1};
        static constexpr uint8_t MODE_CSD   {2};
        static constexpr uint8_t MODE_BLK   {3};
        // SD registries
        uint8_t   _r2;  // R2 register
        cid_t     _cid; // CID register
        csd_t     _csd; // CSD register
        // real card infos
        uint64_t  _size;     // Card size in bytes
        uint64_t  _capacity;  // Card size in 512 byte blocks
        uint32_t _blknum;
        size_t _read_count;
};

extern SDProxy sdproxy;

inline size_t SDProxy::_spiNext(void * rx_buf, const void * tx_buf) {
    _spi_t.length    = 8 * SDPROXY_SPI_SIZE;
    _spi_t.trans_len = 0;
    _spi_t.tx_buffer = tx_buf != NULL ? tx_buf : _tx_dummy;
    _spi_t.rx_buffer = rx_buf;
    _spi_t.user = (void *)this;

    esp_err_t res = spi_slave_transmit(_spi_host, &_spi_t, portMAX_DELAY);
    if (res == ESP_OK) {
        size_t tlen = _spi_t.trans_len / 8; // recieved data len in bytes
        if (_spi_t.trans_len % 8 > 0) {
            ++tlen; // WTF ??? Its need?
        }
        return tlen;
    }
    return 0;
}

#endif
