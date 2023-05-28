#pragma once

#include <driver/spi_master.h>
#include <tuple>

namespace pixy2 {


class LinkSpi {
public:
    // spiDev is now owned by Pixy2 and gets destroyed in destructor
    static LinkSpi existingSpiDevice(spi_device_handle_t spiDev) {
        return LinkSpi(spiDev);
    }

    // host has to be already initialized by spi_bus_initialize
    static std::tuple<LinkSpi, esp_err_t> addSpiDevice(spi_host_device_t host, int frequency_hz = 6000000) {
        spi_device_interface_config_t devCfg = { }; // ty prazdne slozene zavorky jsou tady proto, aby se na vychozi hodnotu nastavily automaticky ty promenne, ktere nejsou nastavene na nasledujicich radcich -> bez nich to nejede spravne 
        devCfg.mode = 3;
        devCfg.clock_speed_hz = frequency_hz;
        devCfg.spics_io_num = -1;
        devCfg.queue_size = 1;

        spi_device_handle_t spiDev;
        auto err = spi_bus_add_device(host, &devCfg, &spiDev);
        if(err != ESP_OK) {
            return std::make_tuple(LinkSpi(nullptr), err);
        }
        return std::make_tuple(LinkSpi(spiDev), ESP_OK);
    }

    LinkSpi(LinkSpi&& other) {
        this->m_spiDev = other.m_spiDev;
        other.m_spiDev = nullptr;
    };

    ~LinkSpi() {
        if(m_spiDev != nullptr) {
            spi_bus_remove_device(m_spiDev);
        }
    }

    esp_err_t receiveData(uint8_t *dest, size_t len) const {
        uint8_t zerobuf[32] = { };
        while (len > 0)
        {
            const size_t chunk = std::min(sizeof(zerobuf), len);

            spi_transaction_t trans = {};
            trans.flags = 0;
            trans.length = chunk * 8;
            trans.tx_buffer = zerobuf;
            trans.rx_buffer = dest;
            

            auto err = spi_device_transmit(m_spiDev, &trans);
            if (err != ESP_OK)
            {
                return err;
            }
            dest += chunk;
            len -= chunk;
        }
        return ESP_OK;
    }

    esp_err_t sendData(const uint8_t *data, size_t len) const {
        spi_transaction_t trans = {};
        trans.flags = 0;
        trans.length = len * 8;
        trans.tx_buffer = data;
        
        return spi_device_transmit(m_spiDev, &trans);
    }

private:
    LinkSpi(spi_device_handle_t spiDev) : m_spiDev(spiDev) {

    }
    LinkSpi(const LinkSpi&) = delete;

    spi_device_handle_t m_spiDev;
};

};
