#include <driver/i2c.h>
#include <tuple>

namespace pixy2 {

class LinkI2C {
public:

    static LinkI2C withoutBusInit(i2c_port_t bus_num, uint8_t address) {
        return LinkI2C(bus_num, address, false);
    }

    static std::tuple<LinkI2C, esp_err_t> withBusInit(i2c_port_t bus_num, uint8_t address,
        gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t speed_hz = 500000) {
        
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = sda_pin;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = scl_pin;            
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master = {
                .clk_speed = speed_hz
        };       

        auto instance = LinkI2C(bus_num, address, true);

        esp_err_t err = i2c_param_config(bus_num, &conf);
        if(err != ESP_OK) {
            return std::make_tuple(std::move(instance), err);
        }

        err = i2c_driver_install(bus_num, I2C_MODE_MASTER, 0, 0, 0);
        if(err != ESP_OK) {
            return std::make_tuple(std::move(instance), err);
        }

        return std::make_tuple(std::move(instance), ESP_OK);
    }

    LinkI2C(LinkI2C&& other) : m_bus_num(other.m_bus_num), m_address(other.m_address) {
        if(other.m_ownsBus) {
            this->m_ownsBus = true;
            other.m_ownsBus = false;
        }
    }

    ~LinkI2C() {
        if(m_ownsBus) {
            i2c_driver_delete(m_bus_num);
        }
    }

    esp_err_t receiveData(uint8_t *dest, size_t len) const;
    esp_err_t sendData(const uint8_t *data, size_t len) const;

private:
    LinkI2C(i2c_port_t bus, uint8_t address, bool ownsBus = false) : m_bus_num(bus), m_address(address), m_ownsBus(ownsBus) {

    }

    i2c_port_t m_bus_num;
    uint8_t m_address;
    bool m_ownsBus;
};

};
