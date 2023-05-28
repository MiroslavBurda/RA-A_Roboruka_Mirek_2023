#include "i2c.hpp"

namespace pixy2 {

#define RETURN_IF_ERR(x) do {                                         \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            return __err_rc;                                            \
        }                                                               \
    } while(0)


class I2cCmdHolder {
public:
    I2cCmdHolder() {
        m_cmd = i2c_cmd_link_create();
    }

    ~I2cCmdHolder() {
        i2c_cmd_link_delete(m_cmd);
    }

    i2c_cmd_handle_t get() const { return m_cmd; }

private:
    i2c_cmd_handle_t m_cmd;
};

esp_err_t LinkI2C::receiveData(uint8_t *dest, size_t len) const
{
    I2cCmdHolder holder;
    auto cmd = holder.get();

    RETURN_IF_ERR(i2c_master_start(cmd));
    RETURN_IF_ERR(i2c_master_write_byte(cmd, m_address << 1 | 1, true));
    RETURN_IF_ERR(i2c_master_read(cmd, dest, len, I2C_MASTER_LAST_NACK));
    RETURN_IF_ERR(i2c_master_stop(cmd));
    RETURN_IF_ERR(i2c_master_cmd_begin(m_bus_num, cmd, pdMS_TO_TICKS(25)));
    return ESP_OK;
}

esp_err_t LinkI2C::sendData(const uint8_t *data, size_t len) const
{
    while (len > 0)
    {
        // Official Arduino lib limits at 16
        const size_t chunk = std::min(16U, len);

        I2cCmdHolder holder;
        auto cmd = holder.get();
        
        RETURN_IF_ERR(i2c_master_start(cmd));
        RETURN_IF_ERR(i2c_master_write_byte(cmd, m_address << 1, true));
        // RETURN_IF_ERR(i2c_master_write(cmd, data, chunk, I2C_MASTER_LAST_NACK)); // esp-idf variant
        RETURN_IF_ERR(i2c_master_write(cmd, (uint8_t*)data, chunk, I2C_MASTER_LAST_NACK));
        RETURN_IF_ERR(i2c_master_stop(cmd));
        RETURN_IF_ERR(i2c_master_cmd_begin(m_bus_num, cmd, pdMS_TO_TICKS(25)));

        data += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

};
