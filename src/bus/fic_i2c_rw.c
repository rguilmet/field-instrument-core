#include <string.h>
#include "fic_i2c_internal.h"

bool fic_i2c_probe_addr(uint8_t addr_7bit)
{
    if (fic_i2c0_bus == NULL) return false;
    if (!fic_i2c_lock_port(0U, 200)) return false;
    esp_err_t err = i2c_master_probe(fic_i2c0_bus, addr_7bit, 50);
    if (err != ESP_OK) {
        fic_i2c_recover_port_locked(0U, "Probe", NULL, addr_7bit, err);
    }
    fic_i2c_unlock_port(0U);
    return (err == ESP_OK);
}

bool fic_i2c_probe_optional_addr_port(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U) return false;
    i2c_master_bus_handle_t bus_handle = *g_fic_i2c_ports[port].bus_ptr;
    if (bus_handle == NULL) return false;
    if (!fic_i2c_lock_port(port, 200)) return false;
    esp_err_t err = i2c_master_probe(bus_handle, addr_7bit, 25);
    fic_i2c_unlock_port(port);
    return (err == ESP_OK);
}

bool fic_i2c_probe_optional_addr(uint8_t addr_7bit)
{
    return fic_i2c_probe_optional_addr_port(0U, addr_7bit);
}

void fic_i2c_report_status_port(uint8_t port, i2c_master_dev_handle_t dev_handle, const char *action, int err_code)
{
    if (port > 1U || !fic_i2c_lock_port(port, 200)) {
        return;
    }
    if (err_code == ESP_OK) {
        g_fic_i2c_ports[port].consec_timeouts = 0;
    } else {
        fic_i2c_recover_port_locked(port, action ? action : "Op", dev_handle, -1, (esp_err_t)err_code);
    }
    fic_i2c_unlock_port(port);
}

void fic_i2c_report_status(i2c_master_dev_handle_t dev_handle, const char *action, int err_code)
{
    uint8_t port = (dev_handle != NULL && dev_handle == fic_beam_imu_dev) ? g_fic_beam_imu_bus : 0U;
    fic_i2c_report_status_port(port, dev_handle, action, err_code);
}

uint8_t fic_i2c_write_buff_port(uint8_t port, i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (port > 1U || !fic_i2c_lock_port(port, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        fic_i2c_unlock_port(port);
        return 1;
    }
    esp_err_t ret;
    if (reg == -1) {
        ret = i2c_master_transmit(dev_handle, buf, len, g_fic_i2c_timeout_ticks);
    } else {
        uint8_t pbuf[256];
        pbuf[0] = (uint8_t)reg;
        if (len > 0 && buf != NULL) {
            memcpy(&pbuf[1], buf, len);
        }
        ret = i2c_master_transmit(dev_handle, pbuf, (size_t)len + 1U, g_fic_i2c_timeout_ticks);
    }
    if (ret == ESP_OK) {
        g_fic_i2c_ports[port].consec_timeouts = 0;
    } else {
        fic_i2c_recover_port_locked(port, "Write", dev_handle, reg, ret);
    }
    fic_i2c_unlock_port(port);
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t fic_i2c_read_buff_port(uint8_t port, i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (port > 1U || !fic_i2c_lock_port(port, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        fic_i2c_unlock_port(port);
        return 1;
    }
    esp_err_t ret;
    if (reg == -1) {
        ret = i2c_master_receive(dev_handle, buf, len, g_fic_i2c_timeout_ticks);
    } else {
        uint8_t addr = (uint8_t)reg;
        ret = i2c_master_transmit_receive(dev_handle, &addr, 1, buf, len, g_fic_i2c_timeout_ticks);
    }
    if (ret == ESP_OK) {
        g_fic_i2c_ports[port].consec_timeouts = 0;
    } else {
        fic_i2c_recover_port_locked(port, "Read", dev_handle, reg, ret);
    }
    fic_i2c_unlock_port(port);
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t fic_i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    return fic_i2c_write_buff_port(0U, dev_handle, reg, buf, len);
}

uint8_t fic_i2c_write_read_dev_port(uint8_t port, i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    if (port > 1U || !fic_i2c_lock_port(port, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        fic_i2c_unlock_port(port);
        return 1;
    }
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, writeBuf, writeLen, readBuf, readLen, g_fic_i2c_timeout_ticks);
    if (ret == ESP_OK) {
        g_fic_i2c_ports[port].consec_timeouts = 0;
    } else {
        fic_i2c_recover_port_locked(port, "WriteRead", dev_handle, -1, ret);
    }
    fic_i2c_unlock_port(port);
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t fic_i2c_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    return fic_i2c_write_read_dev_port(0U, dev_handle, writeBuf, writeLen, readBuf, readLen);
}

uint8_t fic_i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    return fic_i2c_read_buff_port(0U, dev_handle, reg, buf, len);
}
