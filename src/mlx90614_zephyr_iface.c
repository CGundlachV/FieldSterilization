#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "driver_mlx90614_interface.h"

LOG_MODULE_REGISTER(mlx90614, LOG_LEVEL_INF);

static const struct device *i2c_dev;

uint8_t mlx90614_interface_iic_init(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c21), okay)
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c21));
#else
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#endif
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return 1;
    }
    return 0;
}

uint8_t mlx90614_interface_iic_deinit(void)
{
    i2c_dev = NULL;
    return 0;
}

/* MLX90614 passes 8-bit SMBus addresses (0xB4); Zephyr expects 7-bit (0x5A).
   Shift right by 1 to convert. The library uses the 8-bit form only for PEC
   calculation internally; the wire transaction uses the 7-bit form. */
uint8_t mlx90614_interface_iic_read(uint8_t addr, uint8_t reg,
                                     uint8_t *buf, uint16_t len)
{
    if (!i2c_dev) {
        LOG_ERR("iic_read: i2c_dev is NULL");
        return 1;
    }
    int ret = i2c_write_read(i2c_dev, addr >> 1, &reg, 1, buf, len);
    if (ret) {
        LOG_DBG("i2c_write_read(addr=0x%02x reg=0x%02x len=%u) => %d",
                addr >> 1, reg, len, ret);
    }
    return ret ? 1 : 0;
}

uint8_t mlx90614_interface_iic_write(uint8_t addr, uint8_t reg,
                                      uint8_t *buf, uint16_t len)
{
    if (!i2c_dev) {
        LOG_ERR("iic_write: i2c_dev is NULL");
        return 1;
    }
    uint8_t tx[len + 1];
    tx[0] = reg;
    memcpy(&tx[1], buf, len);
    int ret = i2c_write(i2c_dev, tx, len + 1, addr >> 1);
    if (ret) {
        LOG_DBG("i2c_write(addr=0x%02x reg=0x%02x len=%u) => %d",
                addr >> 1, reg, len + 1, ret);
    }
    return ret ? 1 : 0;
}

/* scl_write/sda_write are only called by the driver for SMBus sleep/wake mode,
   which this application does not use. They must be non-null to pass the driver's
   null-check in mlx90614_init; the hardware I2C controller owns these pins. */
uint8_t mlx90614_interface_scl_write(uint8_t val) { return 0; }
uint8_t mlx90614_interface_sda_write(uint8_t val) { return 0; }

void mlx90614_interface_delay_ms(uint32_t ms)
{
    k_msleep(ms);
}

void mlx90614_interface_debug_print(const char *fmt, ...)
{
    (void)fmt;
}
