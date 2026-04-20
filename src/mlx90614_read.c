#include "mlx90614_read.h"
#include "driver_mlx90614.h"
#include "driver_mlx90614_interface.h"
#include <errno.h>

static mlx90614_handle_t gs_handle;

int mlx90614_sensor_init(void)
{
    DRIVER_MLX90614_LINK_INIT(&gs_handle, mlx90614_handle_t);
    DRIVER_MLX90614_LINK_IIC_INIT(&gs_handle, mlx90614_interface_iic_init);
    DRIVER_MLX90614_LINK_IIC_DEINIT(&gs_handle, mlx90614_interface_iic_deinit);
    DRIVER_MLX90614_LINK_IIC_READ(&gs_handle, mlx90614_interface_iic_read);
    DRIVER_MLX90614_LINK_IIC_WRITE(&gs_handle, mlx90614_interface_iic_write);
    DRIVER_MLX90614_LINK_SCL_WRITE(&gs_handle, mlx90614_interface_scl_write);
    DRIVER_MLX90614_LINK_SDA_WRITE(&gs_handle, mlx90614_interface_sda_write);
    DRIVER_MLX90614_LINK_DELAY_MS(&gs_handle, mlx90614_interface_delay_ms);
    DRIVER_MLX90614_LINK_DEBUG_PRINT(&gs_handle, mlx90614_interface_debug_print);

    /* Library uses 8-bit SMBus addresses; set default before init */
    mlx90614_set_addr(&gs_handle, MLX90614_ADDRESS_DEFAULT);

    return mlx90614_init(&gs_handle) ? -EIO : 0;
}

int mlx90614_read_temperature(float *ambient, float *object)
{
    uint16_t raw_ambient, raw_object;

    if (mlx90614_read_ambient(&gs_handle, &raw_ambient, ambient) != 0)
        return -EIO;
    if (mlx90614_read_object1(&gs_handle, &raw_object, object) != 0)
        return -EIO;

    return 0;
}

void mlx90614_sensor_deinit(void)
{
    mlx90614_deinit(&gs_handle);
}