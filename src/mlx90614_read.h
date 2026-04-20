#pragma once

int mlx90614_sensor_init(void);
int mlx90614_read_temperature(float *ambient, float *object);
void mlx90614_sensor_deinit(void);