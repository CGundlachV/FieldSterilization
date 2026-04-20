#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/random/random.h>
#include "mlx90614_read.h"

#ifdef CONFIG_DISPLAY
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <lvgl_mem.h>
#include <lvgl_zephyr.h>
#include <zephyr/sys/heap_listener.h>
#include <zephyr/sys/mem_manage.h>
#include <zephyr/sys/sys_heap.h>
#endif

#define TIMER_MAX_S     600
#define TEMP_MIN_C      50.0f
#define TEMP_MID_C      75.0f
#define TEMP_MAX_C      100.0f

#ifdef CONFIG_DISPLAY
#define ARC_ANGLE_START  135
#define ARC_ANGLE_RANGE  270
#endif

static double temp = 150.0;

void log_free_heap(const char *label)
{
    size_t sizes[] = {65536, 32768, 16384, 8192, 4096, 2048, 1024};
    for (int i = 0; i < ARRAY_SIZE(sizes); i++) {
        void *p = k_malloc(sizes[i]);
        if (p != NULL) {
            LOG_INF("[%s] Free heap >= %zu bytes", label, sizes[i]);
            k_free(p);
            return;
        }
    }
    LOG_WRN("[%s] Free heap < 1024 bytes", label);
}

static float next_temperature(void)
{
    int32_t delta = (int32_t)(sys_rand32_get() % 51) - 25;
    temp += (double)delta / 10;
    if (temp < 95.0)  temp = 95.0;
    if (temp > 205.0) temp = 205.0;
    return temp;
}

#ifdef CONFIG_DISPLAY

typedef struct { uint8_t r; uint8_t g; uint8_t b; } rgb8_t;

static lv_color_t last_col;

static void lv_tick_cb(struct k_timer *t) { lv_tick_inc(1); }
K_TIMER_DEFINE(lv_tick_timer, lv_tick_cb, NULL);

static rgb8_t rgb8_lerp(rgb8_t a, rgb8_t b, uint8_t t)
{
    return (rgb8_t){
        .r = (uint8_t)((a.r * (255 - t) + b.r * t) / 255),
        .g = (uint8_t)((a.g * (255 - t) + b.g * t) / 255),
        .b = (uint8_t)((a.b * (255 - t) + b.b * t) / 255),
    };
}

static lv_color_t temp_to_color(float temp_c)
{
    static const rgb8_t BLUE  = {0x00, 0x00, 0xFF};
    static const rgb8_t GREEN = {0x00, 0xFF, 0x00};
    static const rgb8_t RED   = {0xFF, 0x00, 0x00};
    rgb8_t result;

    if (temp_c <= TEMP_MIN_C) {
        result = BLUE;
    } else if (temp_c >= TEMP_MAX_C) {
        result = RED;
    } else if (temp_c <= TEMP_MID_C) {
        uint8_t t = (uint8_t)((temp_c - TEMP_MIN_C) / (TEMP_MID_C - TEMP_MIN_C) * 255.0f);
        result = rgb8_lerp(BLUE, GREEN, t);
    } else {
        uint8_t t = (uint8_t)((temp_c - TEMP_MID_C) / (TEMP_MAX_C - TEMP_MID_C) * 255.0f);
        result = rgb8_lerp(GREEN, RED, t);
    }
    return lv_color_make(result.r, result.g, result.b);
}

static lv_obj_t *arc_indicator;
static lv_obj_t *countdown_label;
static lv_obj_t *temp_label;
static lv_style_t style_arc_indicator;
static lv_style_t style_arc_bg;
static lv_style_t style_countdown;
static lv_style_t style_temp;

static void ui_create(lv_obj_t *screen, lv_coord_t w, lv_coord_t h)
{
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0A0A0A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    lv_style_init(&style_arc_bg);
    lv_style_set_arc_color(&style_arc_bg, lv_color_hex(0x222222));
    lv_style_set_arc_width(&style_arc_bg, 14);

    lv_style_init(&style_arc_indicator);
    lv_style_set_arc_color(&style_arc_indicator, lv_color_hex(0x00CFFF));
    lv_style_set_arc_width(&style_arc_indicator, 14);
    lv_style_set_arc_rounded(&style_arc_indicator, true);

    arc_indicator = lv_arc_create(screen);
    lv_obj_set_size(arc_indicator, w - 20, h - 20);
    lv_obj_center(arc_indicator);
    lv_obj_remove_style(arc_indicator, NULL, LV_PART_KNOB);
    lv_obj_set_style_pad_all(arc_indicator, 0, LV_PART_KNOB);
    lv_obj_add_style(arc_indicator, &style_arc_bg, LV_PART_MAIN);
    lv_obj_add_style(arc_indicator, &style_arc_indicator, LV_PART_INDICATOR);
    lv_arc_set_bg_angles(arc_indicator, ARC_ANGLE_START, ARC_ANGLE_START + ARC_ANGLE_RANGE);
    lv_arc_set_angles(arc_indicator, ARC_ANGLE_START, ARC_ANGLE_START);
    lv_arc_set_mode(arc_indicator, LV_ARC_MODE_NORMAL);
    lv_arc_set_range(arc_indicator, 0, TIMER_MAX_S);
    lv_arc_set_value(arc_indicator, 0);
    lv_obj_clear_flag(arc_indicator, LV_OBJ_FLAG_CLICKABLE);

    lv_style_init(&style_countdown);
    lv_style_set_text_color(&style_countdown, lv_color_white());
    lv_style_set_text_font(&style_countdown, &lv_font_montserrat_28);
    countdown_label = lv_label_create(screen);
    lv_obj_add_style(countdown_label, &style_countdown, LV_PART_MAIN);
    lv_label_set_text(countdown_label, "10:00");
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 30);

    lv_style_init(&style_temp);
    lv_style_set_text_color(&style_temp, lv_color_hex(0x0080FF));
    lv_style_set_text_font(&style_temp, &lv_font_montserrat_36);
    temp_label = lv_label_create(screen);
    lv_obj_add_style(temp_label, &style_temp, LV_PART_MAIN);
    lv_label_set_text(temp_label, "150C");
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, -15);
}

static void ui_update(uint32_t elapsed_s, double temp_c)
{
    lv_arc_set_value(arc_indicator, (int32_t)elapsed_s);
    uint32_t remaining = TIMER_MAX_S - elapsed_s;
    lv_label_set_text_fmt(countdown_label, "%02u:%02u", remaining / 60, remaining % 60);
    lv_label_set_text_fmt(temp_label, "%2.1lfC", temp_c);
    lv_color_t col = temp_to_color(temp_c);
    if (lv_color_to_u32(col) != lv_color_to_u32(last_col)) {
        lv_obj_set_style_text_color(temp_label, col, LV_PART_MAIN);
        last_col = col;
    }
}

#endif /* CONFIG_DISPLAY */

static void i2c_scan(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c21), okay)
    const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c21));
#else
    const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#endif
    if (!device_is_ready(i2c)) { printk("I2C not ready\n"); return; }
    printk("I2C scan (0x08..0x77):\n");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        if (i2c_read(i2c, &dummy, 1, addr) == 0) {
            printk("  found 0x%02x\n", addr);
            found++;
        }
    }
    if (!found) printk("  no devices found\n");
}

int main(void)
{
    k_msleep(500);
    i2c_scan();

#ifdef CONFIG_DISPLAY
    log_free_heap("Boot");

    const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 1;
    }
    log_free_heap("After display init");

    k_timer_start(&lv_tick_timer, K_MSEC(1), K_MSEC(1));
    lv_init();
    log_free_heap("After lv_init");

    display_blanking_off(display_dev);
    display_set_brightness(display_dev, 255);

    lv_coord_t scr_w, scr_h;
#if LVGL_VERSION_MAJOR >= 9
    lv_display_t *disp = lv_display_get_default();
    scr_w = lv_display_get_horizontal_resolution(disp);
    scr_h = lv_display_get_vertical_resolution(disp);
#else
    lv_disp_t *disp = lv_disp_get_default();
    lv_disp_set_bg_opa(disp, LV_OPA_COVER);
    scr_w = lv_disp_get_hor_res(disp);
    scr_h = lv_disp_get_ver_res(disp);
#endif

    bool mlx_ok = (mlx90614_sensor_init() == 0);
    if (!mlx_ok) {
        LOG_WRN("MLX90614 init failed — will show sensor error");
    }

    float ambient, object;
    LOG_INF("Display: %dx%d", scr_w, scr_h);
    ui_create(lv_scr_act(), scr_w, scr_h);
    log_free_heap("After UI create");

    uint32_t elapsed_s = 0;

    while (1) {
        if (mlx_ok && mlx90614_read_temperature(&ambient, &object) == 0) {
            temp = object;
        } else {
            if (mlx_ok) LOG_WRN("MLX90614 read error");
            temp = next_temperature();
        }
        ui_update(elapsed_s, temp);

        if (elapsed_s < TIMER_MAX_S) elapsed_s++; else elapsed_s = 0;

        lv_timer_handler();
        k_msleep(1000);
    }

#else /* !CONFIG_DISPLAY */

    bool mlx_ok = (mlx90614_sensor_init() == 0);
    if (!mlx_ok) {
        LOG_WRN("MLX90614 init failed");
    }

    float ambient, object;

    while (1) {
        if (mlx_ok && mlx90614_read_temperature(&ambient, &object) == 0) {
            LOG_INF("Ambient: %.1f C  Object: %.1f C", (double)ambient, (double)object);
        } else {
            if (mlx_ok) LOG_WRN("MLX90614 read error");
        }
        k_msleep(1000);
    }

#endif /* CONFIG_DISPLAY */

    return 0;
}
