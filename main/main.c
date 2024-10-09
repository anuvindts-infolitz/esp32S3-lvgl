#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/spi_master.h"

#include "espressif__esp_lcd_axs15231b/include/esp_lcd_axs15231b.h"
#include "espressif__esp_lcd_touch/include/esp_lcd_touch.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"

#include "lvgl__lvgl/lvgl.h" 

static const char *TAG = "lcd_panel";

#define EXAMPLE_PIN_NUM_LCD_RST 9
#define EXAMPLE_PIN_NUM_LCD_PCLK 10
#define EXAMPLE_PIN_NUM_LCD_DATA0 11
#define EXAMPLE_PIN_NUM_LCD_CS 12
#define EXAMPLE_LCD_H_RES 13
#define EXAMPLE_LCD_V_RES 15
#define EXAMPLE_PIN_NUM_LCD_DC 14

#define EXAMPLE_LCD_HOST 10

#define CONFIG_LCD_HRES 100
#define CONFIG_LCD_VRES 200


static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;
esp_lcd_panel_handle_t panel_handle;

bool callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx){
    int a = 0;
    return false;
}

int callback_data = 10;

esp_lcd_touch_handle_t tp;

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint16_t touch_strength[1];
    uint8_t touch_cnt = 0;

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);

    uint16_t pointX = touch_x[0];
    uint16_t pointY = touch_y[0];
    uint16_t type = touchpad_pressed;

    if (type && (pointX || pointY)) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = pointY;
        data->point.y = pointX;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

void lvgl_task(void *pvParameter) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, (void *)color_p);
    lv_disp_flush_ready(disp_drv);
}

// LVGL initialization
void lvgl_init() {
    lv_init();

    size_t buffer_size = sizeof(lv_color_t) * EXAMPLE_LCD_H_RES * CONFIG_LCD_VRES;
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);

    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers");
        return;
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, EXAMPLE_LCD_H_RES * CONFIG_LCD_VRES);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

LV_IMG_DECLARE(test_img);
LV_IMG_DECLARE(test1_180640);
LV_IMG_DECLARE(test3_180640);
LV_IMG_DECLARE(test4_180640);

static const lv_img_dsc_t *imgs[4] = {&test1_180640, &test3_180640, &test4_180640, &test_img};

// Demo animation function
void demo_init(lv_obj_t *parent, const lv_img_dsc_t **dsc, uint8_t num, uint32_t duration, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h) {
    lv_obj_t *animimg0 = lv_animimg_create(parent);
    lv_obj_center(animimg0);
    lv_animimg_set_src(animimg0, dsc, num);

    lv_animimg_set_duration(animimg0, duration);
    lv_animimg_set_repeat_count(animimg0, LV_ANIM_REPEAT_INFINITE);

    lv_obj_set_size(animimg0, w, h);
    lv_animimg_start(animimg0);
}

void demo_animation(void) {
    lv_obj_t *obj = lv_btn_create(lv_scr_act());
    lv_obj_set_size(obj, 180, 640);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    demo_init(obj, imgs, 4, 12000, 0, 0, 180, 640);
}


void app_main(void)
{

    // Init LCD
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t bus_config = AXS15231B_PANEL_BUS_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                                    EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                    EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = AXS15231B_PANEL_IO_SPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_DC,
                                                                                callback, &callback_data);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_HOST, &io_config, &io_handle));

        ESP_LOGI(TAG, "Install AXS15231B panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const axs15231b_vendor_config_t vendor_config = {
        // .init_cmds = lcd_init_cmds,         // Uncomment these line if use custom initialization commands
        // .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(axs15231b_lcd_init_cmd_t),
        .flags = {
            .use_qspi_interface = 0,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
        .bits_per_pixel = 16,                           // Implemented by LCD command `3Ah` (16/18)
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // Touch init
    esp_lcd_panel_io_i2c_config_t io = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_HRES,
        .y_max = CONFIG_LCD_VRES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    esp_lcd_touch_new_i2c_axs15231b(io, &tp_cfg, &tp);

    esp_lcd_touch_read_data(tp);

    lvgl_init();

    // Start LVGL task
    xTaskCreate(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL);

    // Create label
    lv_obj_t *ui_label = lv_label_create(lv_scr_act());
    lv_label_set_text(ui_label, "(0, 0)");
    lv_obj_align(ui_label, LV_ALIGN_CENTER, 0, 0);

    // Start demo animation
    demo_animation();


}
