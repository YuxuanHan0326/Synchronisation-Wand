#ifndef SD_CARD_H
#define SD_CARD_H

typedef struct sd_capacity
{
    bool result_available;
    float used_MB;
    float used_GB;
    float total_MB;
    float total_GB;
} sd_capacity_t;

esp_err_t sd_card_init(gpio_num_t clk, gpio_num_t cmd, gpio_num_t d0, gpio_num_t d1, gpio_num_t d2, gpio_num_t d3);
esp_err_t sd_card_deinit(void);
sd_capacity_t get_sd_capacity_info(void);

#endif