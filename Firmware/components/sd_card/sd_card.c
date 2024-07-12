#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_card.h"

#define MOUNT_POINT "/sdcard"
#define SDMMC_MAX_FREQ_KHZ 80000

static const char *TAG = "SDMMC";
sdmmc_card_t *card;

// Initialise SDMMC
esp_err_t sd_card_init(gpio_num_t clk, gpio_num_t cmd, gpio_num_t d0, gpio_num_t d1, gpio_num_t d2, gpio_num_t d3)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    ESP_LOGI(TAG, "Initializing SD card");

    // Config SDMMC Host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_MAX_FREQ_KHZ;

    // Config SDMMC Slot
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = clk;
    slot_config.cmd = cmd;
    slot_config.d0 = d0;
    slot_config.d1 = d1;
    slot_config.d2 = d2;
    slot_config.d3 = d3;

    ESP_LOGI(TAG, "Mounting SD Card");

    // Mount SD card
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    // If mounting is not successful
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
}

// De-initialise sd card
esp_err_t sd_card_deinit(void)
{
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to de-initialize the card (%s).", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI(TAG, "Card unmounted");
    }
    return ESP_OK;
}

// Get capacity information of the sd card
sd_capacity_t get_sd_capacity_info(void)
{
    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect, used_sect;
    sd_capacity_t capacity_info;

    // Get Volume information and free clusters from drive 0 (SD)
    FRESULT res = f_getfree("0:", &fre_clust, &fs);

    // Set the result_available flag to false if f_getfree failed
    if (res != FR_OK)
    {
        ESP_LOGE(TAG, "Failed to get SD card capacity info");
        capacity_info.result_available = false;
        return capacity_info;
    }
    
    // Get total sectors and free sectors
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    used_sect = tot_sect - fre_sect;

    capacity_info.used_MB = (float)used_sect / 2048;
    capacity_info.used_GB = (float)used_sect / 2097152;
    capacity_info.total_MB = (float)tot_sect / 2048;
    capacity_info.total_GB = (float)tot_sect / 2097152;

    capacity_info.result_available = true;

    ESP_LOGI(TAG, "SD Card Capacity: %.3f GB used, %.3f GB total", capacity_info.used_GB, capacity_info.total_GB);

    return capacity_info;
}
