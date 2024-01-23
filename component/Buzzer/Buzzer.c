#include <sys/types.h> // uint8_t...
#include "driver/ledc.h" // thu vien pwm ledcontroller
#include "esp_err.h" // check loi gia tri esp_err_t
#include "Buzzer.h"

static const char *TAG = "Buzzer";

static ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num  = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = CONFIG_BUZZER_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
};

static ledc_channel_config_t ledc_channel;

void Buzzer_init(uint8_t BUZZER_PIN)
{
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.gpio_num = BUZZER_PIN;

    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
        
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

}

void Buzzer_set_duty(uint8_t level)
{
    uint32_t buzzer_duty = 8191 * level / 255;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, buzzer_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}