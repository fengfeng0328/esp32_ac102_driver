#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "ac102_app.h"
#include "driver/i2s.h"
#include "AC102.h"
#include "ESP32_A2S.h"

esp_err_t AC102_DRV_INIT_44100HZ_16BIT_2CHANNEL() {
	AC102_init();
	i2s_config_t i2s_config = {
			.mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,
			.sample_rate = 44100,
			.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
			.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,           //2-channels
			.communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
			.dma_buf_count = 32,
			.dma_buf_len = 32 * 2,
			.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 //Interrupt level 1
	};

	i2s_pin_config_t pin_config_rx = {
			.bck_io_num = IIS_SCLK,
			.ws_io_num = IIS_LCLK,
			.data_out_num = IIS_DSIN,
			.data_in_num = IIS_DOUT
	};

	int reg_val = REG_READ(PIN_CTRL);
	REG_WRITE(PIN_CTRL, 0xFFFFFFF0);
	reg_val = REG_READ(PIN_CTRL);
	PIN_FUNC_SELECT(GPIO_PIN_REG_0, 1); //GPIO0 as CLK_OUT1

	/* 注册i2s设备驱动 */
	esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
	if (ESP_OK != ret) {
		return ret;
	}
	/* 设置i2s引脚 */
	ret = i2s_set_pin(I2S_NUM_0, &pin_config_rx);
	if (ESP_OK != ret) {
		return ret;
	}

	return ESP_OK;
}

esp_err_t AC102_START() {
	esp_err_t ret = i2s_start(I2S_NUM_0);
	if(ESP_OK == ret){
		return ESP_OK;
	}else{
		return ret;
	}
}

esp_err_t AC102_TEST() {
	for (;;) {
		char buf_t[2048] = { 0 };
		int len = i2s_read_bytes(I2S_NUM_0, buf_t, sizeof(buf_t),
		portMAX_DELAY);
		if (len == ESP_FAIL) {
			goto exit;
		}
		i2s_write_bytes(I2S_NUM_0, buf_t, len, portMAX_DELAY);
		if (len == ESP_FAIL) {
			goto exit;
		}
	}

	exit:
	printf("AC102_TEST TEST ERROR!\n");
	return ESP_FAIL;
}
