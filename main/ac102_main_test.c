#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "AC102.h"
#include "ESP32_A2S.h"
#include "ac102_app.h"


void app_main()
{
	esp_err_t ret = AC102_DRV_INIT_44100HZ_16BIT_2CHANNEL();
	if (ESP_OK == ret) {
		printf("AC102_DRV_INIT_44100HZ_16BIT_2CHANNEL SUCCESS\n");
	} else {
		printf("AC102_DRV_INIT_44100HZ_16BIT_2CHANNEL FAIL\n");
		vTaskDelete(NULL);
	}

	ret = AC102_START();
	if (ESP_OK == ret) {
		printf("AC102_START IS OK\n");
	} else {
		printf("AC102_START IS FAIL\n");
		vTaskDelete(NULL);
	}

	AC102_TEST();

	vTaskDelete(NULL);
}

