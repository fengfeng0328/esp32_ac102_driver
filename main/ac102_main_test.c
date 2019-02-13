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
	AC102_DRV_INIT_44100HZ_16BIT_2CHANNEL();
	AC102_START();
	AC102_TEST();
}

