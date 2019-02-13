/*
 * AC102.c
 *
 *  Created on: 2018年10月26日
 *      Author: mac
 */

#include "AC102.h"
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "ESP32_A2S.h"
#define AC102_TAG  "AC102_DRIVER"
#define IIC_PORT	I2C_NUM_1

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(AC102_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static const i2c_config_t es_i2c_cfg = { .mode = I2C_MODE_MASTER, .sda_io_num =
IIC_DATA, .scl_io_num = IIC_CLK, .sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = 100000 };

int AC102_ADDR = 0;
static int i2c_init() {
	int res;
	res = i2c_param_config(IIC_PORT, &es_i2c_cfg);
	res |= i2c_driver_install(IIC_PORT, es_i2c_cfg.mode, 0, 0, 0);
	AC_ASSERT(res, "i2c_init error", -1);
	return res;
}

static int i2c_example_master_read_slave(uint8_t DevAddr, uint8_t reg,
		uint8_t* data_rd, size_t size) {
	if (size == 0) {
		return ESP_OK;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DevAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DevAddr << 1) | READ_BIT, ACK_CHECK_EN); //check or not
	i2c_master_read(cmd, data_rd, size, ACK_VAL);
	i2c_master_stop(cmd);
	int ret = i2c_master_cmd_begin(IIC_PORT, cmd,
			1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static int AC102_Write_Reg(uint8_t reg, uint16_t val) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	int ret = 0;
	uint8_t send_buff[4];
	send_buff[0] = (AC102_ADDR << 1);
	send_buff[1] = reg;
	send_buff[2] = (val) & 0xff;
	ret |= i2c_master_start(cmd);
	ret |= i2c_master_write(cmd, send_buff, 3, ACK_CHECK_EN);
	ret |= i2c_master_stop(cmd);
	ret |= i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

static uint8_t AC102_read_Reg(uint8_t reg) {
	uint8_t val = 0;
	uint8_t data_rd=0;
	i2c_example_master_read_slave(AC102_ADDR, reg, &data_rd, 1);
	val = data_rd;
	return val;
}

int AC102_init(void) {
	if (DEVID_EN == 1) {
		AC102_ADDR = AC102_ADDR_HIGH;
	} else {
		AC102_ADDR = AC102_ADDR_LOW;
	}
	if (i2c_init())
		return -1;
	int res;
	res = AC102_Write_Reg(CHIP_SOFT_RST, 0x34);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	if (ESP_OK != res) {
		ESP_LOGE(AC102_TAG, "reset failed!");
		return res;
	} else {
		ESP_LOGE(AC102_TAG, "reset succeed");
	}
//	res |= AC102_Write_Reg(PWR_CTRL1, 0x73);
	//Clocking system
//	res |= AC102_Write_Reg(ADC_CLK_SET, 0x1b);
	res |= AC102_Write_Reg(DAC_CLK_SET, 0x01);
	res |= AC102_Write_Reg(SYS_CLK_ENA, 0x3f);
//	res |= AC102_Write_Reg(I2S_CTRL, 0x05);
//	res |= AC102_Write_Reg(I2S_BCLK_CTRL, 0x3f);
//	res |= AC102_Write_Reg(I2S_LRCK_CTRL1, 0x80);			//sample rate
//	//I2S format config
	res |= AC102_Write_Reg(I2S_LRCK_CTRL1, 0x00);			//BCLK/LRCK
	res |= AC102_Write_Reg(I2S_LRCK_CTRL2, 0x1f);		//
	res |= AC102_Write_Reg(I2S_LRCK_CTRL2, 0x0f);
	res |= AC102_Write_Reg(DAC_ANA_CTRL2, 0x00);
//	res |= AC102_Write_Reg(I2S_FMT_CTRL2, 0x33);
//	res |= AC102_Write_Reg(I2S_FMT_CTRL3, 0x00);			//
//
//	res |= AC102_Write_Reg(I2S_SLOT_CTRL,0x05);
//	res |= AC102_Write_Reg(I2S_TX_CTRL, 0x03);
//	res |= AC102_Write_Reg(I2S_TX_CHMP_CTRL, 0x0a);
//	res |= AC102_Write_Reg(I2S_TX_CHMP_CTRL, 0x05);
//
//	//Path Configuration
//	res |= AC102_Write_Reg(I2S_RX_CHMP_CTRL, 0x05);
//	res |= AC102_Write_Reg(I2S_RX_MIX_SRC, 0x0f);
//	res |= AC102_Write_Reg(ADC_DIG_CTRL, 0x0a);
	res |= AC102_Write_Reg(DAC_DVC, 0x81);
	res |= AC102_Write_Reg(ADC_ANA_CTRL1, 0x60);
	res |= AC102_Write_Reg(DAC_ANA_CTRL2, 0x08);
	/***********************录音****************************/
	res |= AC102_Write_Reg(I2S_TX_MIX_SRC, 0x0f);
	res |= AC102_Write_Reg(DAC_ANA_CTRL2, 0x0);
//	res |= AC102_Write_Reg(DAC_MIX_SRC, 0x01);//不监听

	res |= AC102_Write_Reg(ADC_ANA_CTRL1, 0x00);
	/************************/
//	res |= AC102_Write_Reg(DAC_MIX_SRC, 0x03);//监听
	res |= AC102_Write_Reg(I2S_TX_MIX_SRC, 0x05);//监听
	res |= AC102_Write_Reg(DAC_MIX_SRC, 0x01);//监听
	//* Enable Speaker output
	for(int i=0;i<0x4f;i++)
	{
		printf("res:%x,value=%x\r\n",i,AC102_read_Reg(i));
	}
	return res;
}
int AC102_get_spk_volume(void) {
	int res;
	res = AC102_read_Reg(DAC_DVC);
	return res;
}

int AC102_set_spk_volume(uint8_t volume) {
	int ret;
	ret = AC102_Write_Reg(DAC_DVC, volume);
	return ret;
}


//
//int AC102_set_output_mixer_gain(ac_output_mixer_gain_t gain,
//		ac_output_mixer_source_t source) {
//	uint16_t regval;
//	int ret;
//	regval = AC102_read_Reg(OMIXER_BST1_CTRL);
//	switch (source) {
//	case SRC_MIC:
//		temp = (gain & 0x7) << 6;
//		clrbit = ~(0x7 << 6);
//		break;
//	default:
//		return -1;
//	}
//	return ret;
//}

int AC102_deinit(void) {

	return AC102_Write_Reg(CHIP_SOFT_RST, 0x34);		//soft reset
}



int AC102_set_voice_volume(int volume) {
	int res;
	res = AC102_set_spk_volume(volume);
	return res;
}

int AC102_get_voice_volume(int* volume) {
	*volume = AC102_get_spk_volume();
	return 0;
}


