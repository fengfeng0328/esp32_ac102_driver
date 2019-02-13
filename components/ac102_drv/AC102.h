/*
 * AC102.h
 *
 *  Created on: 2018年10月26日
 *      Author: mac
 */

#ifndef COMPONENTS_AUDIO_HAL_DRIVER_AC102_AC102_H_
#define COMPONENTS_AUDIO_HAL_DRIVER_AC102_AC102_H_
#define AC102_ADDR_LOW			0x33				/*!< Device address,DEVID LOW*/
#define AC102_ADDR_HIGH			0x30				/*!< Device address,DEVID HIGH*/
#define WRITE_BIT  			I2C_MASTER_WRITE 	/*!< I2C master write */
#define READ_BIT   			I2C_MASTER_READ  	/*!< I2C master read */
#define ACK_CHECK_EN   		0x1     			/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  		0x0     			/*!< I2C master will not check ack from slave */
#define ACK_VAL    			0x0         		/*!< I2C ack value */
#define NACK_VAL   			0x1         		/*!< I2C nack value */

#define CHIP_SOFT_RST		0x00//CHIP_SOFT_RST
#define PWR_CTRL1 			0x01//Power Control 1 Register
#define PWR_CTRL2			0x02//Power Control 2 Register
#define SYS_FUNC_CTRL		0x03//System Function Control Register
#define ADC_CLK_SET			0x04//ADC Clock Setting Register
#define DAC_CLK_SET			0x05//DAC Clock Setting Register
#define SYS_CLK_ENA			0x06//System Clock Enable Register
#define I2S_CTRL			0x07//I2S Control Register
#define I2S_BCLK_CTRL		0x08//BCLK Control Register
#define I2S_LRCK_CTRL1 		0x09//I2S LRCK Control 1 Register
#define I2S_LRCK_CTRL2 		0x0A//I2S LRCK Control 2 Register
#define I2S_FMT_CTRL1		0x0B//I2S Format Control 1 Register
#define I2S_FMT_CTRL2 		0x0C//I2S Format Control 2 Register
#define I2S_FMT_CTRL3		0x0D//I2S Format Control 3 Register
#define I2S_SLOT_CTRL 		0xOE//I2S Slot Control Register
#define I2S_TX_CTRL			0x0F//I2S TX Control Register
#define I2S_TX_CHMP_CTRL    0x11//I2S TX Channel Mapping Control Register
#define I2S_TX_MIX_SRC		0x13//I2S TX Mixer Source Select Register
#define I2S_RX_CHMP_CTRL 	0x16//I2S RX Channel Mapping Control Register
#define I2S_RX_MIX_SRC 		0x18//I2S RX Mixer Source Select Register
#define ADC_DIG_CTRL		0x19//ADC Digital Control Register
#define ADC_DVC				0x1A//ADC Digital Volume Register
#define DAC_DVC 			0x1C//DAC Digital Volume Register
#define DAC_MIX_SRC			0x1D//DAC Mixer Source Select Register
#define DIG_PADDRV_CTRL		0x1F//Digital Pad Drive Control Register
#define ADC_ANA_CTRL1		0x20//ADC Analog Control 1 Register
#define ADC_ANA_CTRL2		0x21//ADC Analog Control 2 Register
#define ADC_ANA_CTRL3		0x22//ADC Analog Control 3 Register
#define ADC_ANA_CTRL4		0x23//ADC Analog Control 4 Register
#define ADC_ANA_CTRL5		0x24//ADC Analog Control 5 Register
#define DAC_ANA_CTRL1		0x25//DAC Analog Control 1 Register
#define DAC_ANA_CTRL2		0x26//DAC Analog Control 2 Register
#define DAC_ANA_CTRL3		0x27//DAC Analog Control 3 Register
#define DAC_ANA_CTRL4		0x28//DAC Analog Control 4 Register
#define AGC_STA 			0x30//AGC Status Register
#define AGC_CTRL 			0x31//AGC Control Register
#define AGC_DEBT 			0x32//AGC Debounce time Register
#define AGC_TGLVL 			0x33//AGC Target Level Register
#define AGC_MAXG 			0x34//AGC Max Gain Register
#define AGC_AVGC1 			0x35//AGC Average Coef 1 Register
#define AGC_AVGC2 			0x36//AGC Average Coef 2 Register
#define AGC_AVGC3 			0x37//AGC Average Coef 3 Register
#define AGC_AVGC4 			0x38//AGC Average Coef 4 Register
#define AGC_DECAYT1 		0x39//AGC Decay Time 1 Register
#define AGC_DECAYT2  		0x3A//AGC Decay Time 2 Register
#define AGC_ATTACKT1 		0x3B//AGC Attack Time 1 Register
#define AGC_ATTACKT2 		0x3C//AGC Attack Time 2 Register
#define AGC_NTH  			0x3D//AGC Noise Threshold Register
#define AGC_NAVGC1 			0x3E//AGC Noise Average Coef 1 Register
#define AGC_NAVGC2 			0x3F//AGC Noise Average Coef 2 Register
#define AGC_NAVGC3   		0x40//AGC Noise Average Coef 3 Register
#define AGC_NAVGC4     		0x41//AGC Noise Average Coef 4 Register
#define HPF_COEF1 			0x42//HPF Coef 1 Register
#define HPF_COEF2 			0x43//HPF Coef 2 Register
#define HPF_COEF3   		0x44//HPF Coef 3 Register
#define HPF_COEF4     		0x45//HPF Coef 4 Register
#define AGC_OPT				0x46//AGC Optimum Register
#define EQ_CTRL 			0x4F//EQ Control Register

typedef enum {
    AC102_BIT_LENGTH_8_BITS = 0x01,
	AC102_BIT_LENGTH_12_BITS = 0x02,
	AC102_BIT_LENGTH_16_BITS = 0x03,
	AC102_BIT_LENGTH_20_BITS = 0x04,
	AC102_BIT_LENGTH_24_BITS = 0x05,
	AC102_BIT_LENGTH_28_BITS = 0x06,
	AC102_BIT_LENGTH_32_BITS = 0x07,

} ac102_bits_length_t;

typedef enum {
    AC102_MODULE_MIN = -1,
    AC102_MODULE_ADC = 0x01,
    AC102_MODULE_DAC = 0x02,
    AC102_MODULE_ADC_DAC = 0x03,
    AC102_MODULE_MAX
} ac102_module_t;

typedef enum{
	SRC_MIC	= 1,
}ac102_output_mixer_source_t;

int AC102_init(void);
//int AC102_get_spk_volume(void) ;
//int AC102_set_spk_volume(uint8_t volume) ;
//int AC102_start(ac102_module_t mode) ;
//int AC102_stop(ac102_module_t mode);
//int AC102_deinit(void) ;
//int AC102_config_i2s(audio_hal_codec_mode_t mode,
//		audio_hal_codec_i2s_iface_t* iface);
//int AC102_ctrl_state(audio_hal_codec_mode_t mode,
//		audio_hal_ctrl_t ctrl_state);
//int AC102_set_voice_volume(int volume) ;
//int AC102_get_voice_volume(int* volume) ;
//void AC102_pa_power(bool enable);
#endif /* COMPONENTS_AUDIO_HAL_DRIVER_AC102_AC102_H_ */
