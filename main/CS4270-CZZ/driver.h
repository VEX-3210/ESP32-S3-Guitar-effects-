
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2s_std.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

//**************************************************//
//               I2C Confoguraction                 //
//**************************************************//

#define I2C_PORT 0
//#define I2C_MASTER_SCL  GPIO_NUM_13
//#define I2C_MASTER_SDA  GPIO_NUM_12

#define I2C_MASTER_SCL  GPIO_NUM_11
#define I2C_MASTER_SDA  GPIO_NUM_10

#define I2C_INTERNAL_PULLUP_ENABLE false

#define I2C_FREQUENCY_HZ 10000 

#define CODEC_ADDRESS_LINES 0b000 //AD2 AD1 AD0

//**************************************************//
//               I2S Confoguraction                 //
//**************************************************//

/*  Pin Confoguraction  */
/*#define NRST_PIN        GPIO_NUM_4

#define I2S_DIN_PIN     GPIO_NUM_11
#define I2S_SCLK_PIN    GPIO_NUM_10
#define I2S_MCLK_PIN    GPIO_NUM_9
#define I2S_WS_PIN      GPIO_NUM_6
#define I2S_DOUT_PIN    GPIO_NUM_5*/

#define NRST_PIN        GPIO_NUM_12

#define I2S_DIN_PIN     GPIO_NUM_9
#define I2S_SCLK_PIN    GPIO_NUM_6
#define I2S_MCLK_PIN    GPIO_NUM_5
#define I2S_WS_PIN      GPIO_NUM_4
#define I2S_DOUT_PIN    GPIO_NUM_3


#define I2S_SAMPLIG_RATE_HZ 48000
#define I2S_MCLK_MULTIPLE I2S_MCLK_MULTIPLE_256
#define SCLK_PER_WS 64                            // SCLK/LRCK Taken From Datasheet

#define I2S_DMA_BUFFER_COUNT        2
#define I2S_DMA_BUFFER_FRAME_SIZE   256     //dma_buffer_size = dma_frame_num * slot_num * slot_bit_width / 8 (Amount Of Left/Right Samples That Will feet One Buffer)

#define I2S_BUFFER_SIZE  I2S_DMA_BUFFER_FRAME_SIZE*I2S_DMA_BUFFER_COUNT

typedef struct{
    i2c_master_dev_handle_t I2C_device;
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;

    volatile bool data_ready;
    //i2s_event_data_t *input_buffer_info;

    int InData[I2S_BUFFER_SIZE];
    int OutData[I2S_BUFFER_SIZE];
    
} Codec_CS4270;


//**************************************************//
//      Memory Addres Pointer (MAP) Definitions     //
//**************************************************//

#define CODEC_MAP0_MASK (uint8_t) 0b00000001  //Bit 0 of Memory Addres Pointer (MAP)
#define CODEC_MAP1_MASK (uint8_t) 0b00000010  //Bit 1 of Memory Addres Pointer (MAP)
#define CODEC_MAP2_MASK (uint8_t) 0b00000100  //Bit 2 of Memory Addres Pointer (MAP)
#define CODEC_MAP3_MASK (uint8_t) 0b00001000  //Bit 3 of Memory Addres Pointer (MAP)
/*Bits 4, 5, 6 are Reserved and should  be set to 0*/
#define CODEC_MAP_INCR  (uint8_t) 0b10000000  //Auto Increment MAP (It's 7'th Bit of MAP)

//**********************************//
//      Registers Definitions       //
//**********************************//

/*  Register Addresses  */
#define CODEC_REG_ADDR_ID                   (uint8_t) 0x01    //Address of the ID register
#define CODEC_REG_ADDR_POWER_CONTROL        (uint8_t) 0x02    //Address of the Power Control register
#define CODEC_REG_ADDR_MODE_CONTROL         (uint8_t) 0x03    //Address of the Mode register
#define CODEC_REG_ADDR_ADC_DAC_CONTROL      (uint8_t) 0x04    //Address of the ADC and DAC Control register
#define CODEC_REG_ADDR_TRANSITION_CONTROL   (uint8_t) 0x05    //Address of the Transition Control register
#define CODEC_REG_ADDR_MUTE_CONTROL         (uint8_t) 0x06    //Address of the Mute Control register
#define CODEC_REG_ADDR_CH_A_VOL_CONTROL     (uint8_t) 0x07    //Address of the Channel A Volume Control register
#define CODEC_REG_ADDR_CH_B_VOL_CONTROL     (uint8_t) 0x08    //Address of the Channel B Volume Control register


/*  Registers Default Values  */
#define CODEC_REG_DEFAULT_ID                   (uint8_t) 0b11000011     //Address of the ID register
#define CODEC_REG_DEFAULT_POWER_CONTROL        (uint8_t) 0              //Address of the Power Control register
#define CODEC_REG_DEFAULT_MODE_CONTROL         (uint8_t) 0b00110000     //Address of the Mode register
#define CODEC_REG_DEFAULT_ADC_DAC_CONTROL      (uint8_t) 0              //Address of the ADC and DAC Control register
#define CODEC_REG_DEFAULT_TRANSITION_CONTROL   (uint8_t) 0b01100000     //Address of the Transition Control register
#define CODEC_REG_DEFAULT_MUTE_CONTROL         (uint8_t) 0b00100000     //Address of the Mute Control register
#define CODEC_REG_DEFAULT_CH_A_VOL_CONTROL     (uint8_t) 0              //Address of the Channel A Volume Control register
#define CODEC_REG_DEFAULT_CH_B_VOL_CONTROL     (uint8_t) 0              //Address of the Channel B Volume Control register


/*  ID Register Masks  */
//They are not needed because they wouldnt be used i think


/*  Power Control Register Masks  */
#define CODEC_REG_MASK_POWER_CONTROL_PDN        (uint8_t) 0b00000001  //Power Down Bit
#define CODEC_REG_MASK_POWER_CONTROL_PDN_DAC    (uint8_t) 0b00000010  //Power DAC Bit
//Bits 2, 3, 4 are Reserved and should  be set to 0
#define CODEC_REG_MASK_POWER_CONTROL_PDN_ADC    (uint8_t) 0b00100000  //Power ADC Bit
//Bit 6 is Reserved and should  be set to 0
#define CODEC_REG_MASK_POWER_CONTROL_FREEZE     (uint8_t) 0b10000000  //Freeze Bit


/*  Mode Control Register Masks  */
#define CODEC_REG_MASK_MODE_CONTROL_POP_GRD_DIS     (uint8_t) 0b00000001    //PopGuard Disable
#define CODEC_REG_MASK_MODE_CONTROL_MCLK_FREQ_B0    (uint8_t) 0b00000010    //Ratio Select (Main Clock Frequency Select) | Bit 0
#define CODEC_REG_MASK_MODE_CONTROL_MCLK_FREQ_B1    (uint8_t) 0b00000100    //Ratio Select (Main Clock Frequency Select) | Bit 1
#define CODEC_REG_MASK_MODE_CONTROL_MCLK_FREQ_B2    (uint8_t) 0b00001000    //Ratio Select (Main Clock Frequency Select) | Bit 2
#define CODEC_REG_MASK_MODE_CONTROL_FM_MS_M0        (uint8_t) 0b00010000    //ADC Functional Mode and Master / Slave Mode | Bit (Mode) 0
#define CODEC_REG_MASK_MODE_CONTROL_FM_MS_M1        (uint8_t) 0b00100000    //ADC Functional Mode and Master / Slave Mode | Bit (Mode) 1
//Bits 6 and 7 are Reserved and should  be set to 0


/*  ADC and DAC Control Register Masks  */
#define CODEC_REG_MASK_ADC_DAC_CONTROL_ADC_DIF          (uint8_t) 0b00000001    //ADC Digital Interface Format
//Bits 1 and 2 are Reserved and should  be set to 0
#define CODEC_REG_MASK_ADC_DAC_CONTROL_DAC_DIF0         (uint8_t) 0b00001000    //DAC Digital Interface Format | Bit 0
#define CODEC_REG_MASK_ADC_DAC_CONTROL_DAC_DIF1         (uint8_t) 0b00010000    //DAC Digital Interface Format | Bit 1
#define CODEC_REG_MASK_ADC_DAC_CONTROL_DL               (uint8_t) 0b00100000    //Digital Loopback
#define CODEC_REG_MASK_ADC_DAC_CONTROL_CH_B_ADC_HPF_DIS (uint8_t) 0b01000000    //ADC HighPass Filter Freeze Channel B
#define CODEC_REG_MASK_ADC_DAC_CONTROL_CH_A_ADC_HPF_DIS (uint8_t) 0b10000000    //ADC HighPass Filter Freeze Channel A


/*  Transition Control Register Masks  */
#define CODEC_REG_MASK_TRANSITION_CONTROL_DE_EMPH_EN    (uint8_t) 0b00000001    //De-Emphasis ControL Enable
#define CODEC_REG_MASK_TRANSITION_CONTROL_INV_DAC_A     (uint8_t) 0b00000010    //Invert Signal Polarity DAC Channel A
#define CODEC_REG_MASK_TRANSITION_CONTROL_INV_DAC_B     (uint8_t) 0b00000100    //Invert Signal Polarity DAC Channel B
#define CODEC_REG_MASK_TRANSITION_CONTROL_INV_ADC_A     (uint8_t) 0b00001000    //Invert Signal Polarity ADC Channel A
#define CODEC_REG_MASK_TRANSITION_CONTROL_INV_ADC_B     (uint8_t) 0b00010000    //Invert Signal Polarity ADC Channel B
#define CODEC_REG_MASK_TRANSITION_CONTROL_SR_EN         (uint8_t) 0b00100000    //Soft Ramp Enable
#define CODEC_REG_MASK_TRANSITION_CONTROL_ZC_EN         (uint8_t) 0b01000000    //Zero Cross Enable
#define CODEC_REG_MASK_TRANSITION_CONTROL_DAC_SV        (uint8_t) 0b10000000    //DAC Single Volume Enable


/*  Mute Control Register Masks  */
#define CODEC_REG_MASK_MUTE_CONTROL_DAC_MUTE_CH_A       (uint8_t) 0b00000001    //Mute DAC Channel A
#define CODEC_REG_MASK_MUTE_CONTROL_DAC_MUTE_CH_B       (uint8_t) 0b00000010    //Mute DAC Channel B
#define CODEC_REG_MASK_MUTE_CONTROL_DAC_MUTE_POL        (uint8_t) 0b00000100    //Mute Polarity (0 - active low; 1 - active high)
#define CODEC_REG_MASK_MUTE_CONTROL_ADC_MUTE_CH_A       (uint8_t) 0b00001000    //Mute ADC Channel B
#define CODEC_REG_MASK_MUTE_CONTROL_ADC_MUTE_CH_B       (uint8_t) 0b00010000    //Mute ADC Channel B
#define CODEC_REG_MASK_MUTE_CONTROL_ADC_AUTO_MUTE_EN    (uint8_t) 0b00100000    //Auto-Mute Enable
//Bits 6 and 7 are Reserved and should  be set to 0


/*  DAC Channel A Volume Control Register Masks  */
//They are not needed because they are just a 8 bit value of signal attenuation


/*  DAC Channel B Volume Control Register Masks  */
//They are not needed because they are just a 8 bit value of signal attenuation