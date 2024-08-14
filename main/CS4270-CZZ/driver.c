#include "driver.h"

#define I2C_CODEC_ADDRESS (0b1001000 | CODEC_ADDRESS_LINES)

static IRAM_ATTR bool i2s_rx_on_recv_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx){

    // handle RX queue overflow event ...
    //ESP_LOGI("recv_callback", "works");
    //uint32_t buffer[I2S_DMA_BUFFER_FRAME_SIZE];
    //size_t size = I2S_DMA_BUFFER_FRAME_SIZE;
    
    
    Codec_CS4270 *Codec_struct = (Codec_CS4270*) user_ctx;


    i2s_channel_read(handle, &(Codec_struct->InData), sizeof(Codec_struct->InData), NULL, 7);
    i2s_channel_write(Codec_struct->tx_handle, &(Codec_struct->OutData), sizeof(Codec_struct->OutData), NULL, 7);

    Codec_struct->data_ready = true;
    return false;
    
}

/*#define I2S_STD_CLK_CONFIG(rate, mclk_multiple) { \
    .sample_rate_hz = rate, \
    .clk_src = I2S_CLK_SRC_DEFAULT, \
    .mclk_multiple = mclk_multiple, \
}*/


void I2C_Codec_Init(Codec_CS4270 *Codec_struct){
    i2c_master_bus_config_t I2C_CS4270_Codec = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_io_num = I2C_MASTER_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = I2C_INTERNAL_PULLUP_ENABLE,
    };
    i2c_master_bus_handle_t I2C_bus_handle;

    

    ESP_ERROR_CHECK(i2c_new_master_bus(&I2C_CS4270_Codec, &I2C_bus_handle));

    i2c_device_config_t I2C_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_CODEC_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY_HZ,
    };

    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2C_bus_handle, &I2C_dev_cfg, &(Codec_struct->I2C_device)));

    //ESP_ERROR_CHECK(i2c_master_transmit(I2C_dev_handle, &DATA, DATA_LENGTH, -1)); //Transmit

    
}

uint8_t Codec_Read_Register(Codec_CS4270 *Codec_struct, uint8_t address){

    ESP_ERROR_CHECK(i2c_master_transmit(Codec_struct->I2C_device, &address, 1 /*Data Length*/, -1));  //Set the Memory Address Pointer (MAP) of the Codec

    uint8_t DATA_res = 0;
    ESP_ERROR_CHECK(i2c_master_receive(Codec_struct->I2C_device, &DATA_res, 1 /*Data Length*/, -1));  //Reads Data Content of the register with the MAP points too

    return DATA_res;
}

void Codec_Write_Register(Codec_CS4270 *Codec_struct, uint8_t address, uint8_t value){

    uint8_t out_buffer[2] = {address, value};

    //Set the Memory Address Pointer (MAP) of the Codec and Sets The values of the register to values
    ESP_ERROR_CHECK(i2c_master_transmit(Codec_struct->I2C_device, &out_buffer[0], 2 /*Data Length*/, -1));  

}

void Codec_Set_Registers_To_Default(Codec_CS4270 *Codec_struct){
    uint8_t output_buffer[8] = {CODEC_REG_ADDR_POWER_CONTROL | CODEC_MAP_INCR, CODEC_REG_DEFAULT_POWER_CONTROL, CODEC_REG_DEFAULT_MODE_CONTROL, CODEC_REG_DEFAULT_ADC_DAC_CONTROL, 
                                CODEC_REG_DEFAULT_TRANSITION_CONTROL, CODEC_REG_DEFAULT_MUTE_CONTROL, CODEC_REG_DEFAULT_CH_A_VOL_CONTROL, CODEC_REG_DEFAULT_CH_B_VOL_CONTROL};

    ESP_ERROR_CHECK(i2c_master_transmit(Codec_struct->I2C_device, &output_buffer[0], 8 /*Data Length*/, -1));  
}

void Debug_See_Codec_Regitsers(Codec_CS4270 *Codec_struct){
    uint8_t ID = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_ID);
    uint8_t Power_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_POWER_CONTROL);
    uint8_t Mode_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_MODE_CONTROL);
    uint8_t ADC_DAC_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_ADC_DAC_CONTROL);
    uint8_t Transition_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_TRANSITION_CONTROL);
    uint8_t Mute_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_MUTE_CONTROL);
    uint8_t DAC_CH_A_Vol_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_CH_A_VOL_CONTROL);
    uint8_t DAC_CH_B_Vol_Control = Codec_Read_Register(Codec_struct, CODEC_REG_ADDR_CH_B_VOL_CONTROL);

    printf("ID:\t\t\t%d\n", ID);
    printf("Power Control:\t\t%d\n", Power_Control);
    printf("Mode Control:\t\t%d\n", Mode_Control);
    printf("ADC DAC Control:\t%d\n", ADC_DAC_Control);
    printf("Transition Control:\t%d\n", Transition_Control);
    printf("Mute Control:\t\t%d\n", Mute_Control);
    printf("DAC CH A Vol Control:\t%d\n", DAC_CH_A_Vol_Control);
    printf("DAC CH B Vol Control:\t%d\n", DAC_CH_B_Vol_Control);

    printf("\n///////////////////////////////////////////\n\n");
}

void I2S_DMA_Codec_Init(Codec_CS4270 *Codec_struct){
    gpio_set_direction(NRST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NRST_PIN, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);


    /* Allocate a pair of I2S channel */
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_AUTO,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = I2S_DMA_BUFFER_COUNT,
        .dma_frame_num = I2S_DMA_BUFFER_FRAME_SIZE,
        .auto_clear = false,
        .intr_priority = 7,
    };

    /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
    i2s_new_channel(&chan_cfg, &(Codec_struct->tx_handle), &(Codec_struct->rx_handle));

    i2s_std_config_t I2S_config = {
        .clk_cfg = {
            .sample_rate_hz = I2S_SAMPLIG_RATE_HZ, 
            .clk_src = I2S_CLK_SRC_DEFAULT, 
            .mclk_multiple = I2S_MCLK_MULTIPLE 
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        //.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_SCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }

        
    };

    i2s_channel_init_std_mode((Codec_struct->tx_handle), &I2S_config);
    i2s_channel_init_std_mode((Codec_struct->rx_handle), &I2S_config);

    i2s_event_callbacks_t cbs = {
        .on_recv = i2s_rx_on_recv_callback,
        .on_recv_q_ovf = NULL,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };

    i2s_channel_register_event_callback((Codec_struct->rx_handle), &cbs, Codec_struct);

    i2s_channel_enable((Codec_struct->tx_handle));
    i2s_channel_enable((Codec_struct->rx_handle));

    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(NRST_PIN, 1);
}

void Codec_Init(Codec_CS4270 *Codec_struct){
    Codec_struct->data_ready = false;

    for (size_t i = 0; i < I2S_BUFFER_SIZE; i++){
        Codec_struct->OutData[i] = 0;
    }

    I2C_Codec_Init(Codec_struct);
    


    Codec_Set_Registers_To_Default(Codec_struct);

    //Debug_See_Codec_Regitsers(Codec_struct);

    //Codec_Write_Register(Codec_struct, CODEC_REG_ADDR_MODE_CONTROL, CODEC_REG_DEFAULT_MODE_CONTROL | CODEC_REG_MASK_MODE_CONTROL_MCLK_FREQ_B0);
    //Codec_Write_Register(Codec_struct, CODEC_REG_ADDR_MUTE_CONTROL, 0);
    //Codec_Write_Register(Codec_struct, CODEC_REG_ADDR_TRANSITION_CONTROL, 0);
    //Codec_Write_Register(Codec_struct, CODEC_REG_ADDR_ADC_DAC_CONTROL, CODEC_REG_MASK_ADC_DAC_CONTROL_ADC_DIF | CODEC_REG_MASK_ADC_DAC_CONTROL_DAC_DIF0);

    Debug_See_Codec_Regitsers(Codec_struct);


    I2S_DMA_Codec_Init(Codec_struct);

}
