#include "esp_adc/adc_oneshot.h"
#include <math.h>

#define BITWIDTH ADC_BITWIDTH_12

const uint32_t BITWIDTH_MAX_VALUE = (uint32_t) pow(2, BITWIDTH)-1;
const float ADC_RAW_NORMALIZE = 1.0f / ((float) BITWIDTH_MAX_VALUE);

void ADC_Pot_Init(adc_oneshot_unit_handle_t *adc_handle, adc_unit_t ADC){

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC,                     //ADC index
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, //Deafult clock for ADC
        .ulp_mode = ADC_ULP_MODE_DISABLE    //Sets if ADC will be used by ULP processor of ESP32
    };

    adc_oneshot_new_unit(&adc_config, adc_handle);
}

void ADC_Pot_Channel_Init(adc_oneshot_unit_handle_t *adc_handle, adc_channel_t ADC_CHANNEL){

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = BITWIDTH,               //Sets Up Amount of Bits that ADC will use
        .atten = ADC_ATTEN_DB_12            //Allows to measure voltages between 0V and 3.3V
    };

    adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL, &adc_channel_config);
}

float ADC_Pot_Get_Value(adc_oneshot_unit_handle_t *adc_handle, adc_channel_t ADC_CHANNEL){

    int raw;
    adc_oneshot_read(*adc_handle, ADC_CHANNEL, &raw);

    return ((float) raw)*ADC_RAW_NORMALIZE;
}

float range(float input, float min, float max){
    return input*(max-min) + min;
}