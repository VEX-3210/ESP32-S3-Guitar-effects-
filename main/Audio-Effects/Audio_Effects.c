#include "Audio_Effects.h"


//******************************************************************//
//     LowPass with First Order IIR Filter with parameter Alpha     //
//******************************************************************//

void LowPass_init(LowPass_IIR *filter, float32_t fc_Hz, float32_t fs_Hz){
    filter->fs_Hz = fs_Hz;
    filter->out = (float32_t) 0;

    LowPass_Set_Cutoff(filter, fc_Hz);
}


void LowPass_Set_Cutoff(LowPass_IIR *filter, float32_t fc_Hz){
    float32_t alpha = 2.0f*PI*(fc_Hz / filter->fs_Hz);

    filter->coeffs[0] = alpha/(1.0f+alpha);    //a0 coefficient

    filter->coeffs[1] = 1.0f/(1.0f+alpha);        //b1 coefficient


    //printf("alpha: %0.12f; a0: %0.12f; b1: %0.12f", alpha, filter->coeffs[0], filter->coeffs[1]);
}


float32_t LowPass_Calculate_Sample(LowPass_IIR *filter, float32_t input){

    filter->out = filter->coeffs[0]*input + filter->coeffs[1]*filter->out;

    return filter->out;
}


//********************************************************//
//         HighPass with First Order IIR Filter           //
//********************************************************//

void HighPass_init(HighPass_IIR *filter, float32_t fc_Hz, float32_t fs_Hz){
    filter->fs_Hz = fs_Hz;
    filter->out = (float32_t) 0;
    filter->previous_input = (float32_t) 0;

    HighPass_Set_Cutoff(filter, fc_Hz);
}


void HighPass_Set_Cutoff(HighPass_IIR *filter, float32_t fc_Hz){
    filter->coefficient_b0 = 1/(1+(2*PI*(fc_Hz/filter->fs_Hz)));
}


float32_t HighPass_Calculate_Sample(HighPass_IIR *filter, float32_t input){
    filter->out = filter->coefficient_b0*((input - filter->previous_input) + filter->out);
    filter->previous_input = input;
    return filter->out;
}


//********************************************************//
//          Overdrive with output LowPass filter          //
//********************************************************//


void Overdrive_init(Overdrive *filter, float32_t Gain, float32_t threshold, float32_t LowPass_fc_Hz, float32_t fs_Hz, float32_t volume){
    filter->Gain = Gain;
    filter->threshold = threshold;
    filter->volume = volume;
    filter->LowPass_Enable = true;

    LowPass_init(&(filter->LowPass), LowPass_fc_Hz, fs_Hz);
}

void Overdrive_set_all_params(Overdrive *filter, float32_t Gain, float32_t threshold, float32_t LowPass_fc_Hz, float32_t volume){
    filter->Gain = Gain;
    filter->threshold = threshold;
    filter->volume = volume;

    LowPass_Set_Cutoff(&(filter->LowPass), LowPass_fc_Hz);
}

void Overdrive_set_Gain(Overdrive *filter, float32_t Gain){
    filter->Gain = Gain;
}

void Overdrive_set_Threshold(Overdrive *filter, float32_t threshold){
    filter->threshold = threshold;
}

void Overdrive_set_LowPass_cutoff(Overdrive *filter, float32_t LowPass_fc_Hz){
    LowPass_Set_Cutoff(&(filter->LowPass), LowPass_fc_Hz);
}

void Overdrive_Enable_LowPass(Overdrive *filter){
    filter->LowPass_Enable = true;
}

void Overdrive_Disable_LowPass(Overdrive *filter){
    filter->LowPass_Enable = false;
}

float32_t Overdrive_Calculate_Sample(Overdrive *filter, float32_t Input){

    float Clip_In = ( (float) filter->Gain )*( (float) Input );
    float Clip_In_abs = fabs(Clip_In);
    float Clip_In_singn = (Clip_In >= 0.0f) ? 1.0f : -1.0f;

    float Clip_Out = 0.0f;

    if (Clip_In_abs<filter->threshold){
        Clip_Out = 2.0f * Clip_In;
    } else if (Clip_In_abs >= filter->threshold && Clip_In_abs < (2.0f * filter->threshold)){
        Clip_Out = Clip_In_singn * (3.0f - (2.0f - 3.0f * Clip_In_abs) * (2.0f - 3.0f * Clip_In_abs)) / 3.0f;
    } else {
        Clip_Out = Clip_In_singn;

    }

    if (filter->LowPass_Enable){
        Clip_Out = LowPass_Calculate_Sample(&(filter->LowPass), Clip_Out);
    }


    Clip_Out = Clip_Out*filter->volume;

    return Clip_Out;
}


//********************************************************//
//         Anti-Aliasing with FIR Filter (1/4 fs)         //
//********************************************************//

void AntiAliasing_FIR_init(AntiAliasing_FIR *filter){
    for (uint16_t i = 0; i < ANTI_ALIASING_FIR_LENGHT; i++){
        filter->previous_input_buffer[i] = 0.0f;
    }
    
    filter->buffer_index = 0;
    filter->out = 0.0f;
}


float32_t AntiAliasing_FIR_Calculate_Sample(AntiAliasing_FIR *filter, float32_t Input){

    filter->previous_input_buffer[filter->buffer_index] = Input;

    filter->buffer_index++;

    if (filter->buffer_index == ANTI_ALIASING_FIR_LENGHT){
        filter->buffer_index = 0;
    }

    filter->out = 0.0f;
    uint16_t sumIndex = filter->buffer_index;
    for (uint16_t i = 0; i < ANTI_ALIASING_FIR_LENGHT; i++){

        if (sumIndex > 0){
            sumIndex--;
        } else {
            sumIndex = ANTI_ALIASING_FIR_LENGHT-1;
        }

        filter->out += ANTI_ALIASING_FIR_RESPONSE[i] * filter->previous_input_buffer[sumIndex];
    }

    return filter->out;
}


//********************************************************//
//          Cabity Simulation with FIR Filter             //
//********************************************************//

void CabSim_FIR_init(CabSim_FIR *filter){
    for (uint16_t i = 0; i < CAB_SIM_FIR_LENGHT; i++){
        filter->previous_input_buffer[i] = 0.0f;
    }
    
    filter->buffer_index = 0;
    filter->out = 0.0f;
}


float32_t CabSim_FIR_Calculate_Sample(CabSim_FIR *filter, float32_t Input){

    filter->previous_input_buffer[filter->buffer_index] = Input;

    filter->buffer_index++;

    if (filter->buffer_index == CAB_SIM_FIR_LENGHT){
        filter->buffer_index = 0;
    }

    filter->out = 0.0f;
    uint16_t sumIndex = filter->buffer_index;
    for (uint16_t i = 0; i < CAB_SIM_FIR_LENGHT; i++){

        if (sumIndex > 0){
            sumIndex--;
        } else {
            sumIndex = CAB_SIM_FIR_LENGHT-1;
        }

        filter->out += CAB_SIM_FIR_RESPONSE[i] * filter->previous_input_buffer[sumIndex];
    }

    return filter->out;
}


//********************************************************//
//        FIR Filters Optimized by esp-dsp library        //
//********************************************************//


//********************************************************//
//          Cabity Simulation with FIR Filter             //
//********************************************************//

static const char *TAG = "FIR_INIT";

void CabSim_ESP_S3_FIR_init(CabSim_ESP_S3_FIR *filter){

    for (size_t i = 0; i < CAB_SIM_ESP_S3_FIR_LENGHT; i++){
        (filter->coeffs)[i] = CAB_SIM_ESP_S3_FIR_RESPONSE[(CAB_SIM_ESP_S3_FIR_LENGHT-1) - i];
    }

    esp_err_t ret = dsps_fir_init_f32(&(filter->dsp_filter), filter->coeffs, filter->delay, CAB_SIM_ESP_S3_FIR_LENGHT);
    ESP_LOGI(TAG, "FIR filter 0x%x", ret);
}


void CabSim_ESP_S3_FIR_Calculate_Block(CabSim_ESP_S3_FIR *filter, float32_t *Input_Block){
    dsps_fir_f32_aes3(&(filter->dsp_filter), Input_Block, filter->out, CAB_SIM_ESP_S3_BLOCK_LENGHT);
}