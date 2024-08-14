#include <stdio.h>
#include "CS4270-CZZ/driver.c"
#include "Audio-Effects/Audio_Effects.c"
#include "ADC_Potentiometer.c"



//#define INT24_NORMALIZE (float) 4.656613150633165e-10
#define INT24_NORMALIZE (float) 4.656612875245796e-10
#define INT24_DENORMALIZE INT32_MAX



Codec_CS4270 codec;
adc_oneshot_unit_handle_t ADC2;


void audio_process_task(void *param) {

    float R_in[I2S_BUFFER_SIZE/2];
    float L_in[I2S_BUFFER_SIZE/2];

    LowPass_IIR filter1;
    HighPass_IIR filter2;
    AntiAliasing_FIR filter3;
    CabSim_ESP_S3_FIR filter4;
    Overdrive filter5;


    LowPass_init(&filter1, (float) 3000, (float) I2S_SAMPLIG_RATE_HZ);
    HighPass_init(&filter2, (float) 100, (float) I2S_SAMPLIG_RATE_HZ);
    AntiAliasing_FIR_init(&filter3);
    CabSim_ESP_S3_FIR_init(&filter4);

    Overdrive_init(&filter5, 10.0f, 1.0f/3.0f, 7000, I2S_SAMPLIG_RATE_HZ, 1.0f);

    Overdrive_Disable_LowPass(&filter5);


    float Pots[3];
    Pots [0] = 1;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_7);
    Pots [1] = 1;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_6);
    Pots [2] = 0.05;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_5);
    
    while (true){


        if (codec.data_ready){
            gpio_set_level(GPIO_NUM_8, 0);
            
            int LowPass_Enable = gpio_get_level(GPIO_NUM_36); //SCK Pin

            //printf("%d\n", LowPass_Enable);

            
            //Pots [0] = 1;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_7);
            //Pots [1] = 1;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_6);
            //Pots [2] = 0.05;//ADC_Pot_Get_Value(&ADC2, ADC_CHANNEL_5);

            int cutoff = (int) range(Pots[1], 500, 10000);
            float gain = range(Pots[0], 1, 50);

            //printf("%0.12f\t%0.12f\t%0.12f\t\n", Pots[0], Pots[1], Pots[2]);
            //printf("%0.12f\t%d\n", Pots[1], cutoff);
            

            Overdrive_set_LowPass_cutoff(&filter5, cutoff);
            Overdrive_set_Gain(&filter5, gain);
            

            /*if (LowPass_Enable){
                Overdrive_Enable_LowPass(&filter5);
            } else {
                Overdrive_Disable_LowPass(&filter5);
            }*/

            for (size_t i = 0; i < I2S_BUFFER_SIZE; i+=2){
                
                //float output_normalized = (float)( ((float) codec.InData[i]) *INT24_NORMALIZE );
                R_in[(int)(i/2)] = (float)( ((float) codec.InData[i]) *INT24_NORMALIZE );
                L_in[(int)(i/2)] = (float)( ((float) codec.InData[i+1]) *INT24_NORMALIZE );

                //codec.OutData[i] = (int) (output_normalized*INT24_DENORMALIZE);
            }
            

            
            if (LowPass_Enable){

                
            } else {
                Pots [2] = 0.05f;

                for (size_t i = 0; i < (I2S_BUFFER_SIZE/2); i++){
                    R_in[i] = Overdrive_Calculate_Sample(&filter5, R_in[i]);
                }

                CabSim_ESP_S3_FIR_Calculate_Block(&filter4, R_in);

                for (size_t i = 0; i < (I2S_BUFFER_SIZE/2); i++){
                    R_in[i] = filter4.out[i]*Pots[2];
                }

            }

            


            for (size_t i = 0; i < I2S_BUFFER_SIZE; i+=2){
                //codec.OutData[i] =   (int)  ( (filter4.out[(int)(i/2)] * 0.5) * INT24_DENORMALIZE );
                /*if (LowPass_Enable){
                    codec.OutData[i] =   (int)  ( filter4.out[(int)(i/2)] * INT24_DENORMALIZE );
                } else {*/
                    codec.OutData[i] =   (int)  ( R_in[(int)(i/2)] * INT24_DENORMALIZE );
                //}
                
                codec.OutData[i+1] = (int)  ( L_in[(int)(i/2)] * INT24_DENORMALIZE );
            }

            //printf( "R: %d\tL: %d\tsizeof: %d\n", codec.OutData[I2S_BUFFER_SIZE-2], codec.OutData[I2S_BUFFER_SIZE-1], sizeof(codec.OutData));
            //printf( "R: %0.12f\tL: %0.12f\n", R_in[(I2S_BUFFER_SIZE/2)-1], L_in[(I2S_BUFFER_SIZE/2)-1]);
            codec.data_ready = false;
            gpio_set_level(GPIO_NUM_8, 1);
        }

        
    }
}

void app_main(void){

    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);  //A2 Pin
    gpio_set_level(GPIO_NUM_8, 1);

    gpio_set_direction(GPIO_NUM_36, GPIO_MODE_INPUT);   //SCK Pin
    gpio_set_pull_mode(GPIO_NUM_36, GPIO_PULLUP_ENABLE);

    
    Codec_Init(&codec);

    ADC_Pot_Init(&ADC2, ADC_UNIT_2);
    ADC_Pot_Channel_Init(&ADC2, ADC_CHANNEL_7); //A0
    ADC_Pot_Channel_Init(&ADC2, ADC_CHANNEL_6); //A1
    ADC_Pot_Channel_Init(&ADC2, ADC_CHANNEL_5); //A2

    printf("Setup Done!\n");
    

    xTaskCreate(audio_process_task, "audio_process_task", 8192, NULL, 5, NULL);
}
