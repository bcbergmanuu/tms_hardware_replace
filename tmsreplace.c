#include <stdio.h>

#include "pico/stdlib.h"
#include <stdlib.h>   

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "math.h"

#include "hardware/clocks.h"


#define ADC_CHAN 0
#define ADC_PIN 26

#define NUM_SAMPLES 3

// Sample rate (Hz)
#define Fs 100.0
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

#define pulsetime 20
#define pulse_gpio 21
#define update_time_timer_ms 50
#define sleep_time_core1 4

// DMA channels for sampling ADC
int sample_chan ;
int control_chan ;

uint8_t sample_array[NUM_SAMPLES] ;

uint8_t * sample_address_pointer = &sample_array[0] ;

/// @brief OffTime, OnTime, Frequency
uint8_t settings[NUM_SAMPLES] = {0};
volatile bool changed_settings[NUM_SAMPLES] = {0};
volatile bool state = false;
volatile uint8_t period = 0;

//output between 21 and 1100 msec
uint32_t period_calc_ms(uint8_t potsetting) {
    return (uint32_t)20+pow(1.028, 255-potsetting);
}

void core1_main() {
    
    uint32_t sleeptimer = 0;
    while(1) {                        
        if(state) {
            gpio_put(pulse_gpio, true);
            sleep_us(pulsetime);
            gpio_put(pulse_gpio, false);            
        }
        while(sleeptimer < period_calc_ms(period)) {                        
            sleep_ms(sleep_time_core1);
            sleeptimer += sleep_time_core1;
        }
        sleeptimer = 0;          
    }         
}


bool repeating_timer_callback(__unused struct repeating_timer *t) {
        
    static uint8_t ontime = 0, offtime = 0;    
    
    switch(state) {
        case true:
            //on
            ontime++;            
            if(ontime > settings[1]) {
                state = false;
                ontime = 0;
                printf("state -> off\n");
            }
            break;
        case false:
            offtime++;
            if(offtime > settings[0]) {
                state = true;                
                offtime = 0;
                printf("state -> on\n");
            }
            break;
    }    
    
    return true;
}

int round_to_5(int x) {
    return ((x + 2) / 5) * 5;
}

void dma_complete(void) {
    // Clear IRQ
    dma_hw->ints0 = 1u << sample_chan;
    
    adc_run(false);
    //dma_channel_abort(sample_chan);
    
    for(int x = 0; x < NUM_SAMPLES; x++) {    
        if (abs(settings[x] - sample_array[x]) > 4) {            
        
            settings[x] = round_to_5(sample_array[x]); 
            changed_settings[x] = true;
            if(x == 2) {                 
                period = settings[x];               
            }
        }
    }
    
    dma_channel_start(control_chan) ;
   
    adc_run(true);
}


int main()
{    
    stdio_init_all();

    printf("startup\n");
    
    gpio_init(pulse_gpio);
    gpio_set_dir(pulse_gpio, GPIO_OUT);  

    adc_gpio_init(26); //adc0
    adc_gpio_init(27); //adc1
    adc_gpio_init(28); //adc2
    

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init() ;

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    //adc_select_input(ADC_CHAN) ;
    //adc_set_temp_sensor_enabled(false);
    adc_set_round_robin(0b111);

    // Setup the FIFO
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
   
    adc_set_clkdiv(ADCCLK/Fs);


    sample_chan = dma_claim_unused_channel(true);
    control_chan = dma_claim_unused_channel(true);

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);


    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,    // transfer count
        false            // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);      // 32-bit txfers
    channel_config_set_read_increment(&c3, false);                // no read incrementing
    channel_config_set_write_increment(&c3, false);               // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);                // chain to sample chan

    dma_channel_configure(
        control_chan,                           // Channel to be configured
        &c3,                                    // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr,    // Write address (channel 0 read address)
        &sample_address_pointer,                // Read address (POINTER TO AN ADDRESS)
        1,                                      // Number of transfers, in this case each is 4 byte
        false                                   // Don't start immediately.
    );              
    

    
    dma_start_channel_mask((1u << sample_chan)) ;
    // // Start the ADC
    adc_run(true) ;
    
    dma_channel_set_irq0_enabled(sample_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete);
    irq_set_enabled(DMA_IRQ_0, true);

    struct repeating_timer timer;        
    add_repeating_timer_ms(update_time_timer_ms, repeating_timer_callback, NULL, &timer);            
    
    
    multicore_launch_core1(core1_main);
        
    while(1) {        
        sleep_ms(100);
        char* labels[8] = {"offtime", "onTime", "Period"};
        for(int x = 0; x< NUM_SAMPLES; x++) {
            if(!changed_settings[x]) continue;
            changed_settings[x] = false;
            if(x < 2) {
                printf("%s -> %f\n", labels[x], (float)settings[x] * update_time_timer_ms * 0.001);
            } else {            
                printf("%s -> %d\n", labels[x], period_calc_ms(settings[x]));                
            }            
        }                   
    }
}


