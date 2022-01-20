#include "stm8s.h"
#include "milis.h"

#include "delay.h"
#include <stdio.h>
#include "spse_stm8.h"
#include "stm8_hd44780.h"

#include "stm8s_adc2.h"
#include "uart1.h"

#define _ISOC99_SOURCE
#define _GNU_SOURCE


/*
neopixel DI - PC
encoder CLK - PF7
        DT - PF6
*/

uint16_t vzdalenost = 400;
static uint16_t minule=1; // pamatuje si minulý stav vstupu A (nutné k detekování sestupné hrany)
	// pokud je na vstupu A hodnota 0 a minule byla hodnota 1 tak jsme zachytili sestupnou hranu
void init_spi(void)
{
    // Software slave managment (disable CS/SS input), BiDirectional-Mode release MISO pin to general purpose
    SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI | SPI_CR2_BDM | SPI_CR2_BDOE;
    // Enable SPI as master at maximum speed (F_MCU/2, there 16/2=8MHz)
    SPI->CR1 |= SPI_CR1_SPE | SPI_CR1_MSTR;
}

void setup(void)
{
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);      // taktovani MCU na 16MHz

    init_milis();
    init_uart1();

     // na pinech/vstupech ADC_IN2 (PB4) a ADC_IN3 (PB5) vypneme vstupní buffer
    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL4, DISABLE);
    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL5, DISABLE);

    // při inicializaci volíme frekvenci AD převodníku mezi 1-4MHz při 3.3V
    // mezi 1-6MHz při 5V napájení
    // nastavíme clock pro ADC (16MHz / 4 = 4MHz)
    ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);

    // volíme zarovnání výsledku (typicky vpravo, jen vyjmečně je výhodné vlevo)
    ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
    
    // nasatvíme multiplexer na některý ze vstupních kanálů
    ADC2_Select_Channel(ADC2_CHANNEL_4);
    // rozběhneme AD převodník
    ADC2_Cmd(ENABLE);
    // počkáme než se AD převodník rozběhne (~7us)
    ADC2_Startup_Wait();
}

#define L_PATTERN 0b01110000    // 3x125ns (8MHZ SPI)
#define H_PATTERN 0b01111100    // 5x125ns (8MHZ SPI), first and last bit must be zero (to remain MOSI in Low between frames/bits)
// takes array of LED_number * 3 bytes (RGB per LED)

void neopixel(uint8_t * data, uint16_t length)
{
    uint8_t mask;
    disableInterrupts();        // can be omitted if interrupts do not take more then about ~25us
    while(length--) {            // for all bytes from input array
        mask = 0b10000000;     // for all bits in byte
        while (mask) {
            while (!(SPI->SR & SPI_SR_TXE));    // wait for empty SPI buffer
            if (mask & data[length]) {  // send pulse with coresponding length ("L" od "H")
                SPI->DR = H_PATTERN;
            } else {
                SPI->DR = L_PATTERN;
            }
            mask = mask >> 1;
        }
    }
    enableInterrupts();
    while (SPI->SR & SPI_SR_BSY); // wait until end of transfer - there should come "reset" (>50us in Low)
}


// test pattern for (16 RGB LED ring)
uint8_t colors[24*3];

void my_delay_ms(uint16_t ms) {
    uint16_t  i;
    for (i=0; i<ms; i = i+1){
        _delay_us(250);
        _delay_us(248);
        _delay_us(250);
        _delay_us(250);
    }
}

 
 void process_enc(void){
	
	if(GPIO_ReadInputPin(GPIOF,GPIO_PIN_7) == RESET && minule==1){
		minule = 0; // nyní je pin v log.0
		// pøeèteme stav vstupu B
		if(GPIO_ReadInputPin(GPIOF,GPIO_PIN_6) == RESET){
			// log.0 na vstupu B (krok jedním smìrem)
			vzdalenost++;
		}else{
			// log.1 na vstupu B (krok druhým smìrem)
			vzdalenost--;
		}
	}
	if(GPIO_ReadInputPin(GPIOF,GPIO_PIN_7) != RESET){minule = 1;} // pokud je vstup A v log.1
}
 
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
 {
     TIM3_ClearITPendingBit(TIM3_IT_UPDATE); // vyeistit vlajku (nutné vždy)
     process_enc(); // zkontrolovat stav pinu enkodéru
 }
void init_enc(void){
	// enkodéry jsou jen spínaèe, takže vstupy volíme ve stejném režimu jako pro tlaèítka
GPIO_Init(GPIOF,GPIO_PIN_7,GPIO_MODE_IN_PU_NO_IT);  // vstup, s vnitøním pullup rezistorem
GPIO_Init(GPIOF,GPIO_PIN_6,GPIO_MODE_IN_PU_NO_IT);
}

void init_timer(void){
TIM3_TimeBaseInit(TIM3_PRESCALER_16,1999); // perioda pøeteèení/update 2ms
TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE); // povolit pøerušení
TIM3_Cmd(ENABLE); // spustit timer
}

void minVzdalenost(void){
}

void fillAll(uint16_t r,uint16_t g,uint16_t b ) {
    uint16_t  i = 0;
    while(i<=24*3){
        colors[i] = r;
        colors[i+1] = g;
        colors[i+2] = b;
        i = i+3;
    }
    neopixel(colors, sizeof(colors));
}

int main(void)
{
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // 16MHz from internal RC
    init_milis(); // millis using TIM4 - not necessary
    init_spi();
    init_enc();        // inicializace vstupu enkodéru
    init_timer();    // spustí tim3 s poerušením každé 2ms
    setup();
    init_spi();
    fillAll(20,0,0);
    while (1) {
        my_delay_ms(10);
        printf("%d\r\n", vzdalenost);
    }

}


/*-------------------------------  Assert -----------------------------------*/
/*#include "__assert__.h"*/
