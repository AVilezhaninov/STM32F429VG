/* CMSIS */
#include "CMSIS\Device\stm32f4xx.h"

/* User */
#include "user\RCC.h"

/******************************************************************************/
/* Private definitions ********************************************************/
/******************************************************************************/
#define TIM2_PSC    (9000u - 1u)  /* 10kHz timer clock */
#define TIM2_ARR    (1000u - 1u)  /* 100ms ADC start period */

#define ADC_DMA_IRQ_PRIORITY        5u
#define INTERNAL_REFERENCE_VOLTAGE  1.21f

/******************************************************************************/
/* Private variables **********************************************************/
/******************************************************************************/
static struct {
  uint16_t ref_value;
  uint16_t ch0_value;
  uint16_t ch1_value;
  uint16_t ch2_value;
  uint16_t ch3_value;
  uint16_t ch4_value;
  uint16_t ch5_value;
  uint16_t ch6_value;
  uint16_t ch7_value;
} adc_data;

static struct {
  float ref_step;
  float ch0_voltage;
  float ch1_voltage;
  float ch2_voltage;
  float ch3_voltage;
  float ch4_voltage;
  float ch5_voltage;
  float ch6_voltage;
  float ch7_voltage;
} adc_voltage;

/******************************************************************************/
/* Private function prototypes ************************************************/
/******************************************************************************/
void InitGpios();
void InitTimer();
void InitAdc();
void InitAdcDma();
void StartConversion();

/******************************************************************************/
/* Interrupts *****************************************************************/
/******************************************************************************/
extern "C" {

void DMA2_Stream0_IRQHandler(void) {
  float ref_step = INTERNAL_REFERENCE_VOLTAGE / adc_data.ref_value;;

  DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

  adc_voltage.ref_step = ref_step;
  adc_voltage.ch0_voltage = adc_data.ch0_value * ref_step;
  adc_voltage.ch1_voltage = adc_data.ch1_value * ref_step;
  adc_voltage.ch2_voltage = adc_data.ch2_value * ref_step;
  adc_voltage.ch3_voltage = adc_data.ch3_value * ref_step;
  adc_voltage.ch4_voltage = adc_data.ch4_value * ref_step;
  adc_voltage.ch5_voltage = adc_data.ch5_value * ref_step;
  adc_voltage.ch6_voltage = adc_data.ch6_value * ref_step;
  adc_voltage.ch7_voltage = adc_data.ch7_value * ref_step;
}

} /* extern "C" */

/******************************************************************************/
/* Main ***********************************************************************/
/******************************************************************************/
int main(void) {
  InitSystemClock();
  InitGpios();
  InitTimer();
  InitAdc();
  InitAdcDma();
  StartConversion();

  while (1) {
    ;
  }
}

/******************************************************************************/
/* Private functions **********************************************************/
/******************************************************************************/
void InitGpios() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  /* Enable port clock */
  GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0; /* PA0 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE1_1 | GPIO_MODER_MODE1_0; /* PA1 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE2_0; /* PA2 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE3_1 | GPIO_MODER_MODE3_0; /* PA3 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE4_1 | GPIO_MODER_MODE4_0; /* PA4 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE5_1 | GPIO_MODER_MODE5_0; /* PA5 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE6_0; /* PA6 in analog
                                                            *  mode */
  GPIOA->MODER |= GPIO_MODER_MODE7_1 | GPIO_MODER_MODE7_0; /* PA7 in analog
                                                            *  mode */
}

void InitTimer() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /* Enable timer clock */
  TIM2->PSC = TIM2_PSC;               /* Set prescaler */
  TIM2->ARR = TIM2_ARR;               /* Set auto-reload value */
  TIM2->DIER |= TIM_DIER_UIE;         /* Enable update IRQ */
  TIM2->CR2 |= TIM_CR2_MMS_1;         /* Update TRGO output */
}

void InitAdc() {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* Enable ADC clock */

  ADC1->SQR3 |= (17u << ADC_SQR3_SQ1_Pos);  /* Ch17 - ref channel */
  ADC1->SQR3 |= (1u << ADC_SQR3_SQ3_Pos);   /* Ch1 */
  ADC1->SQR3 |= (2u << ADC_SQR3_SQ4_Pos);   /* Ch2 */
  ADC1->SQR3 |= (3u << ADC_SQR3_SQ5_Pos);   /* Ch3 */
  ADC1->SQR3 |= (4u << ADC_SQR3_SQ6_Pos);   /* Ch4 */
  ADC1->SQR2 |= (5u << ADC_SQR2_SQ7_Pos);   /* Ch5 */
  ADC1->SQR2 |= (6u << ADC_SQR2_SQ8_Pos);   /* Ch6 */
  ADC1->SQR2 |= (7u << ADC_SQR2_SQ9_Pos);   /* Ch7 */
  ADC1->SQR1 |= (8u << ADC_SQR1_L_Pos);     /* 9 conversions */

  ADC1->SMPR2 |= ADC_SMPR2_SMP0;  /* Ch17 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP1;  /* Ch0 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP2;  /* Ch1 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP3;  /* Ch2 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP4;  /* Ch3 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP5;  /* Ch4 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP6;  /* Ch5 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP7;  /* Ch6 -  480 cycles conversion */
  ADC1->SMPR2 |= ADC_SMPR2_SMP8;  /* Ch7 -  480 cycles conversion */

  ADC1->CR1 |= ADC_CR1_SCAN;      /* Scan mode */
  ADC1->CR2 |= ADC_CR2_DMA;       /* DMA mode */
  ADC1->CR2 |= ADC_CR2_DDS;       /* DMA disable selection */
  ADC1->CR2 |= ADC_CR2_EXTEN;     /* Enable external trigger */
  ADC1->CR2 |= ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1; /* Timer 2 TRGO event */
  ADC->CCR |= ADC_CCR_TSVREFE;    /* Vref enable */
  ADC->CCR |= ADC_CCR_ADCPRE_0;   /* ADC prescaler - 4 */
}

void InitAdcDma() {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;        /* Enable DMA clock */
  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;      /* Set memory size */
  DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;      /* Set peripheral size */
  DMA2_Stream0->CR |= DMA_SxCR_MINC;         /* Set memory increment mode */
  DMA2_Stream0->CR |= DMA_SxCR_CIRC;         /* Set circular mode */
  DMA2_Stream0->CR |= DMA_SxCR_TCIE;         /* Enabel transfer comlete IRQ */
  DMA2_Stream0->NDTR = 9u;                   /* Set number of data */
  DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;   /* Set peripheral address */
  DMA2_Stream0->M0AR = (uint32_t)&adc_data;  /* Set adc DMA address */

  NVIC_SetPriority(DMA2_Stream0_IRQn, ADC_DMA_IRQ_PRIORITY);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void StartConversion() {
  DMA2_Stream0->CR |= DMA_SxCR_EN;  /* Enable ADC DMA */
  ADC1->CR2 |= ADC_CR2_ADON;        /* Enablr ADC */
  TIM2->CR1 |= TIM_CR1_CEN;         /* Enable ADC timer */
}
