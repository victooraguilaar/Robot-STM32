
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h> //New implementation to use the USART
#include <stdarg.h> //Implementation to use variadic functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUTOMATICO 1
#define MANUAL 0
#define RANGE 7 //Grados de diferencia minimos para actualizar la posicion
#define STOP_WAITING 45000
#define SOUND_SPEED 0.034 //[cm/µs]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart2;

volatile unsigned int current_voltage=0;
volatile int position=0;
volatile int last_position=0;
volatile int counter;
volatile char mode, change_mode, pulsador_2;
volatile int counter_medidas;
volatile int tiempo_salida, tiempo_llegada;
volatile int distancia_objeto;
volatile char medida_ok=0;
volatile char echo_start=0;//Estado inicial
volatile char posicion_servo=0;

volatile int distancias_auto[5]; //To store the measurements in the automatic mode

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void led_verde_on(void);

void led_verde_off(void);

void led_rojo_on(void);

void led_rojo_off(void);

void medir_distancia_auto(void);

void medir_posicion(void);

void medir_distancia_manual(void);

void conversor_tiempo_distancia(void);

void resultado_medidas_LED(void);

void posicion_servo_auto(void);

void uart_print(const char *format, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//We comment this function, previously used to send the printf content from the microcontroller to the console through the USART
/*int _write(int file, char *ptr, int len) {
int i=0;
for(i=0; i<len; i++)
HAL_UART_Transmit(&huart2,(uint8_t *)ptr++,1,1000);
return len;
}
*/

//New function used to send the messages through the usart without using printf
void uart_print(const char *format, ...) {
    char tx_buffer[100]; //buffer array

    //just the regular structure for variadic functions
    va_list args;
    va_start(args, format);
    vsprintf(tx_buffer, format, args); //This function is used to store strings, however, by using it inside a variadic function, the extra arguments are for the format of different numbers such as decimals, integers, etc
    va_end(args);

    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), 100);
}

void led_verde_on(void){
  GPIOA->BSRR |= (1<<5);
}

void led_verde_off(void){
  GPIOA->BSRR |= (1<<21);
}

void led_rojo_on(void){
  GPIOB->BSRR |= (1<<5);
}

void led_rojo_off(void){
  GPIOB->BSRR |= (1<<21);
}

void trigger_on(void){
  GPIOC->BSRR |= (1<<8);
}

void trigger_off(void){
  GPIOC->BSRR |= (1<<24);
}

void all_leds_on(void){
  GPIOC->BSRR |= 31;
}

void all_leds_off(void){
  GPIOC->BSRR |= (31<<16);
}

void posicion_servo_auto(void){

  if(counter_medidas==0)
    TIM2->CCR1=500;

  else if(counter_medidas==1)
    TIM2->CCR1=975;

  else if(counter_medidas==2)
    TIM2->CCR1=1450;

  else if(counter_medidas==3)
    TIM2->CCR1=1925;

  else if(counter_medidas==4)
    TIM2->CCR1=2400;

}

void EXTI15_10_IRQHandler(void){

  if((EXTI->PR &(1<<13))!=0){
      change_mode=1;

    EXTI->PR |= (1<<13);
  }

}

void EXTI4_IRQHandler(void){

  if((EXTI->PR & (1<<4))!=0){
    pulsador_2=1;

    if((mode==AUTOMATICO) && ((counter_medidas >= 5)||(counter_medidas < 0)))
      counter_medidas=0;

    if(mode==MANUAL){
      trigger_on(); //Activamos el voltaje en el trigger
      TIM3->CR1 |= 1; //Inicia el contador del sensor.
    }

    EXTI->PR |= (1<<4);
  }

}

void ADC1_IRQHandler(void){

  if((ADC1->SR & 2)!=0){
    current_voltage= ADC1->DR;
    ADC1->SR &= ~2;
  }

}


void TIM4_IRQHandler(void){

  if((TIM4->SR & (1<<2))!=0){

    if(mode==MANUAL){
      TIM4->CR1 &= ~(1); //Paramos el contador
      TIM4->CNT=0; //Reseteamos el CNT
    }

    led_rojo_off();

    if(mode==AUTOMATICO){
      trigger_on(); //Activamos el voltaje en el trigger
      TIM3->CR1 |= 1;//Inicia el contador del sensor.
    }

    TIM4->SR &= ~(1<<2); //Limpiamos el flag
  }

  if((TIM4->SR & 2)!=0){
    TIM4->CR1 &= ~(1); //Paramos el contador
    TIM4->CNT=0; //Reseteamos el CNT
    TIM4->SR &= ~(2); //Limpiamos el flag
  }
}

void TIM3_IRQHandler(void){

  if((TIM3->SR & 2)!=0){ //Si la interrupcion se debe al canal 1 (han pasado 10µs)
    trigger_off();
    echo_start=1;// Indica que el sensor va a lanzar lso pulsos de sonido
    TIM3->SR &= ~(2); //Limpiamos el flag
  }

  else if((TIM3->SR & (1<<4))!=0){//Si la interrupcion se debe al canal 4 (el ECHO cambia de estado)

    if(echo_start==1){//Si esta a 1 es que la interrupción se debe a que el ECHO se ha puesto a 1 (a lanzado el sonido)
      echo_start=2; //Que esté a 2 implica que el sonido esta viajando
      tiempo_salida=TIM3->CCR4;
    }
    else if(echo_start==2){ //Si esta a 2 implica que la interrupcion se debe a que el ECHO a pasado a 0 (el sonido a rebotado y llegado al sensor)
      TIM3->CR1 &= ~(1); //En cuanto el sonido llegue de nuevo al sensor paramos el contador y reseteamos el CNT
      TIM3->CNT=0;
      tiempo_llegada=TIM3->CCR4;
      medida_ok=1;
    }

    TIM3->SR &= ~(1<<4);
  }
}

void medir_distancia_auto(void){

  if((TIM4->CR1 & 1)==0){

    TIM4->CR1 |= 1; //Inicia el contador de 2s para una medida
    led_rojo_on();
    posicion_servo_auto();
    counter_medidas++;

  }

}

void medir_posicion(void){

  int current_position= ((uint32_t)current_voltage*180)/4095;

  if(abs(current_position - last_position) > RANGE){

    if(current_position >= 175)
      position= 180;

    else if(current_position <= 6)
      position=0;

    else
      position=current_position;

    last_position = current_position;

    TIM2->CCR1=500 + (position*1900)/180; /* Factor de conversion de grados a tiempo para el servomotor,

    los 500µs son el offset ya que para el servo 0º son 500µs de Duty Cycle*/

    counter=0;

  }

  else{

    if(counter < STOP_WAITING)

      counter++;

    if(counter == STOP_WAITING ){ //Cuando se hayan pasado 45000 iteraciones y el voltaje no haya variado, entonces imprimimos los datos
      uart_print("POSITION= %d\r\n", position);
      counter++;
    }

  }

}

void medir_distancia_manual(void){

  led_rojo_on();
  pulsador_2=0;
  TIM4->CR1 |= 1; //Inicia el contador de encendido del LED
}

void conversor_tiempo_distancia(void){

  // SOLO hacemos cosas si el sonido ha vuelto con éxito
  if(medida_ok == 1){

    distancia_objeto = (int)(((tiempo_llegada - tiempo_salida)/2) * SOUND_SPEED);

    if(mode == MANUAL){
      uart_print("> Modo manual -> distancia: %d cm\r\n", distancia_objeto);
    }
    else if(mode == AUTOMATICO) {
      if (counter_medidas > 0 && counter_medidas <= 5) {
        distancias_auto[counter_medidas - 1] = distancia_objeto;
      }
    }
  }

  // Hacemos un if a parte para manejar el texto que se imprima en el modo automatico
  //Se pone como condicion que se haya detenido el timer para que no haya race conditions
  //En caso de no haberse tomado una de las medidas, se imprimira "ERROR en esa posicion del array en vez de no imprimirse el array como en los laboratorios anteriores
  if (mode == AUTOMATICO && counter_medidas == 5 && (TIM4->CR1 & 1) == 0) {

    uart_print("> Modo automatico -> distancia: [");

    if(distancias_auto[0] == -1) uart_print("ERROR, ");
    else uart_print("%d, ", distancias_auto[0]);

    if(distancias_auto[1] == -1) uart_print("ERROR, ");
    else uart_print("%d, ", distancias_auto[1]);

    if(distancias_auto[2] == -1) uart_print("ERROR, ");
    else uart_print("%d, ", distancias_auto[2]);

    if(distancias_auto[3] == -1) uart_print("ERROR, ");
    else uart_print("%d, ", distancias_auto[3]);

    if(distancias_auto[4] == -1) uart_print("ERROR");
    else uart_print("%d", distancias_auto[4]);

    uart_print("] cm\r\n");

    counter_medidas = -1;
  }
}

void resultado_medidas_LED(void){

  if(medida_ok==1){
    medida_ok=0;
    all_leds_off();

    if((distancia_objeto >= 60)&&(distancia_objeto < 80))
      GPIOC->BSRR |= 1;

    else if((distancia_objeto >= 40)&&(distancia_objeto < 60))
      GPIOC->BSRR |= 3;

    else if((distancia_objeto >= 20)&&(distancia_objeto < 40))
      GPIOC->BSRR |= 7;

    else if((distancia_objeto >= 10)&&(distancia_objeto < 20))
      GPIOC->BSRR |= 15;

    else if(distancia_objeto < 10)
      all_leds_on();

  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Configurar el boton "Seleccionar modo" en entrada digital, el cual corresponde al pin PC13.

   GPIOC->MODER &= ~(1<<26);
   GPIOC->MODER &= ~(1<<27);

   //Ahora configuramos en salida digital el led de la placa, que es el pin PA5:

   GPIOA->MODER |= (1<<10);
   GPIOA->MODER &= ~(1<<11);

   //Configuracion del boton de inicio de medida, el pin PB4 como entrada digital:

   GPIOB->MODER &= ~(1<<8);
   GPIOB->MODER &= ~(1<<9);

   //Hay que configurarlo en pull-up:

   GPIOB->PUPDR |= (1<<8);
   GPIOB->PUPDR &= ~(1<<9);

   //Configuramos el led indicador de "midiendo distancia" como salida digital, con pin PB5:

   GPIOB->MODER &= ~(1<<11);
   GPIOB->MODER |= (1<<10);

   //Configuracion de los pines PC0,PC1,PC2,PC3,PC4 (LEDs) en salida digital:

   GPIOC->MODER = (GPIOC->MODER & ~(0x03FF)) | 0x0155;

   //Configuramos el pin PA1 en modo analogico ya que corresponde al potenciómetro:

   GPIOA->MODER |= (3<<2);

   //Configuracion del pin PC8 en salida digital para el trigger:

   GPIOC->MODER &= ~(1<<17);
   GPIOC->MODER |= (1<<16);

   //Configuracion del pin PC9 en FUNCION ALTERNA para el ECHO:

   GPIOC->MODER |= (2<<19);

   //Configuracion del pin PA0 en FUNCION ALTERNA para el Servomotor:

   GPIOA->MODER |= 2;

   //Configuracion de interrupciones:

   SYSCFG->EXTICR[3] |= (2<<4);
   EXTI->IMR |= (1<<13);

   //Queremos que nos avise cuando caiga el voltaje, que es al apretar el boton
   EXTI->FTSR |= (1<<13);
   EXTI->RTSR &= ~(1<<13);

   SYSCFG->EXTICR[1] |= 1;
   EXTI->IMR |= (1<<4);
   EXTI->FTSR |= (1<<4);
   EXTI->RTSR &= ~(1<<4);

   //Configuracion del ADC:

   ADC1->CR2 &= ~1;   //ADON=0 => Apagar el conversor

   //Ahora configuramos la resolucion poniendo a 00 (12 bits) el RES[1:0]
   ADC1->CR1 &= ~(1<<24);
   ADC1->CR1 &= ~(1<<25);

   //Habilitamos la interrupcion por fin de conversion (EOCIE):

   ADC1->CR1 |= (1<<5);

   //Configuramos el SCAN a 0, ya que solo tenemos que convertir en un canal:

   ADC1->CR1 &= ~(1<<9);

   ADC1->CR2 |= 2; //Ponemos a 1 el bit de la posicion 1 del CR2 (CONT, conversion continua)
   ADC1->CR2 &= ~(1<<11); //Alineamos los datos a la derecha (ALIGN)

   //Configuracion del retardo entre conversiones: ponemos el DELS a 1 para que espere a que leamos el dato guardado en ADR1->DR
   ADC1->CR2 |= (1<<4);
   ADC1->CR2 &= ~(1<<5);
   ADC1->CR2 &= ~(1<<6);

   //Configuramos el SQR para que el ADC sepa cuantos y qué canales tiene que convertir a digital:

   ADC1->SQR1 &= ~(31<<20); //ponemos a 0 los bits 24:20 que corresponden al "Regular channel sequence lenght".
   ADC1->SQR5 = 1; //no hace falta mascara

   ADC1->CR2 |= 1;  //Encendemos ahora el conversor poiendo a 1 el ADON

   while((ADC1->SR &(1<<6))==0); //Espera activa mientras ADONS esta a 0, lo que implica que espera mientras el ADC no esté disponible para comenzar otra conversion

   ADC1->CR2 |= (1<<30); //SWSTART a 1 para que comience la conversion

   //Configuracion del TIM2 para el pin PA0 del servomotor:

   TIM2->CR1=0;
   TIM2->CR2=0;
   TIM2->SMCR=0;

   TIM2->CNT=0;
   TIM2->PSC=31; //Preescalado a 31 ya que queremos que la unidad del CNT sea de µs
   TIM2->ARR=20000; //Valor del periodod de la señal PWM
   TIM2->CCR1=500; //El CCR a 0 inicialmente ya que el servo solo puede girar cuando se gire el potenciometro o mida en modo automático

   TIM2->CCMR1 &=~3; //Configuramos el timer en modo PWM (00 como el TOC)
   TIM2->CCMR1 |= (1<<3); //Preload
   TIM2->CCMR1 |= (6<<4); //Nivel alto

   TIM2->CR1|=(1<<7); //(ARPE a 1)

   TIM2->EGR|=1;
   TIM2->SR=0;
   TIM2->CCER|=1;

   TIM2->CR1|=1; //Encendemos el timer, para que el servo reciba la señal constantemente.

   //Configuracion del TIM4 para los segundos que tarda en cada medida del modo automático y para el medio segundo del led "midiendo distancia":

  TIM4->CR1=0;
  TIM4->CR2=0;
  TIM4->SMCR=0;

  TIM4->CNT=0;
  TIM4->PSC=31999; //Preescalado
  TIM4->ARR=65535; //Limite del CNT
  TIM4->CCR1=2000; //Salta la interrupción cuando CNT==CCR1 (2000 ms <=> 2 s)
  TIM4->CCR2 = 500; //Salta la interrupción cuando CNT==CCR2 (500 ms <=> 0.5 s)

  TIM4->DIER= 2|(1 << 2);  //Habilitamos la interrupcion para el canal 1 y 2

  TIM4->CCMR1 &= ~3; //Configuramos el canal 1 y 2 en modo Output Compare
  TIM4->CCMR1 &= ~(3 << 8);

  TIM4->EGR|=1;
  TIM4->SR=0;

  //Configuracion de los timers para el trigger y el echo del sensor de ultrasonidos:

  TIM3->CR1=0;
  TIM3->CR2=0;
  TIM3->SMCR=0;

  TIM3->CNT=0;
  TIM3->PSC=31; // Como queremos contar cada 1µs hasta los 10µs
  TIM3->ARR=65535;
  TIM3->CCR1=10; //La interrupcion del canal 1 se activa cuando CNT llega a los 10µs

  //Configuramos ahora el canal 1 del TIM3 en TOC para que nos avise cuando CNT llegue a CCR1 (10µs):
  TIM3->CCMR1 &= ~(3);

  /* Configuramos el canal 4 en modo Input Compare (TIC) de modo que cuando ECHO cambie de estado el CCR4
   se actualizará al valor de CNT que tenga cuando ocurra el evento:
  */
  TIM3->CCMR2 &= ~(1<<9);
  TIM3->CCMR2 |= (1<<8);
  TIM3->CCMR2 &= ~(15<<11); // OC4M y OC4PE a 0

  //Seleccionamos el flanco de bajada y de subida en el CCER (CC4P a 1 y CC4NP a 1)
  TIM3->CCER |= (1<<13);
  TIM3->CCER |= (1<<15);
  TIM3->CCER |=(1<<12);

  TIM3->DIER |= 2; //Habilitamos la interrupción del canal 1
  TIM3->DIER |= (1<<4);

  //Ahora hay que configurar las NVICs para todas las interrupciones:

  NVIC->ISER[1]|= (1<<8);  // NVIC para la interrupcion habilitada por el PC13.
  NVIC->ISER[0]|= (1<<10); //NVIC para la interrupcion habilitada por PB4.
  NVIC->ISER[0]|= (1<<18); //NVIC para la interrupcion habilitada por el EOCIE.
  NVIC->ISER[0]|= (1<<29); //NVIC de la interrupcion habilitada por el TIM3->DIER
  NVIC->ISER[0]|= (1<<30); //NVIC de la interrupcion habilitada por el TIM4->DIER


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   change_mode=0;
    mode=AUTOMATICO;
    uart_print("AUTOMATIC MODE BY DEFAULT\r\n");
    led_verde_on();
    pulsador_2=0;
    counter_medidas=-1;

    while (1){

        if(change_mode!=0){ //Cambiando el modo (Automático/Manual)

          change_mode=0;

          if(mode==MANUAL){
            led_verde_on();
            mode=AUTOMATICO;
            uart_print("AUTOMATIC MODE ACTIVATED\r\n");
          }
          else{
            led_verde_off();
            mode=MANUAL;
            uart_print("MANUAL MODE ACTIVATED\r\n");
          }

        }

        if(mode==AUTOMATICO){

          if((ADC1->CR2 & 2)!=0) //Si el modo continuo esta activado, lo desactivamos para que no siga convirtiendo el voltaje del potenciometro, ya que no nos sirve
            ADC1->CR2 &= ~2;

          if((pulsador_2!=0) || ((counter_medidas >= 0)&&(counter_medidas < 5))){
            pulsador_2=0;

            if(counter_medidas==0) {

              //Se inicializa las distancias a -1 para el manejo en caso de que no se haya tomado una o varias de ellas
              distancias_auto[0] = -1;
              distancias_auto[1] = -1;
              distancias_auto[2] = -1;
              distancias_auto[3] = -1;
              distancias_auto[4] = -1;

              uart_print("Executing sequence of distance measurements\r\n");

            }

            medir_distancia_auto();

          }
          conversor_tiempo_distancia();

          resultado_medidas_LED();

        }

        else if(mode==MANUAL){

          if((ADC1->CR2 & 2)==0){//Si el modo continuo esta desactivado, lo activamos para que siga convirtiendo el voltaje del potenciometro
            ADC1->CR2 |= 2;
            ADC1->CR2 |= (1<<30);
          }

          if(pulsador_2!=0)
            medir_distancia_manual();

          conversor_tiempo_distancia();

          resultado_medidas_LED();

          medir_posicion();

        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

