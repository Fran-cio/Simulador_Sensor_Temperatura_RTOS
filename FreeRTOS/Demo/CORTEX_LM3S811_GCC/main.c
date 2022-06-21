/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "hw_include/hw_memmap.h"
#include "hw_include/hw_timer.h"
#include "hw_include/timer.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"

/* Delay between cycles of the 'check' task. */
#define CENSADO_DELAY						( ( TickType_t ) 100 / portTICK_PERIOD_MS )
#define TOP_DELAY						( ( TickType_t ) 1000 / portTICK_PERIOD_MS )
#define N_MAX 20
#define VALORES_X 85
#define TEMP_MAX 30
#define TEMP_MIN 0
#define RANGO_TEMP (TEMP_MAX-TEMP_MIN)

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Demo board specifics. */
#define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
/*
 * Configure the processor and peripherals for this demo.
 */
static void prvSetupHardware( void );

/*
 * The 'check' task, as described at the top of this file.
 */
static void vSensorTemperatura( void *pvParameters );
static void vGenerarPromedio( void *pvParameters );
static void vGraficar(void *pvParameters);
static char* sObtenerCaracter(int valor);
static int dObtenerFila(int valor);
static void vTop(void *pvParameter);
void vTaskGetRunTimeStatus(void);

TaskStatus_t *pxTaskStatusArray;
/*
 * The task that is woken by the ISR that processes GPIO interrupts originating
 * from the push button.
 */
static void vButtonHandlerTask( void *pvParameters );

/*
 * The task that controls access to the LCD.
 */
static void vPrintTask( void *pvParameter );

static void vPushArreglo(int arreglo[], int valor,int tam_arreglo);
static int dCalcularPromedio(int arreglo[],int ventana,int tam_arreglo);

/* String that is transmitted on the UART. */
static volatile char *pcNextChar;
static unsigned int temperatura_actual = 15;
static uint32_t _dwRandNext=1 ;

static unsigned int N=10;
unsigned long runtime_contador = 0;

/* The semaphore used to wake the button handler task from within the GPIO
interrupt handler. */
SemaphoreHandle_t xButtonSemaphore;

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xCensadoQueue;
QueueHandle_t xPromedioQueue;

/*-----------------------------------------------------------*/

extern uint32_t rando( void )
{
    _dwRandNext = _dwRandNext * 1103515245 + 12345 ;

    return (uint32_t)(_dwRandNext/131072) % 65536 ;
}

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the semaphore used to wake the button handler task from the GPIO
	ISR. */
	// vSemaphoreCreateBinary( xButtonSemaphore );
	// xSemaphoreTake( xButtonSemaphore, 0 );

	/* Create the queue used to pass message to vPrintTask. */
	xCensadoQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int) );
        xPromedioQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int) );

	/* Start the standard demo tasks. */

	/* Start the tasks defined within the file. */
	xTaskCreate( vSensorTemperatura, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );
	xTaskCreate( vGenerarPromedio, "Media", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY , NULL );
	xTaskCreate( vGraficar, "Grafico", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
        xTaskCreate( vTop, "Top", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}
/*-----------------------------------------------------------*/

static void vSensorTemperatura( void *pvParameters )
{
  portBASE_TYPE xErrorOccurred = pdFALSE;

  TickType_t xLastExecutionTime;
  /* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
     works correctly. */

  xLastExecutionTime = xTaskGetTickCount();

  UBaseType_t uxHighWaterMark;

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark < 1)
  {
    while (1) {
      
    }
  }
  for (;;)
  {

    /* Perform this check every mainCHECK_DELAY milliseconds. */

    /* Has an error been found in any task? */
    if(rando()%2)
    {
      if(temperatura_actual < 30)
      {
        temperatura_actual++;
      }
    }
    else
    {
      if(temperatura_actual > 0)
      {
        temperatura_actual--;
      }
    }
    xQueueSend(xCensadoQueue, &temperatura_actual, portMAX_DELAY );

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

    if(uxHighWaterMark < 1)
    {
      while (1) {

      }
    }
    vTaskDelayUntil( &xLastExecutionTime, CENSADO_DELAY );
  }
}

static void vGenerarPromedio(void *pvParameters)
{
  int arreglo[N_MAX] = {}
  ,valor_censado
    ,promedio;

  char N_nuevo[3];

  UBaseType_t uxHighWaterMark;

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

  if(uxHighWaterMark <1)
  {
    while (1) {

    }
  }
  for(;;)
  {
    xQueueReceive( xCensadoQueue, &valor_censado, portMAX_DELAY);

    if(UARTCharsAvail(UART0_BASE))
    {
      char caracter;
      int i=0;
      int nuevo_N=0;
      while((caracter =(char)UARTCharNonBlockingGet(UART0_BASE)) != -1)
      {
        N_nuevo[i] = caracter;
        if(i==2) break;
        i++;
      }
      N_nuevo[i] = '\0';
      nuevo_N = atoi(N_nuevo);
      if( atoi(N_nuevo) > 1 && atoi(N_nuevo) < 10 )
      {
        N = atoi(N_nuevo);
      }
    }
    vPushArreglo(arreglo,valor_censado,N_MAX);
    promedio = dCalcularPromedio(arreglo,N,N_MAX);

    xQueueSend(xPromedioQueue, &promedio, portMAX_DELAY );

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
    {
      while (1) {

      }
    }
  }
}
void vGraficarEjes(void)
{
  OSRAMImageDraw("",9,0,2,1); //eje Y
  OSRAMImageDraw("",9,1,2,1);
  for (int i = 0; i < VALORES_X ; i++)//eje x
  {
    OSRAMImageDraw("@",i+11,1,2,1);
  }
  OSRAMImageDraw("",0,0,3,1); //Numero 3
  OSRAMImageDraw("",4,0,4,1);//Numero 0
  OSRAMImageDraw("8DD8",4,1,4,1);//Numero 0 inferior

}

static void vGraficar(void *pvParameters)
{
  int arreglo[VALORES_X] = {}
  , promedio;
  UBaseType_t uxHighWaterMark;

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark <1)
    while (1) {}

  for(;;)
  {
    xQueueReceive( xPromedioQueue, &promedio, portMAX_DELAY);
    vPushArreglo(arreglo,promedio,VALORES_X);

    OSRAMClear();
    //96 columnas de ancho
    vGraficarEjes();
    for (int i = 0; i < VALORES_X; i++)
    {
      OSRAMImageDraw(sObtenerCaracter(arreglo[i]),i+11,dObtenerFila(arreglo[i]),1,1);
    }
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
      while (1) { }

  }
}

static void vTop(void *pvParameter)
{
  UBaseType_t uxHighWaterMark;
  volatile UBaseType_t uxArraySize, x;

  uxArraySize = uxTaskGetNumberOfTasks();
  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark < 1)
    while (1) {}

  for(;;)
  {
    vTaskGetRunTimeStatus();
    
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

    if(uxHighWaterMark < 1)
      while (1) {}

    vTaskDelay(TOP_DELAY);
  }
  
}

void vMandarStringUART(char *mensaje)
{
  while(*mensaje != NULL)
    {
      UARTCharPut(UART0_BASE,*mensaje);
      mensaje++;
    }
  UARTCharPut(UART0_BASE,'\0');
  return;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  /* Setup the PLL. */
  // SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );
  //
  // /* Setup the push button. */
  // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  //    GPIODirModeSet(GPIO_PORTC_BASE, mainPUSH_BUTTON, GPIO_DIR_MODE_IN);
  // GPIOIntTypeSet( GPIO_PORTC_BASE, mainPUSH_BUTTON,GPIO_FALLING_EDGE );
  // IntPrioritySet( INT_GPIOC, configKERNEL_INTERRUPT_PRIORITY );
  // GPIOPinIntEnable( GPIO_PORTC_BASE, mainPUSH_BUTTON );
  // IntEnable( INT_GPIOC );
  //
  //
  //
  // /* Enable the UART.  */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  //
  // /* Set GPIO A0 and A1 as peripheral function.  They are used to output the
  // UART signals. */
  // GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW );
  //
  // /* Configure the UART for 8-N-1 operation. */
  UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );
  //
  // /* We don't want to use the fifo.  This is for test purposes to generate
  // as many interrupts as possible. */
  HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;
  //
  // /* Enable Tx interrupts. */
  HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_TX;
  IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
  IntEnable( INT_UART0 );
  //

  /* Initialise the LCD> */
  OSRAMInit( false );
}
/*-----------------------------------------------------------*/
void vUART_ISR(void)
{
  unsigned long ulStatus;

  /* What caused the interrupt. */
  ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

  /* Clear the interrupt. */
  UARTIntClear( UART0_BASE, ulStatus );

  /* Was a Tx interrupt pending? */
  if( ulStatus & UART_INT_TX )
  {
    /* Send the next character in the string.  We are not using the FIFO. */
    if( *pcNextChar != 0 )
    {
      if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
      {
        HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
      }
      pcNextChar++;
    }
  }
}
/*-----------------------------------------------------------*/

void vGPIO_ISR( void )
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Clear the interrupt. */
  GPIOPinIntClear(GPIO_PORTC_BASE, mainPUSH_BUTTON);

  /* Wake the button handler task. */
  xSemaphoreGiveFromISR( xButtonSemaphore, &xHigherPriorityTaskWoken );

  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void vPushArreglo(int arreglo[], int valor,int tam_arreglo)
{
  for(int i=0; i < tam_arreglo-1; i++)
  {
    arreglo[i] = arreglo[i+1];
  }

  arreglo[tam_arreglo-1] = valor;
}

static int dCalcularPromedio(int arreglo[],int ventana,int tam_arreglo)
{
  int promedio=0;
  if (ventana > tam_arreglo)
  {
    while (1) {}
  }

  for (int i = 0; i < ventana; i++) 
  {
    promedio += arreglo[(tam_arreglo-1)-i];
  }

  return promedio/ventana;
}

static char* sObtenerCaracter(int valor)
{
  if(valor < 2)
    return "@";
  else if(valor < 4)
    return "`";
  else if( valor < 8)
    return "P";
  else if( valor < 10)
    return "H";
  else if(valor < 12)
    return "D";
  else if(valor < 14)
    return "B";
  else if( valor < 16)
    return "A";
  else if(valor <18)
    return "@";
  else if(valor <20)
    return " ";
  else if(valor <22)
    return "";
  else if(valor <24)
    return "";
  else if (valor <26)
    return "";
  else if(valor <28)
    return "";
  else if(valor <=30)
    return "";
}

static int dObtenerFila(int valor)
{
  if(valor > 16)
    return 0;

  return 1;
}

void vTaskGetRunTimeStatus( void )
{
volatile UBaseType_t uxArraySize, x;
unsigned long ulTotalRunTime, ulStatsAsPercentage;

   /* Make sure the write buffer does not contain a string. */

   /* Take a snapshot of the number of tasks in case it changes while this
   function is executing. */
   /* Allocate a TaskStatus_t structure for each task.  An array could be
   allocated statically at compile time. */

   if( pxTaskStatusArray != NULL )
   {
      /* Generate raw status information about each task. */
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

      /* For percentage calculations. */
      ulTotalRunTime /= 100UL;

      /* Avoid divide by zero errors. */
      vMandarStringUART("\r");
      if( ulTotalRunTime > 0 )
      {
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
          vMandarStringUART("TAREA\t|TICKS\t|CPU%\t|STACK FREE\r\n");
         for( x = 0; x < uxArraySize; x++ )
         {
            /* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage = 
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

              char counter[8],porcentaje[8],stack[8];
              vMandarStringUART(pxTaskStatusArray[ x ].pcTaskName);
              vMandarStringUART("\t|");
              utoa(pxTaskStatusArray[ x ].ulRunTimeCounter,counter,10);
              vMandarStringUART(counter);
              vMandarStringUART("\t|");
              // vMandarStringUART(" Porcentaje: ");

            if( ulStatsAsPercentage > 0UL )
            {
              utoa(ulStatsAsPercentage,porcentaje,10);
              vMandarStringUART(porcentaje);
            //    sprintf( pcWriteBuffer, "%stt%lutt%lu%%rn",           
            //                      pxTaskStatusArray[ x ].pcTaskName,
            //                      pxTaskStatusArray[ x ].ulRunTimeCounter,
            //                      ulStatsAsPercentage );
            }
            else
            {
              vMandarStringUART("0");
            //    sprintf( pcWriteBuffer, "%stt%lutt<1%%rn",
            //                      pxTaskStatusArray[ x ].pcTaskName,
            //                      pxTaskStatusArray[ x ].ulRunTimeCounter );
            }
            vMandarStringUART("%\t|");
            // vMandarStringUART("Uso de Stack: ");
            utoa(pxTaskStatusArray[ x ].usStackHighWaterMark,stack,10);
            vMandarStringUART(stack);
            vMandarStringUART(" Words\r\n");
         }
         vMandarStringUART("\r\n\r\n\r\n\r\n\r\n\r\n");
      }
   }
}
void Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    runtime_contador++;
}
void configureTimer(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerConfigure(TIMER0_BASE,TIMER_CFG_32_BIT_TIMER);
//    TimerLoadSet(TIMER0_BASE, TIMER_A, (configCPU_CLOCK_HZ / 10000UL ) - 1UL);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 1500);
    TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler);
    TimerEnable(TIMER0_BASE,TIMER_A);
}

unsigned long get_value(void)
{
//    return TimerValueGet(TIMER0_BASE,TIMER_A);
//    return SysTickValueGet();
    return  runtime_contador;
}
