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
/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "hw_include/hw_memmap.h"
#include "hw_include/hw_timer.h"
#include "hw_include/timer.h"
#include "task.h"
#include "queue.h"

/* Delay between cycles of the 'check' task. */
#define CENSADO_DELAY		( ( TickType_t ) 100 / portTICK_PERIOD_MS )
#define TOP_DELAY		( ( TickType_t ) 1000 / portTICK_PERIOD_MS )
#define N_MAX                   20
#define VALORES_X               85

/* UART configuracion. */
#define mainBAUD_RATE		( 19200 )

/* Prioridades. */
#define mainCHECK_TASK_PRIORITY	( tskIDLE_PRIORITY + 3 )

/* Misc. */
#define mainQUEUE_SIZE		( 3 )

static void prvIniciarHardware( void );
/*---------------------------- Funciones -------------------------------------*/
int dObtenerN(unsigned short int);
void vGraficarEjes(void);
void vMandarStringUART(const char*);
uint32_t ulRandom( void );
static char* sObtenerCaracter(int );
static int dObtenerFila(int );
void vImprimirEstatusDeEjecucion(void);
static void vPushArreglo(int arreglo[], int valor,int tam_arreglo);
static int dCalcularPromedio(int arreglo[],int ventana,int tam_arreglo);
/*------------------------------ Tareas --------------------------------------*/
static void vSensorTemperatura( void *pvParameters );
static void vGenerarPromedio( void *pvParameters );
static void vGraficar(void *pvParameters);
static void vTop(void *pvParameter);
/*----------------------------------------------------------------------------*/
static unsigned int temperatura_actual = 15;
static uint32_t _dwRandNext=1 ;
unsigned long runtime_contador = 0;

TaskStatus_t *pxTaskStatusArray;

/* Colas de mensajes */
QueueHandle_t xCensadoQueue;
QueueHandle_t xPromedioQueue;

/*-----------------------------------------------------------*/
int main( void )
{
  /* Configuro UART y Display. */
  prvIniciarHardware();

  /* Para sincrozar las tareas se usan 2 colas de mensajes. */
  xCensadoQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int) );
  xPromedioQueue = xQueueCreate( mainQUEUE_SIZE, sizeof(int) );

  /* 
   * Aca se da inicio a las tareas de la consigna.
   * 1. Una tarea que simule un sensor de temperatura. Generando valores aleato-
   * rios, con una frecuencia de 10 Hz. (Sensor)
   * 
   * 2. Una tarea que reciba los valores del sensor y aplique un filtro pasa bajos.
   * Donde cada valor resultante es el promedio de las ultimas N mediciones. (Media)
   * 
   * 3. Una tarea que grafica en el display los valores de temperatura en el tiempo. (Grafico)
   * 
   * 6. Implementar una tarea tipo top de linux, que muestre periódicamente
   * estadı́sticas de las tareas (uso de cpu, uso de memoria, etc). (Top)
   */
  xTaskCreate( vSensorTemperatura, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY + 1, NULL );
  xTaskCreate( vGenerarPromedio, "Media", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY , NULL );
  xTaskCreate( vGraficar, "Grafico", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
  xTaskCreate( vTop, "Top", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );

  /* Arranca el Scheduler a Schedulear. */
  vTaskStartScheduler();

  return 0;
}
/*--------------------------------- Tareas -----------------------------------*/

static void vSensorTemperatura( void *pvParameters )
{
  /* Para ejecutar esta tarea con el periodo correcto se usa el sig parametro*/
  TickType_t xLastExecutionTime;
  xLastExecutionTime = xTaskGetTickCount();

  /* En cada tarea se hace este arreglo para asegurarse que no se viole el stack*/ 
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark < 1)
    while (1) {}

  for (;;)
  {
    /* Se genera una decision binaria para aumentar o disminuir */
    if(ulRandom()%2)
    {
      if(temperatura_actual < 30)
        temperatura_actual++;
    }
    else
    {
      if(temperatura_actual > 0)
        temperatura_actual--;
    }
    
    /* Envio el valor numerico para el generador de promedios*/
    xQueueSend(xCensadoQueue, &temperatura_actual, portMAX_DELAY );

    /* Me aseguro que una vez ejecutada la Tarea no vuelva haber problemas de stack*/
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
      while (1) {}

    /* Duermo exactamente 10hz, la funcion esta hace exactamente eso*/
    vTaskDelayUntil( &xLastExecutionTime, CENSADO_DELAY );
  }
}

static void vGenerarPromedio(void *pvParameters)
{
  int arreglo[N_MAX] = {}
    ,valor_censado
    ,promedio;
  unsigned short int N = 10;

  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark <1)
    while (1) { }

  for(;;)
  {
    /* El valor enviado del censor llega aca*/
    xQueueReceive( xCensadoQueue, &valor_censado, portMAX_DELAY);
    /* La funcion pushea el valor ingresado y descartando el primero*/
    vPushArreglo(arreglo,valor_censado,N_MAX);

    N = dObtenerN(N);

    /* Genera el promedio de los ultimos N valores*/
    promedio = dCalcularPromedio(arreglo,N,N_MAX);

    /* El valor del promedio se envia al graficador */
    xQueueSend(xPromedioQueue, &promedio, portMAX_DELAY );

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
      while (1) {}
  }
}


static void vGraficar(void *pvParameters)
{
  /* Uso el arreglo para graficar y uso la cantidad columnas del display */
  int arreglo[VALORES_X] = {}
  , promedio;

  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark < 1)
    while (1) {}

  for(;;)
  {
    /* Obtengo el promedio y lo guardo en el arreglo tambien pusheando*/
    xQueueReceive( xPromedioQueue, &promedio, portMAX_DELAY);
    vPushArreglo(arreglo,promedio,VALORES_X);

    OSRAMClear();
    //96 columnas de ancho
    // Graficos los ejes con los limites
    vGraficarEjes();

   /* Genero los puntos guardados en el arreglo */
    for (int i = 0; i < VALORES_X; i++)
      OSRAMImageDraw(sObtenerCaracter(arreglo[i]),i+11,dObtenerFila(arreglo[i]),1,1);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
      while (1) { }
  }
}

static void vTop(void *pvParameter)
{
  UBaseType_t uxHighWaterMark
              ,uxArraySize;

  uxArraySize = uxTaskGetNumberOfTasks();
  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  if(uxHighWaterMark < 1)
    while (1) {}

  for(;;)
  {
    /* Escribo por el UART */
    vImprimirEstatusDeEjecucion();

    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if(uxHighWaterMark < 1)
      while (1) {}

    vTaskDelay(TOP_DELAY);
  }

}

/*---------------------------------- Config ----------------------------------*/
/*
 * Prendo el UART arranco el display
 */
static void prvIniciarHardware( void )
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

  OSRAMInit( false );
}
/*- ------------------------------ Funciones ---------------------------------*/
/*
 * Recibe un valor y un arreglo y coloca el valor de este al final,
 * descartando el primero
 * similar a una operacion Shift Left
 */
static void vPushArreglo(int arreglo[], int valor,int tam_arreglo)
{
  for(int i=0; i < tam_arreglo-1; i++)
  {
    arreglo[i] = arreglo[i+1];
  }

  arreglo[tam_arreglo-1] = valor;
}
/*
 * Genera el promedio de los ultimos N elementos de un arregloz
 */
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
/*
 * Segun el valor a graficar devuelve un caracter que su valor en UTF-8 es 
 * valor en binario de la columna que quiero graficar
 */
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
/*
 * Elijo entre el primer y segundo reglon del display
 */
static int dObtenerFila(int valor)
{
  if(valor > 16)
    return 0;

  return 1;
}
/*
 * Recibe el valor actual de N, si este cambia por una entrada de uart valida
 */
int dObtenerN(unsigned short int N)
{
  if(UARTCharsAvail(UART0_BASE))
  {
    char caracter
      ,N_nuevo[3];
    int i=0
      ,nuevo_N=0;

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
  return N;
}

/*
 * Dibujo los ejes y los limites verticales
 */

void vGraficarEjes(void)
{
  OSRAMImageDraw("",9,0,2,1); //eje Y
  OSRAMImageDraw("",9,1,2,1);

  for (int i = 0; i < VALORES_X ; i++)//eje x
    OSRAMImageDraw("@",i+11,1,2,1);

  OSRAMImageDraw("",0,0,3,1); //Numero 3
  OSRAMImageDraw("",4,0,4,1);//Numero 0
  OSRAMImageDraw("8DD8",4,1,4,1);//Numero 0 inferior

}
/*
 * Esta funcion es una adaptacion de la brindada por la documentacion
 * pero no usa las fuciones de la lib std, en lugar de eso envia los strings
 * por la UART directamente
 */
void vImprimirEstatusDeEjecucion( void )
{
  volatile UBaseType_t uxArraySize, x;
  unsigned long ulTotalRunTime, ulStatsAsPercentage;

  if( pxTaskStatusArray != NULL )
  {
    uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
        uxArraySize,
        &ulTotalRunTime );

    ulTotalRunTime /= 100UL;

    vMandarStringUART("\r");
    if( ulTotalRunTime > 0 )
    {
      vMandarStringUART("TAREA\t|TICKS\t|CPU%\t|STACK FREE\r\n");
      for( x = 0; x < uxArraySize; x++ )
      {
        ulStatsAsPercentage = 
          pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

        char counter[8],porcentaje[8],stack[8];
        vMandarStringUART(pxTaskStatusArray[ x ].pcTaskName);
        vMandarStringUART("\t|");
        utoa(pxTaskStatusArray[ x ].ulRunTimeCounter,counter,10);
        vMandarStringUART(counter);
        vMandarStringUART("\t|");

        if( ulStatsAsPercentage > 0UL )
        {
          utoa(ulStatsAsPercentage,porcentaje,10);
          vMandarStringUART(porcentaje);
        }
        else
        {
          vMandarStringUART("0");
        }
        vMandarStringUART("%\t|");
        utoa(pxTaskStatusArray[ x ].usStackHighWaterMark,stack,10);
        vMandarStringUART(stack);
        vMandarStringUART(" Words\r\n");
      }

      /* Genera la ilusion de clear */
      vMandarStringUART("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n \
          \r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");
    }
  }
}
/*
 * Abstrae el hecho de que se pueden mandar solamente caracteres
 */
void vMandarStringUART(const char *mensaje)
{
  while(*mensaje != '\0')
  {
    UARTCharPut(UART0_BASE,*mensaje);
    mensaje++;
  }
  UARTCharPut(UART0_BASE,'\0');
  return;
}

/*
 * Funcion sacada de domentacion que genera numeros aleatorios
 */
uint32_t ulRandom( void )
{
  _dwRandNext = _dwRandNext * 1103515245 + 12345 ;

  return (uint32_t)(_dwRandNext/131072) % 65536 ;
}
/*----------------------------- Timer ----------------------------------------*/
void Timer0IntHandler(void)
{
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  runtime_contador++;
}

void configureTimer(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  IntMasterEnable();
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerConfigure(TIMER0_BASE,TIMER_CFG_32_BIT_TIMER);
  /* Bajar el 1500 aumenta la precision del calculo */
  TimerLoadSet(TIMER0_BASE, TIMER_A, 1500);
  TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler);
  TimerEnable(TIMER0_BASE,TIMER_A);
}

unsigned long get_value(void)
{
  return  runtime_contador;
}
/*----------------------------------------------------------------------------*/
