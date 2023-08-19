#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#define LED_PIN 25
#define RED_LED 0

#define GPIO_ON     1
#define GPIO_OFF    0
char userInput;

void GreenLEDTask(void *param)
{
    while(1){
        //Get User Input
        // printf("(1 = on or 0 = off):\n");
        // userInput = getchar();

        // if(userInput == '1'){
        //     // Turn On LED
        //     gpio_put(25, 1); // Set pin 25 to high
        //     printf("LED switched on!\n");
        // }
        // else if(userInput == '0'){
        //     // Turn Off LED
        //     gpio_put(25, 0); // Set pin 25 to high.
        //     printf("LED switched off!\n");
        // }
        // else{
        //     printf("Invalid Input!\n");
        // }
        int j=0;
        gpio_put(25,1);
        printf("%d",get_core_num());
        for (int i=0;i<25000;i++){
          j++;
        }
        gpio_put(25,0);
        vTaskDelay(3000);

    }
}

void RedLEDTask(void *param)
{   
    while(1){
      int j=0;
      gpio_put(25,0);
      for (int i=0;i<25;i++){
        printf("LED switched on!\n");
        }
      printf("%d",get_core_num());
      vTaskDelay(3000);
  }
}

int main() 
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(RED_LED);
    gpio_set_dir(RED_LED, GPIO_OUT);

    TaskHandle_t gLEDtask = NULL;
    TaskHandle_t rLEDtask = NULL;

    uint32_t status = xTaskCreate(
                    GreenLEDTask,
                    "Green LED",
                    1024,
                    NULL,
                    6,
                    &gLEDtask);
                    //lower no is lower priority

    status = xTaskCreate(
                    RedLEDTask,
                    "Red LED",
                    1024,
                    NULL,
                    4,
                    &rLEDtask);

    vTaskStartScheduler();

    for( ;; )
    {
        //should never get here
    }

}
