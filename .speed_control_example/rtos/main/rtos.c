// /* Hello World Example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "driver/gpio.h"
// #define BLINK_GPIO 2
// //#define BLINK_GPIO2 5
// #include "driver/ledc.h"
// #include "esp_err.h"
// #include "driver/touch_pad.h"

// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (2) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz



// void tp_example_read_task(void *pvParameter)
// {
//     touch_pad_init();
//   while (1) {
//       uint16_t touch_value;

//       ESP_ERROR_CHECK(touch_pad_read(TOUCH_PAD_NUM0, &touch_value));
//       printf("T:%4d \n", touch_value);
//       vTaskDelay(500 / portTICK_PERIOD_MS);
//   }
// }




// void ledc_task(void *pvParameter)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .timer_num        = LEDC_TIMER,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .timer_sel      = LEDC_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .duty           = 0, // Set duty to 0%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));


//  while(1){
//     for(int i=0; i<8000; i++){
//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,i));
//     ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//     vTaskDelay(10/ portTICK_PERIOD_MS);
//     //printf("%d\n",i);
//     // Update duty to apply the new value
//     }
//  }
// }


// void hello_task(void *pvParameter)
// {
//     printf("Hello world!\n");
//     for (int i = 10; i >= 0; i--) {
//         printf("Restarting in %d seconds...\n", i);
//         vTaskDelay(1000 / portTICK_RATE_MS);
//     }
//     // printf("Restarting now.\n");
//     // fflush(stdout);
//     // esp_restart();
// }

// void led_task(void *pvParameter)
// {
//     gpio_pad_select_gpio(BLINK_GPIO);
//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
//     while(1) {
//         /* Blink off (output low) */
//         gpio_set_level(BLINK_GPIO, 0);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//         /* Blink on (output high) */
//         gpio_set_level(BLINK_GPIO, 1);
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }


// // void led2_task(void *pvParameter)
// // {
// //     gpio_pad_select_gpio(BLINK_GPIO2);
// //     /* Set the GPIO as a push/pull output */
// //     gpio_set_direction(BLINK_GPIO2, GPIO_MODE_OUTPUT);
// //     while(1) {
// //         /* Blink off (output low) */
// //         gpio_set_level(BLINK_GPIO2, 0);
// //         vTaskDelay(500 / portTICK_PERIOD_MS);
// //         /* Blink on (output high) */
// //         gpio_set_level(BLINK_GPIO2, 1);
// //         vTaskDelay(500 / portTICK_PERIOD_MS);
// //     }
// // }
// void app_main()
// {

//   //xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 10000, NULL, 5, NULL);
//   xTaskCreate(&ledc_task, "ledc_task",2048, NULL, 12, NULL);
//   //xTaskCreate(&hello_task, "hello_task", 2048, NULL, 6, NULL);
//   //xTaskCreate(&led_task, "led_task", 512, NULL, 5, NULL);
//  //xTaskCreate(&led2_task, "blink_task", 512, NULL, 5, NULL);
//  // xTaskCreatePinnedToCore(&ledc_task, "ledc_task",10000, NULL, 5, NULL, 0);
//  // xTaskCreatePinnedToCore(&led_task, "led_task", 512, NULL, 5, NULL, 1);
  
// //     touch_pad_init();
// //   while (1) {
// //       uint16_t touch_value;

// //       ESP_ERROR_CHECK(touch_pad_read(TOUCH_PAD_NUM0, &touch_value));
// //       printf("T:%4d \n", touch_value);
// //       vTaskDelay(500 / portTICK_PERIOD_MS);
// //   }
// }

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"

// #define LED_PIN 2
// #define INTERRUPT_PIN 13

// bool ledState = false;

// void IRAM_ATTR isr_handler(void *arg) {
//     ledState = !ledState;
//     gpio_set_level(LED_PIN, ledState);
// }

// void setup_interrupt() {
//     gpio_set_intr_type(INTERRUPT_PIN, GPIO_INTR_POSEDGE);
//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(INTERRUPT_PIN, isr_handler, NULL);
// }

// void app_main() {
//     gpio_pad_select_gpio(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     gpio_pad_select_gpio(INTERRUPT_PIN);
//     gpio_set_direction(INTERRUPT_PIN, GPIO_MODE_INPUT);

//     setup_interrupt();

//     while (true) {
//         vTaskDelay(100 / portTICK_RATE_MS);
//          printf("H   ");
//     }
// }

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"

// #define LED_GPIO    2
// #define BUTTON_GPIO 4

// void IRAM_ATTR button_isr_handler(void* arg)
// {
//     uint32_t gpio_num = (uint32_t) arg;
//     gpio_set_level(LED_GPIO, 1);
     
// }

// void app_main(void)
// {
//     gpio_pad_select_gpio(LED_GPIO);
//     gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

//     gpio_pad_select_gpio(BUTTON_GPIO);
//     gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
//     gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_NEGEDGE);
//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, (void*) BUTTON_GPIO);

//     while(1) {
//           gpio_set_level(LED_GPIO, 0);
//           printf("H   ");
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }



// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "driver/gpio.h"

// #define INPUT_PIN 4
// #define LED_PIN 2

// int state = 0;
// xQueueHandle interputQueue;

// static void IRAM_ATTR gpio_interrupt_handler(void *args)
// {
//     int pinNumber = (int)args;
//     xQueueSendFromISR(interputQueue, &pinNumber, NULL);
// }

// void LED_Control_Task(void *params)
// {
//     int pinNumber, count = 0;
//     while (true)
//     {
//         if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
//         {
//             printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count++, gpio_get_level(INPUT_PIN));
//             gpio_set_level(LED_PIN, gpio_get_level(INPUT_PIN));
//         }
//     }
// }

// void app_main()
// {
//     gpio_pad_select_gpio(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     gpio_pad_select_gpio(INPUT_PIN);
//     gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
//     gpio_pulldown_en(INPUT_PIN);
//     gpio_pullup_dis(INPUT_PIN);
//     gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);

//     interputQueue = xQueueCreate(10, sizeof(int));
//     xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);

//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
// }



#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#define INPUT_PIN 13
#define LED_PIN 25

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (25) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

int state = 0;
xQueueHandle interputQueue;


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}


void zero_crossing()
{
  int chop_time = (180);// 5-180 dải xung( min 180- max 5)
  vTaskDelay(chop_time/ portTICK_RATE_MS);
  gpio_set_level(LED_PIN, 1);
  vTaskDelay(20/ portTICK_RATE_MS );
  gpio_set_level(LED_PIN, 0);
}

void LED_Control_Task(void *params)
{
    int pinNumber, count = 0;
    while (true)
    {
         if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        // {
            zero_crossing();
    }
}

void app_main()
{
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
}