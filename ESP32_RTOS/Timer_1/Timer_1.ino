#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"



#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static TimerHandle_t one_shot_timer = NULL;

void myTimerCallback(TimerHandle_t xTimer) {
  Serial.println("Timer expired");
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Timer Demo---");

  one_shot_timer = xTimerCreate(
    "One-shot timer",
    2000 / portTICK_PERIOD_MS,
    pdFALSE,
    (void *)0,
    myTimerCallback
  );

  if (one_shot_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers...");
    xTimerStart(one_shot_timer, portMAX_DELAY);
  }

  vTaskDelete(NULL);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
