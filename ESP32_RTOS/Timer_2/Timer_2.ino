static TimerHandle_t one_shot_timer = NULL;
static TimerHandle_t auto_reload_timer = NULL;

void myTimerCallback(TimerHandle_t xTimer) {
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 0) {
    Serial.println("One-shot timer expired");
  }
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 1) {
    Serial.println("Auto-reload timer expired");
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  one_shot_timer = xTimerCreate("One-shot timer", 2000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, myTimerCallback);
  auto_reload_timer = xTimerCreate("Auto-reload timer", 1000 / portTICK_PERIOD_MS, pdTRUE, (void *)1, myTimerCallback);

  if (one_shot_timer == NULL || auto_reload_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers...");
    xTimerStart(one_shot_timer, portMAX_DELAY);
    xTimerStart(auto_reload_timer, portMAX_DELAY);
  }

  vTaskDelete(NULL);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}