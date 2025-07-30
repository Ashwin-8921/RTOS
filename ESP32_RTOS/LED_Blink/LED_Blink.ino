
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

const int LED_PIN=2;

void task1(void *parameters)
{
  while(1)
  {
   digitalWrite(LED_PIN,HIGH);
   vTaskDelay(500 / portTICK_PERIOD_MS);
   digitalWrite(LED_PIN,LOW);
   vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void task2(void *parameters)
{
  while(1)
  {
   digitalWrite(LED_PIN,HIGH);
   vTaskDelay(323 / portTICK_PERIOD_MS);
   digitalWrite(LED_PIN,LOW);
   vTaskDelay(323 / portTICK_PERIOD_MS);
  }
}




void setup() {
  // put your setup code here, to run once:

  pinMode(LED_PIN,OUTPUT);
  xTaskCreatePinnedToCore(
    task1,
    "Task1",
    1000,
    NULL,
    1,
    NULL,
    app_cpu );
  xTaskCreatePinnedToCore(
    task2,
    "Task2",
    1000,
    NULL,
    1,
    NULL,
    app_cpu );

}

void loop() {
  // put your main code here, to run repeatedly:

}
