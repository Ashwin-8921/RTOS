#include <stdlib.h>

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static const uint8_t buf_len = 20;
static const int led_pin = 2;
static int led_delay = 500;

void toggleLED(void *parameter) {
  while (1) 
  {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameters) {
  char c;
  char buf[buf_len];
  uint8_t i = 0;
  memset(buf, 0, buf_len);

  while (1) 
  {
    if (Serial.available() > 0) 
    {
      c = Serial.read();
      if (c == '\n') 
      {
        led_delay = atoi(buf);
        Serial.print("Updated LED delay to: ");
        Serial.println(led_delay);
        memset(buf, 0, buf_len);
        i = 0;
      } 
      else 
      {
        if (i < buf_len - 1) 
        {
          buf[i] = c;
          i++;
        }
      }
    }
  }
}

void setup() {
  pinMode(led_pin, OUTPUT);
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Multi-task LED Demo");
  Serial.println("Enter delay in milli seconds:");

  xTaskCreatePinnedToCore(
    toggleLED,
    "Toggle LED",
    1024,
    NULL,
    1,
    NULL,
    app_cpu);

  xTaskCreatePinnedToCore(
    readSerial,
    "Read Serial",
    1024,
    NULL,
    1,
    NULL,
    app_cpu);

  vTaskDelete(NULL);
}

void loop() {
}
