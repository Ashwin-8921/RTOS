#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static char *msg_ptr = NULL;
static volatile uint8_t flag = 0;
const int buf_len = 100;

void readserial(void *parameters) {
  char buf[buf_len];
  uint8_t i = 0;
  char c;

  while (1) {
    memset(buf, 0, buf_len);
    i = 0;

    while (Serial.available()) {
      c = Serial.read();
      if (c == '\n') {
        if (flag == 0) {
          msg_ptr = (char *)malloc(i + 1);
          configASSERT(msg_ptr);
          memcpy(msg_ptr, buf, i);
          msg_ptr[i] = '\0'; // Null terminate
          flag = 1;
        }
        break;
      } else {
        if (i < buf_len - 1) {
          buf[i++] = c;
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void writeserial(void *parameters) {
  while (1) {
    if (flag == 1) {
      Serial.println(msg_ptr);
      Serial.print("Free heap (bytes): ");
      Serial.println(xPortGetFreeHeapSize());
      vPortFree(msg_ptr);
      msg_ptr = NULL;
      flag = 0;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Heap Demo---");
  Serial.println("Enter a string:");

  xTaskCreatePinnedToCore(readserial, "Read Serial", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(writeserial, "Write Message", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop() {
  
}
