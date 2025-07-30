#include<Arduino.h>

int count1=0;
int count2=0;


void task1(void *parameters)
{
  while(1)
  {
    Serial.print("Task 1 counter:");
    Serial.println(count1++);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}
void task2(void *parameters)
{
  while(1)
  {
    Serial.print("Task 2 counter:");
    Serial.println(count2++);
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  xTaskCreate(
    task1,
    "Task1",
    1000,
    NULL,
    1,
    NULL

  );
  
 xTaskCreate(
    task2,
    "Task2",
    1000,
    NULL,
    1,
    NULL

  );

}

void loop() {
  // put your main code here, to run repeatedly:

}
