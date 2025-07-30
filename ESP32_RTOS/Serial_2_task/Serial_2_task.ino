const char MSG[]="welcome to india";

static TaskHandle_t task1=NULL;
static TaskHandle_t task2=NULL;


void startTask1(void *parameters)
{
  int MSG_LEN=strlen(MSG);

  while(1)
  {
    for(int i=0;i<MSG_LEN;i++)
    {
      Serial.print(MSG[i]);
    }
    Serial.println();
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}

void startTask2(void *parameters)
{
   while(1)
  {
    Serial.print('*');
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
  
}



 
void setup() {

  Serial.begin(300);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("FreeRTOS");
  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(" with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  xTaskCreatePinnedToCore(
    startTask1,
    "Task 1",
    1024,
    NULL,
    1,
    &task1,
    1 );
  xTaskCreatePinnedToCore(
    startTask2,
    "Task 2",
    1024,
    NULL,
    2,
    &task2,
    1 );

}

void loop() {

  
  for (int i = 0; i < 3; i++) 
  {
    vTaskSuspend(task2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  if (task1 != NULL) 
  {
    vTaskDelete(task1);
    task1 = NULL;
  }
}
