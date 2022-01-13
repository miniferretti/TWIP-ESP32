#include "main.h"

BaseType_t xReturned;
TaskHandle_t screenOled_Handle = NULL;
TaskHandle_t vescControl_Handle = NULL;
TaskHandle_t break_Handle = NULL;
TaskHandle_t accel_Handle = NULL;
TaskHandle_t button_Handle = NULL;

const char *ssid = "Resaux Bruyere";
const char *password = "Bruyere55";
boolean OTA = false;

void setup()
{
  //OTA update setup
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println();
    Serial.println("WIFI connection failed");
    Serial.println("IP address: N/A");
    Serial.println("No WiFi working mode");
    delay(1000);
  }
  else
  {
    OTA = true;
    ArduinoOTA
        .onStart([]()
                 {
                   String type;
                   if (ArduinoOTA.getCommand() == U_FLASH)
                     type = "sketch";
                   else // U_SPIFFS
                     type = "filesystem";

                   // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                   Serial.println("Start updating " + type);
                 })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
                   Serial.printf("Error[%u]: ", error);
                   if (error == OTA_AUTH_ERROR)
                     Serial.println("Auth Failed");
                   else if (error == OTA_BEGIN_ERROR)
                     Serial.println("Begin Failed");
                   else if (error == OTA_CONNECT_ERROR)
                     Serial.println("Connect Failed");
                   else if (error == OTA_RECEIVE_ERROR)
                     Serial.println("Receive Failed");
                   else if (error == OTA_END_ERROR)
                     Serial.println("End Failed");
                 });

    ArduinoOTA.begin();
    Serial.println();
    Serial.println("WIFI connection ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("WiFi working mode");
  }

  EEPROM.begin(512);

  // Task creation of the controller
  vTaskDelay(500 / portTICK_PERIOD_MS);

  xReturned = xTaskCreatePinnedToCore(screenOled_Task, "Oled", 5120, NULL, 1, &screenOled_Handle, 0);
  if (xReturned == pdPASS)
  {
    Serial.println("\n\r");
    Serial.println("Task < " + String(pcTaskGetTaskName(screenOled_Handle)) + " > has been created !");
  }
  else
  {
    Serial.println("Failed with task creation, program exit....");
    exit(0);
  }

  xReturned = xTaskCreatePinnedToCore(vescControl_Task, "Vesc", 2048, NULL, 1, &vescControl_Handle, 1);
  if (xReturned == pdPASS)
  {
    Serial.println("Task < " + String(pcTaskGetTaskName(vescControl_Handle)) + " > has been created !");
  }
  else
  {
    Serial.println("Failed with task creation, program exit....");
    exit(0);
  }

  xReturned = xTaskCreatePinnedToCore(break_Task, "Break", 1024, NULL, 1, &break_Handle, 1);
  if (xReturned == pdPASS)
  {
    Serial.println("Task < " + String(pcTaskGetTaskName(break_Handle)) + " > has been created !");
  }
  else
  {
    Serial.println("Failed with task creation, program exit....");
    exit(0);
  }

  xReturned = xTaskCreatePinnedToCore(accel_Task, "Accel", 1024, NULL, 1, &accel_Handle, 1);
  if (xReturned == pdPASS)
  {
    Serial.println("Task < " + String(pcTaskGetTaskName(accel_Handle)) + " > has been created !");
  }
  else
  {
    Serial.println("Failed with task creation, program exit....");
    exit(0);
  }

  xReturned = xTaskCreatePinnedToCore(button_Task, "Button", 2048, NULL, 1, &button_Handle, 0);
  if (xReturned == pdPASS)
  {
    Serial.println("Task < " + String(pcTaskGetTaskName(button_Handle)) + " > has been created !");
  }
  else
  {
    Serial.println("Failed with task creation, program exit....");
    exit(0);
  }

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  Serial.println();
  Serial.println("TWIP scooter ready to mingle !");
}

void loop()
{
  if (OTA)
  {
    ArduinoOTA.handle();
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
}