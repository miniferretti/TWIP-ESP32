#include "tasks.h"

QueueHandle_t button_queue;
QueueHandle_t accel_queue;
QueueHandle_t break_queue;
QueueHandle_t vescToScreen_queue;
QueueHandle_t screenToVesc_queue;

void screenOled_Task(void *paramter)
{
    Adafruit_SH1106 display = Adafruit_SH1106(SDA, SCL);
    dataPackage Vescdata;
    nunchuckPackage crusedata;
    stateMachine state = BOOT;
    screenToVesc_queue = xQueueCreate(SCREENTOVESC_QUEUE_LEN, sizeof(nunchuckPackage));

    uint32_t max_accel = EEPROM.readUInt(0);
    uint32_t min_accel = EEPROM.readUInt(4);
    uint32_t max_break = EEPROM.readUInt(8);
    uint32_t min_break = EEPROM.readUInt(12);

    uint32_t accel_val;
    uint32_t break_val;

    uint32_t but;

    SimpleKalmanFilter Filter1(2, 2, 0.01);
    float velocity = 0;
    float inpVoltage = 0;
    float distance = 0;
    float power = 0;
    float current = 0;
    float max_speed = 0;
    float max_power = 0;

    display.begin(SH1106_SWITCHCAPVCC, OLED_ADD);
    display.clearDisplay();
    display.display();

    while (1)
    {
        switch (state)
        {
        case BOOT:

            if (screenToVesc_queue != 0)
            {
                crusedata.lowerButton = false;
                crusedata.valueY = VESC_IDL;
                xQueueSend(screenToVesc_queue, (void *)&crusedata, 0);
            }

            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(2);
            display.drawLine(0, 20, 127, 20, WHITE);
            display.setCursor(40, 28);
            display.print("TWIP");
            display.drawLine(0, 50, 127, 50, WHITE);

            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print("IP:");
            if (WiFi.status() == WL_CONNECTED)
            {

                display.println(WiFi.localIP());
            }
            else
            {
                display.println("No WiFi");
            }

            display.print("V:");
            if (vescToScreen_queue != 0)
            {
                if (xQueueReceive(vescToScreen_queue, (void *)&Vescdata, 0) == pdPASS)
                {
                    display.print(Vescdata.inpVoltage);
                }
                else
                {
                    display.print("N/A");
                }
            }

            if (button_queue != 0)
            {
                if (xQueueReceive(button_queue, (void *)&but, 0) == pdPASS)
                {
                    state = CRUSE;
                }
            }

            display.setCursor(0, 55);
            display.print("Press button to Ride");

            break;

        case CRUSE:

            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print("Mode: Cruse");

            if (vescToScreen_queue != 0)
            {
                if (xQueueReceive(vescToScreen_queue, (void *)&Vescdata, 0) == pdPASS)
                {
                    velocity = -(Vescdata.rpm / POLE_PAIRS) * REDUCTION_RATIO * 3.14 * 2 * WHEEL_RAD * 3.6 / 60;
                    inpVoltage = Vescdata.inpVoltage;
                    distance = (Vescdata.tachometerAbs / 42) * 3.14 * 2 * WHEEL_RAD * REDUCTION_RATIO / 1000;
                    current = Vescdata.avgInputCurrent;
                    power = Filter1.updateEstimate(inpVoltage * current);

                    if (velocity > max_speed)
                        max_speed = velocity;
                    if (power > max_power)
                        max_power = power;
                }
                else
                {
                    velocity = 0;
                    inpVoltage = 0;
                    distance = 0;
                    current = 0;
                    power = 0;
                }
            }

            display.setCursor(0, 21);
            display.setTextSize(3);
            display.print(velocity, 1);
            display.setTextSize(1);
            display.print("Km/h");

            display.setCursor(0, 55);
            display.print("Km: ");
            display.print(distance, 1);

            display.setCursor(80, 55);
            display.print("I: ");
            display.print(current);

            display.setCursor(80, 0);
            display.print("Bat lvl");
            display.drawRect(108, 10, 10, 40, WHITE);
            display.fillRect(108, 10 + 40 - map(inpVoltage, MIN_BAT_VOLATGE, MAX_BAT_VOLTAGE, 0, 40), 10, map(inpVoltage, MIN_BAT_VOLATGE, MAX_BAT_VOLTAGE, 0, 40), WHITE);

            if (accel_queue != 0)
            {
                xQueueReceive(accel_queue, (void *)&accel_val, 0);
            }
            if (break_queue != 0)
            {
                xQueueReceive(break_queue, (void *)&break_val, 0);
            }

            if (accel_val > break_val)
            {
                crusedata.valueY = (uint8_t)map(accel_val, min_accel, max_accel, 128, VESC_MAX);
            }
            else if (break_val > min_break)
            {
                crusedata.valueY = 126 - (uint8_t)map(break_val, min_break, max_break, VESC_MIN, 126);
            }
            else
            {
                crusedata.valueY = VESC_IDL;
            }

            if (screenToVesc_queue != 0)
            {
                display.setCursor(55, 55);
                display.printf("%d%%", 1 + map(crusedata.valueY, 0, 255, -100, 100));
                crusedata.lowerButton = false;
                xQueueSend(screenToVesc_queue, (void *)&crusedata, 0);
            }

            if (button_queue != 0)
            {
                if (xQueueReceive(button_queue, (void *)&but, 0) == pdPASS)
                {
                    state = INFO;
                }
            }

            break;

        case ACCEL_BREAK_TEST:
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print("Mode: Input test");
            display.setCursor(0, 21);
            display.print("Accel: ");
            if (accel_queue != 0)
            {

                if (xQueueReceive(accel_queue, (void *)&accel_val, 0) == pdPASS)
                {
                    display.print(map(accel_val, min_accel, max_accel, 0, 100));
                    display.print("%");
                    display.drawRect(70, 21, 50, 10, WHITE);
                    display.fillRect(70, 21, map(accel_val, min_accel, max_accel, 0, 50), 10, WHITE);
                }
            }
            else
            {
                display.print("N/A");
            }
            display.setCursor(0, 35);
            display.print("Break: ");
            if (break_queue != 0)
            {

                if (xQueueReceive(break_queue, (void *)&break_val, 0) == pdPASS)
                {
                    display.print(map(break_val, min_break, max_break, 0, 100));
                    display.print("%");
                    display.drawRect(70, 35, 50, 10, WHITE);
                    display.fillRect(70, 35, map(break_val, min_break, max_break, 0, 50), 10, WHITE);
                }
            }
            else
            {
                display.print("N/A");
            }

            if (button_queue != 0)
            {

                if (xQueueReceive(button_queue, (void *)&but, 0) == pdPASS)
                {
                    if (but == 20)
                    {
                        state = CALIBRATION;
                    }
                    else
                    {
                        state = CRUSE;
                    }
                }
            }
            display.setCursor(0, 55);
            display.print("Long press for calib");

            if (screenToVesc_queue != 0)
            {
                crusedata.lowerButton = false;
                crusedata.valueY = VESC_IDL;
                xQueueSend(screenToVesc_queue, (void *)&crusedata, 0);
            }

            break;

        case CALIBRATION:

            if (screenToVesc_queue != 0)
            {
                crusedata.lowerButton = false;
                crusedata.valueY = VESC_IDL;
                xQueueSend(screenToVesc_queue, (void *)&crusedata, 0);
            }

            if (calibration_routine(&display, &max_accel, &min_accel, &max_break, &min_break))
            {
                state = CRUSE;
            };
            break;

        case INFO:
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setCursor(0, 0);
            display.println("Mode: INFO");
            display.println();

            if (vescToScreen_queue != 0)
            {
                if (xQueueReceive(vescToScreen_queue, (void *)&Vescdata, 0) == pdPASS)
                {
                    velocity = -(Vescdata.rpm / POLE_PAIRS) * REDUCTION_RATIO * 3.14 * 2 * WHEEL_RAD * 3.6 / 60;
                    inpVoltage = Vescdata.inpVoltage;
                    distance = (Vescdata.tachometerAbs / 42) * 3.14 * 2 * WHEEL_RAD * REDUCTION_RATIO / 1000;
                    current = Vescdata.avgInputCurrent;
                    power = Filter1.updateEstimate(inpVoltage * current);
                }
                else
                {
                    velocity = 0;
                    inpVoltage = 0;
                    distance = 0;
                    current = 0;
                    power = 0;
                }
            }
            display.printf("Trav dist = %0.1f \n", distance);
            display.printf("Max speed: %0.1f Km/h \n", max_speed);
            display.printf("Max power: %0.1f W\n", max_power);
            display.setCursor(0, 45);
            display.printf("Author: %s \n", FIRMWARE_CREATOR);
            display.setCursor(0, 55);
            display.printf("Version: V%d\n", FIRMWARE_VERSION);

            if (button_queue != 0)
            {

                if (xQueueReceive(button_queue, (void *)&but, 0) == pdPASS)
                {
                    state = ACCEL_BREAK_TEST;
                }
            }

            break;

        default:
            break;
        }

        display.display();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void vescControl_Task(void *parameter)
{
    vescToScreen_queue = xQueueCreate(VESCTOSCREEN_QUEUE_LEN, sizeof(dataPackage));
    VescUart UART;
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    while (!Serial2)
    {
        ;
    }

    UART.setSerialPort(&Serial2);

    while (1)
    {
        if (UART.getVescValues())
        {
            if (vescToScreen_queue != 0)
            {
                xQueueSend(vescToScreen_queue, (void *)&UART.data, 0);
            }
        }
        if (screenToVesc_queue != 0)
        {
            if (xQueueReceive(screenToVesc_queue, (void *)&UART.nunchuck, 0) == pdPASS)
            {
                UART.setNunchuckValues();
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void button_Task(void *parameter)
{
    button_queue = xQueueCreate(BUTTON_QUEUE_LEN, sizeof(uint32_t));
    pinMode(BUZZ_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    uint32_t static state = 0;
    boolean buzz = false;

    while (1)
    {
        if ((digitalRead(BUTTON_PIN) == HIGH) && (state < 20))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (digitalRead(BUTTON_PIN) == HIGH)
            {
                state++;
                if (!buzz)
                {
                    digitalWrite(BUZZ_PIN, HIGH);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    digitalWrite(BUZZ_PIN, LOW);
                }
                buzz = true;
            }
        }
        else
        {
            if (state > 0)
            {
                xQueueSend(button_queue, (void *)&state, 0);
                // Serial.println(state);
            }
            if (state == 20)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
            state = 0;
            buzz = false;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void break_Task(void *parameter)
{
    break_queue = xQueueCreate(BREAK_QUEUE_LEN, sizeof(uint32_t));
    pinMode(BREAK_PIN, INPUT);
    movingAvg mySensor(200);
    mySensor.begin();
    static uint32_t val;

    while (1)
    {
        // Serial.print("Break avg data is ");
        // Serial.println(mySensor.reading(analogRead(BREAK_PIN)));
        val = mySensor.reading(analogRead(BREAK_PIN));
        xQueueSend(break_queue, (void *)&val, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void accel_Task(void *parameter)
{
    accel_queue = xQueueCreate(ACCEL_QUEUE_LEN, sizeof(uint32_t));
    pinMode(ACCEL_PIN, INPUT);
    movingAvg mySensor(200);
    mySensor.begin();
    static uint32_t val;

    while (1)
    {
        //  Serial.print("Accel avg data is ");
        //  Serial.println(mySensor.reading(analogRead(ACCEL_PIN)));
        val = mySensor.reading(analogRead(ACCEL_PIN));
        xQueueSend(accel_queue, (void *)&val, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

int calibration_routine(Adafruit_SH1106 *display, uint32_t *max_accel, uint32_t *min_accel, uint32_t *max_break, uint32_t *min_break)
{
    static stateCalibration state = BEGIN;
    uint32_t accel_val;
    uint32_t button;
    uint32_t break_val;

    display->clearDisplay();
    display->setCursor(0, 0);
    display->setTextColor(WHITE);
    display->setTextSize(1);
    display->println("Mode: CALIBRATION");

    switch (state)
    {
    case BEGIN:
        display->setCursor(0, 31);
        display->print("Long press to begin");
        if (button_queue != 0)
        {

            if (xQueueReceive(button_queue, (void *)&button, 0) == pdPASS)
            {
                if (button == 20)
                {
                    state = ACCEL_MIN;
                }
            }
        }
        break;
    case ACCEL_MIN:
        display->println();
        display->println("Press button when min accel");
        display->println();

        display->drawRect(0, 40, 100, 10, WHITE);
        if (accel_queue != 0)
        {
            if (xQueueReceive(accel_queue, (void *)&accel_val, 0) == pdPASS)
            {
                display->fillRect(0, 40, map(accel_val, *min_accel, *max_accel, 0, 100), 10, WHITE);
            }
        }

        if (button_queue != 0)
        {
            if (xQueueReceive(button_queue, (void *)&button, 0) == pdPASS)
            {
                *min_accel = accel_val;
                state = ACCEL_MAX;
            }
        }
        break;
    case ACCEL_MAX:
        display->println();
        display->println("Press button when max accel");
        display->println();

        display->drawRect(0, 40, 100, 10, WHITE);
        if (accel_queue != 0)
        {
            if (xQueueReceive(accel_queue, (void *)&accel_val, 0) == pdPASS)
            {
                display->fillRect(0, 40, map(accel_val, *min_accel, *max_accel, 0, 100), 10, WHITE);
            }
        }

        if (button_queue != 0)
        {
            if (xQueueReceive(button_queue, (void *)&button, 0) == pdPASS)
            {
                *max_accel = accel_val;
                state = BREAK_MIN;
            }
        }
        break;

    case BREAK_MIN:
        display->println();
        display->println("Press button when min break");
        display->println();

        display->drawRect(0, 40, 100, 10, WHITE);
        if (break_queue != 0)
        {
            if (xQueueReceive(break_queue, (void *)&break_val, 0) == pdPASS)
            {
                display->fillRect(0, 40, map(break_val, *min_break, *max_break, 0, 100), 10, WHITE);
            }
        }

        if (button_queue != 0)
        {
            if (xQueueReceive(button_queue, (void *)&button, 0) == pdPASS)
            {
                *min_break = break_val;
                state = BREAK_MAX;
            }
        }
        break;

    case BREAK_MAX:
        display->println();
        display->println("Press button when max break");
        display->println();

        display->drawRect(0, 40, 100, 10, WHITE);
        if (break_queue != 0)
        {
            if (xQueueReceive(break_queue, (void *)&break_val, 0) == pdPASS)
            {
                display->fillRect(0, 40, map(break_val, *min_break, *max_break, 0, 100), 10, WHITE);
            }
        }

        if (button_queue != 0)
        {
            if (xQueueReceive(button_queue, (void *)&button, 0) == pdPASS)
            {
                *max_break = break_val;
                state = STORAGE;
            }
        }
        break;

    case STORAGE:
        EEPROM.writeUInt(0, *max_accel);
        EEPROM.writeUInt(4, *min_accel);
        EEPROM.writeUInt(8, *max_break);
        EEPROM.writeUInt(12, *min_break);
        EEPROM.commit();
        display->setCursor(0, 35);
        display->println("Calibration stored !");
        state = END;
        break;

    case END:
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        state = BEGIN;
        return 1;
        break;
    default:
        break;
    }

    return 0;
}
