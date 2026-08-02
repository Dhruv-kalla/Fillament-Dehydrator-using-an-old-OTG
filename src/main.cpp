
// al the libs we need
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduPID.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// defined quantities in the order of: PINS -> Physical constants to be kept the same all over code
#define SDA 19
#define SCL 22
#define I_PIN 33
#define P_PIN 25
#define TEMP_PIN_ONEWIRE 13
#define SSR_PIN 14

#define TIME 1000   // using a 1s pwm
#define SENS_RES 10 // you can increase it but i dont think this will make a diff since thermal systems are usually slow
#define CAPTHR 50   // threshold for interrupt triggering, closer your hand the less measured value gets so here when it gets below 50 isr runs, make sure to tune it acc to your setup
#define LCD_COL 16  // may change if u are using a 20x4 lcd but it would req making changes in lcd packet and the lcd func
#define LCD_ROW 2
// isr var
volatile uint64_t startTime = 0;
volatile bool start = false;
volatile uint32_t pTick = 0;
volatile uint32_t cTick = 0;
// pid const change depending on your system, you will have to tune it, make sure ki is pretty small
float kp = 8;
float ki = 0.07;
float kd = 5;

// Structs used as data packets to send data through pipelines i guess?
struct packet // data for control and display will be sent in packets and this struct is well the blueprint
{
  float cT;        // current temp
  float sT;        // set temp
  float dT;        // change in temp, d is used due to delta, not used rn but will use it in v2 probably
  float U;         // the time for which the ssr must be on, u is used because this value is sometimes known as the ERROR which is represented by U
  bool motorState; // not used rn but for motor control, coming soon
};
struct lcdData // this struct is for outputting data on lcd and you can say it acts as a mailbox packet(made that name up lol)?
{

  String firstLine;
  String secondLine;
};

// innit diff things PID ect
ArduPID PID1;
// our temp sensor
OneWire TempSensors(TEMP_PIN_ONEWIRE);
DallasTemperature sensors(&TempSensors);
// lcd
LiquidCrystal_I2C lcd(0x27, LCD_COL, LCD_ROW);
// tasks def, all func is understandable by the name and description in xTaskCreatePinnedToCore() func
TaskHandle_t UserInputH;
TaskHandle_t TempH;
TaskHandle_t lcdTaskH;
TaskHandle_t PidTask;
TaskHandle_t OutTask;
// Queues, works as a mailbox but instead of peeking data is Received so sync still happens!
QueueHandle_t Data1H;
QueueHandle_t Data2H;
QueueHandle_t Data3H;
QueueHandle_t LcdQueue;

// func def
void ARDUINO_ISR_ATTR ISR();
void uInputTask(void *pvParameters);
void tempSense(void *pvParameters);
void pidTask(void *pvParameters);
void lcdWrite(void *pvParameters);
void outputTask(void *pvParameters);

void setup()
{
  Serial.begin(9600); // serial used for debugging, if there is problem like lcd output getting cut u may check serial output
  // all lcd stuff
  Wire.begin(SDA, SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  // temp sensor init
  sensors.setResolution(SENS_RES); // as i said you may change resolution to make it more acc or you could dec it to make the pid loop faster
  sensors.begin();
  sensors.setWaitForConversion(false);

  PID1.setTunings(kp, ki, kd);   // this sets the pid gains
  PID1.setOutputLimits(0, TIME); // 1s duty

  pinMode(P_PIN, INPUT);
  pinMode(SSR_PIN, OUTPUT);
  analogReadResolution(SENS_RES); // i am using the same res for both pot and temp since it is good enough but you may change it

  touchAttachInterrupt(I_PIN, ISR, CAPTHR); // this for the esp32 touch interrupts, only a few pins support it so check which pin you want to use before changing

  // here we are creating all the mailbox queues ( mailbox is a queue with only space for one item in freeRTOS)
  Data1H = xQueueCreate(1, sizeof(packet));
  Data2H = xQueueCreate(1, sizeof(packet));
  Data3H = xQueueCreate(1, sizeof(packet));
  LcdQueue = xQueueCreate(1, sizeof(lcdData));
  // here we create the tasks, some are in core 0 placed intentionally due to them taking more time like lcd
  xTaskCreatePinnedToCore(uInputTask, "this is user lcd screen", 1500, NULL, 2, &UserInputH, 1);
  xTaskCreatePinnedToCore(tempSense, "this task will measure temp (and humidity in future)", 1500, NULL, 2, &TempH, 1);
  xTaskCreatePinnedToCore(lcdWrite, "writes to lcd", 6000, NULL, 2, &lcdTaskH, 0);
  xTaskCreatePinnedToCore(pidTask, "PID U CALC", 1500, NULL, 2, &PidTask, 1);
  xTaskCreatePinnedToCore(outputTask, "this controls all output", 1800, NULL, 2, &OutTask, 0);

  // just some error checking
  if (!Data1H || !Data2H || !LcdQueue || !UserInputH || !TempH || !lcdTaskH || !OutTask)
  {
    lcd.print("Error");
    Serial.print("Error");
    // set Out = 0 explicitly
    digitalWrite(SSR_PIN, HIGH);
  }
}
void loop()
{
}

void ARDUINO_ISR_ATTR ISR()
{
  BaseType_t xHigherPriorityTaskWoken = false; // FREERTOS var used in ISR
  cTick = xTaskGetTickCountFromISR();

  if (pdTICKS_TO_MS(cTick - pTick) >= 300) // debounce time may be changed
  {
    start = !start;
    vTaskNotifyGiveFromISR(UserInputH, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    pTick = cTick;
    startTime = esp_timer_get_time(); // this is used to get how many hours has elapsed
  }
}
void uInputTask(void *pvParameters)
{
  packet D1 = {
      .cT = 0,
      .sT = 0,
      .dT = 0,
      .U = 0,
      .motorState = 0};
  float sTemp = 0;
  lcdData Utext;
  int countNotif = 0;
  for (;;)
  {

    if (start == false)
    {
      D1.sT = 0;
      sTemp = map(analogRead(P_PIN), 0, 1023, 0, 100); // mapping between 0 and 100 which are min and max temp which may be changed but the max temp for our sensor is 125 degree c
      Utext.firstLine = "Sel sT and tap";
      Utext.secondLine = ("to start: " + String(sTemp, 1));
      xQueueOverwrite(LcdQueue, &Utext); // this is how lcd prints
    }
    else if (start == true)
    {
      if (Data1H != NULL)
      {
        D1.sT = sTemp; // setting the set point whenever the start is true. this happens due to isr btw
      }
    }
    xQueueOverwrite(Data1H, &D1);
    PID1.setSetpoint(D1.sT);                                  // at first as a safety feature it sets 0 as set temp
    countNotif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30)); // uses task notifs to sync with isr
  }
};
void tempSense(void *pvParameters)
{
  float cTemp = 0;
  lcdData text1;
  packet D2;
  bool rec1 = pdFALSE;
  for (;;)
  {
    rec1 = xQueueReceive(Data1H, &D2, portMAX_DELAY);
    if (rec1 != pdFALSE)
    {
      sensors.requestTemperaturesByIndex(0);
      vTaskDelay(pdMS_TO_TICKS(200)); // delay because it takes time to convert the sensor reading to actual temp(about 187ms)
      cTemp = sensors.getTempCByIndex(0);
      D2.cT = cTemp;
      D2.dT = D2.sT - cTemp;

      xQueueOverwrite(Data2H, &D2); // pipeline continues
    }
  }
};
void lcdWrite(void *pvParameters)
{
  lcdData text;
  text.firstLine = "";
  text.secondLine = "";
  lcdData textR;
  textR = text;
  lcd.setCursor(0, 0);
  lcd.println("                ");
  lcd.println("                ");

  for (;;)
  {

    if (LcdQueue != NULL)
    {
      BaseType_t lcdStat = xQueueReceive(LcdQueue, &text, portMAX_DELAY);
      if (lcdStat == pdPASS)
      {
        textR = text;
        if (text.firstLine.length() > 16 || text.secondLine.length() > 16)
        {
          Serial.print("You have a line with more than 16 char so it will be cut");
        }
      };
      if (textR.firstLine != "") // we are doing this because we are not using lcd.clear() sadly
      {
        lcd.setCursor(0, 0);
        while (textR.firstLine.length() <= 16)
        {
          textR.firstLine = textR.firstLine + " ";
        }
        lcd.print(textR.firstLine);
      }
      if (textR.secondLine != "")
      {
        lcd.setCursor(0, 1);
        while (textR.secondLine.length() <= 16)
        {
          textR.secondLine = textR.secondLine + " ";
        }
        lcd.print(textR.secondLine);
      }
    }
  }
}
void pidTask(void *pvParameters)
{
  packet D3;
  BaseType_t QueueStatPID;
  float U = 0;
  for (;;)
  {
    QueueStatPID = xQueueReceive(Data2H, &D3, portMAX_DELAY);
    if (QueueStatPID != pdFAIL)
    {
      D3.U = PID1.compute(D3.cT);
      xQueueOverwrite(Data3H, &D3); // pipeline continues, this time the error value is added
    }
  }
}
void outputTask(void *pvParameters) // this task is responsible for all output handling, in the future it will handle the fan too
{
  BaseType_t QueueStatFromPID;

  lcdData textOut = {.firstLine = " ", .secondLine = " "};
  packet D4;
  for (;;)
  {

    QueueStatFromPID = xQueueReceive(Data3H, &D4, portMAX_DELAY);
    textOut.firstLine = "sT:" + String(D4.sT, 1) + " cT:" + String(D4.cT, 1);
    textOut.secondLine = "U:" + String(D4.U) + " H:" + String((esp_timer_get_time() - startTime) / 3600000000.0, 2); // H is the hours elapsed
    if (start and QueueStatFromPID != pdFAIL)
    {
      xQueueOverwrite(LcdQueue, &textOut); // only if data is received then it shall send lcd the data to be print
      digitalWrite(SSR_PIN, LOW);          // my ssr turns on when out = low
      vTaskDelay(pdMS_TO_TICKS(D4.U));
      digitalWrite(SSR_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(TIME - D4.U)); // to make sure the  duty cycle is roughly 1s
    }
    else
    {
      digitalWrite(SSR_PIN, HIGH);
    }
    // add a FAN control in future along with a ADDRESSABLE LED based loading bar
  }
};