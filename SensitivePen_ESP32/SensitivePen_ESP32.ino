#include <elapsedMillis.h>
#include <WiFi.h>
#include <time.h>

#include "_MOVUINO_ESP32/_MPU9250.h"
// #include "_MOVUINO_ESP32/_WifiOSC.h"
#include "_MOVUINO_ESP32/_Button.h"
#include "_MOVUINO_ESP32/_Recorder.h"
#include "_MOVUINO_ESP32/_Neopixel.h"
#include "_MOVUINO_SHIELDS/_PressureSensor.h"

// Color swap
#define WHITE255 ((255 << 16) | (255 << 8) | 255)
#define RED ((255 << 16) | (0 << 8) | 0)
#define GREEN ((0 << 16) | (250 << 8) | 0)
#define YELLOW ((200 << 16) | (175 << 8) | 0)
#define BLUE ((0 << 16) | (0 << 8) | 255)
#define MAGENTA ((255 << 16) | (0 << 8) | 255)

// Command for serial messages
#define CMD_FORMAT_SPIFF 'f' // Format the SPIFF
#define CMD_CREATE_FILE 'c'  // Create a new file in the SPIFF
#define CMD_READ_FILE 'r'    // Read the file
#define CMD_ADD_LINE 'a'     // Add a new line in the SPIFFS (useful for debugging)
#define CMD_START_RECORD 'b'  // Start recording
#define CMD_STOP_RECORD 's'  // Stop recording
#define CMD_LISTING_DIR 'l'  // List files in the directory
#define CMD_SPIFF_INFO 'i'   // Get informations about the spiff

#define BATTERY_PIN 36       // Used to read the battery level
#define BATTERY_MIN_VAL 1900 // ~3.3v
#define BATTERY_MAX_VAL 2500 // ~4.2v

// ****************************** //
//       Wifi configuration       //
// ****************************** //
//char ssid[] = "COCOBONGO";
//char password[] = "welcome!";
const char* ssid     = "<SSID>";
const char* password = "<PASSWORD>";
int port = 555;
int ip[4] = {192, 168, 1, 18};


// ****************************** //
//       NTP configuration        //
// ****************************** //
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600; // gmt+1
const int   daylightOffset_sec = 3600;


// ****************************** //
//     MovuinoInitialisation      //
// ****************************** //
MovuinoMPU9250 mpu = MovuinoMPU9250();
// MovuinoWifiOSC osc = MovuinoWifiOSC(ssid, password, ip, port);
MovuinoButton button = MovuinoButton();
MovuinoRecorder recorder = MovuinoRecorder();
MovuinoNeopixel neopix = MovuinoNeopixel();
MovuinoPressureSensor pressure = MovuinoPressureSensor();

bool isBtnHold = false;
elapsedMillis dlyRec;

String recordId = "SensitivePen";
// basic cols
//String colsId = "ax,ay,az,gx,gy,gz,mx,my,mz,pressure";
// custom cols (timestamp added)
String colsId = "ts,ax,ay,az,gx,gy,gz,mx,my,mz";

int timeHoldCallib = 1800;

uint32_t colOn = BLUE;
uint32_t colRec = RED;
uint32_t colCallib = MAGENTA;
uint32_t colReadFiles = YELLOW;
uint32_t colFormat = RED;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  pinMode(BATTERY_PIN, INPUT);

  // Connect to Wi-Fi
  int status = WL_IDLE_STATUS;
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while(status != WL_CONNECTED){
      delay(500);
      status = WiFi.status();
      //Serial.println(get_wifi_status(status));
      Serial.print(".");
  }
  Serial.println(get_wifi_status(status));
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  Serial.print("Timestamp now in seconds is: ");
  Serial.println(get_time()); 
  Serial.print("Timestamp now is: ");
  Serial.println(get_timestamp());


  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Neopixel
  neopix.begin();
  neopix.setBrightness(5);
  showBatteryLevel();
  normalMode();
  freezBlink(2);
  neopix.update();
  
  // Other
  mpu.begin();
  // osc.begin();
  button.begin();
  recorder.begin();
  pressure.begin();
  freezBlink(4);
}

void loop()
{
  

  // -----------------------------------------
  //                UPDATES
  // -----------------------------------------
  neopix.update();
  button.update();

  // -----------------------------------------
  //                SERIAL
  // -----------------------------------------
  if (Serial.available() > 0)
  {
    char serialMessage = Serial.read();
    Serial.print("\n");
    Serial.print("Message received : ");
    Serial.println(serialMessage);

    //--------- Serial command -------------
    switch (serialMessage)
    {
    case CMD_CREATE_FILE:
      Serial.println("Creation of ");
      recorder.newRecord(recordId);
      break;
    case CMD_READ_FILE:
      Serial.println("reading all recorded files ");
      readAllfiles();
      break;
    case CMD_FORMAT_SPIFF:
      Serial.println("Formating the SPIFFS (data files)...");
      freezColorStrob(5, colFormat);
      neopix.setColor(colFormat);
      neopix.forceUpdate();
      recorder.formatSPIFFS();
      normalMode();
      break;
    case CMD_LISTING_DIR:
      Serial.println("Listing directory");
      neopix.blinkOn(50, 2);
      recorder.listDirectory();
      break;
    case CMD_SPIFF_INFO:
      Serial.println("Print info SPIFFS");
      neopix.blinkOn(50, 2);
      recorder.printStateSPIFFS();
      break;
    case CMD_ADD_LINE:
      neopix.blinkOn(50, 2);
      recorder.addRow();
      break;
    case CMD_START_RECORD:
      if (!recorder.isRecording())
      {
        startRecord();
      }
      break;    
    case CMD_STOP_RECORD:
      if (recorder.isRecording())
      {
        stopRecord();
      }
      break;
    default:
      break;
    }
  }

  // -----------------------------------------
  //                RECORDER
  // -----------------------------------------
  if (button.isReleased())
  {
    if (!isBtnHold)
    {
      if (!recorder.isRecording())
      {
        startRecord();
      }
      else
      {
        stopRecord();
      }
    }
    isBtnHold = false;
  }

  if (recorder.isRecording())
  {
    if (dlyRec > 10)
    {
      dlyRec = 0;
      mpu.update();
      pressure.update();  

      recorder.addRow();
      recorder.pushData<String>(ToString(get_timestamp()));
      recorder.pushData<float>(mpu.ax);
      recorder.pushData<float>(mpu.ay);
      recorder.pushData<float>(mpu.az);
      recorder.pushData<float>(mpu.gx);
      recorder.pushData<float>(mpu.gy);
      recorder.pushData<float>(mpu.gz);
      recorder.pushData<float>(mpu.mx);
      recorder.pushData<float>(mpu.my);
      recorder.pushData<float>(mpu.mz);
      // recorder.pushData<float>(pressure.getPressure());
    }
  }

  // -----------------------------------------
  //               CALLIBRATION
  // -----------------------------------------
  if (button.timeHold())
  {
    // Color shade
    float r_ = (button.timeHold() - 400) / (float)timeHoldCallib;
    neopix.lerpTo(colCallib, r_);

    if (button.timeHold() > timeHoldCallib)
    {
      neopix.setColor(colCallib); // lock color
      if (button.timeHold() > timeHoldCallib + 20)
      {
        isBtnHold = true;
        freezBlink(2);
        if (!recorder.isRecording())
        {
          mpu.magnometerCalibration();
          button.reset(); // force reset
          neopix.blinkOn(100, 2);
          normalMode();
        }
      }
    }
  }
}

void normalMode() {
  neopix.setColor(colOn);
}

void startRecord()
{
  recorder.newRecord(recordId);
  recorder.defineColumns(colsId);
  
  freezColorStrob(2, RED);
  neopix.rainbowOn();
  neopix.breathOn(1000, 0.8);
}

void stopRecord()
{
  freezColorStrob(2, GREEN);
  neopix.rainbowOff();
  neopix.breathOff();
  recorder.stop();
  normalMode();
}

void readAllfiles()
{
  freezColorStrob(2, colReadFiles);
  neopix.setColor(colReadFiles);
  neopix.forceUpdate();

  recorder.readAllRecords();
  
  freezBlink(3);
  normalMode();
}

void freezBlink(int nblink_)
{
  for (int i = 0; i < nblink_; i++)
  {
    neopix.turnOff();
    neopix.forceUpdate();
    delay(50);
    neopix.turnOn();
    neopix.forceUpdate();
    delay(50);
  }
}

void freezColorStrob(int nblink_, uint32_t color_)
{
  uint32_t curCol_ = neopix.getColor();
  for (int i = 0; i < nblink_; i++)
  {
    neopix.setColor(color_);
    neopix.forceUpdate();
    delay(100);
    neopix.setColor(curCol_);
    neopix.forceUpdate();
    delay(100);
  }
}

void showBatteryLevel(void)
{
  int sum;
  int level;

  sum = 0;
  for (uint8_t i = 0; i < 10; i++) // Do the average over 10 values
    sum += analogRead(BATTERY_PIN);
  
  if (sum < BATTERY_MIN_VAL * 10)
    sum = BATTERY_MIN_VAL * 10;

  if (sum > BATTERY_MAX_VAL * 10)
    sum = BATTERY_MAX_VAL * 10;
   
  level = ((float)((sum / 10) - BATTERY_MIN_VAL ) / (float)(BATTERY_MAX_VAL - BATTERY_MIN_VAL)) * 100.0;
  
  if (level >= 50)
    neopix.setColor((uint32_t)GREEN);
  else if (level >= 25)
    neopix.setColor((uint32_t)YELLOW);
  else
    neopix.setColor((uint32_t)RED);
    
  neopix.forceUpdate();
  delay(2500);
  Serial.printf("Battery Reading: %d\n", sum / 10);
  Serial.printf("Battery Level: %d%%\n", level);
  delay(2500);
}


String get_wifi_status(int status){
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "Current date is %A, %B %d %Y %H:%M:%S");
}

// Function that gets current epoch time
unsigned long get_time() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}


int64_t get_timestamp() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  return time_us;
}

String ToString(uint64_t x)
{
     boolean flag = false; // For preventing string return like this 0000123, with a lot of zeros in front.
     String str = "";      // Start with an empty string.
     uint64_t y = 10000000000000000000;
     int res;
     if (x == 0)  // if x = 0 and this is not testet, then function return a empty string.
     {
           str = "0";
           return str;  // or return "0";
     }    
     while (y > 0)
     {                
            res = (int)(x / y);
            if (res > 0)  // Wait for res > 0, then start adding to string.
                flag = true;
            if (flag == true)
                str = str + String(res);
            x = x - (y * (uint64_t)res);  // Subtract res times * y from x
            y = y / 10;                   // Reducer y with 10    
     }
     return str;
}  
