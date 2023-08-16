#define TOUCH_MODULES_CST_MUTUAL
#include "Arduino.h"
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "OneButton.h"
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "TouchLib.h"
#include "img_logo.h" //For heart img
#include "pin_config.h"
#include <TaskScheduler.h>
#include <MAX30105.h> //Read HR IR and RED
#include "heartRate.h"
#include "SD_MMC.h"
#include "SD.h"
#include "ble_functions.h"
#include "FirebaseC.h"
#include "BG.h"                  //bg image
#include "filters.h"             //filter for spo2
#include <Pangodream_18650_CL.h> //Battery calculator
#define I2C_SDA 44               // for Max30102 sensor
#define I2C_SCL 16
#define TESTPIN 4
bool SerialDebug = true; //for connection check
//-------------------SPI for SD--------------------
#define HSPI_SDO 11 // d pin
#define HSPI_SDI 13 // Qpin
#define HSPI_SCLK 1
#define HSPI_CS 10
//---------------------Battery---------------------
#define MIN_USB_VOL 4.9
#define CONV_FACTOR 1.8
#define READS 20
Pangodream_18650_CL BL(PIN_BAT_VOLT, CONV_FACTOR, READS);

//----------For Serial---------------
String msg;
String message = "";
bool messageReady = false;
int UARTSW = 1;
int Req_ID;
bool Doc_Created = false;
bool DocRecieve_Created = false;
//-----------------------------------
DynamicJsonDocument doc_Recieved(1024);
DynamicJsonDocument Request(1024);
//------------------------------------------------------------------------------------------------------
MAX30105 particleSensor;
// Sensor (adjust to your sensor type)
const float kSamplingFrequency = 400.0;
// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;
// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;
//-------------------------------------------------------------
TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS328_SLAVE_ADDRESS, PIN_TOUCH_RES);
#define TOUCH_GET_FORM_INT 0

TFT_eSPI tft = TFT_eSPI();
bool flags_sleep = false;
#if TOUCH_GET_FORM_INT
bool get_int = false;
#endif
//--------------BUTTONS------------------------------------------
#define APin 2
#define BPin 3
#define SensorDetecPin 12
#define Vibepin 43
OneButton button(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);
//---------------------------------------
TFT_eSprite sprite = TFT_eSprite(&tft);
TFT_eSprite spr2 = TFT_eSprite(&tft);
TFT_eSprite HRgraph = TFT_eSprite(&tft);      // Line graph
TFT_eSprite ECGgraph = TFT_eSprite(&tft);     // Line graph
TFT_eSprite PPGgraph = TFT_eSprite(&tft);     // table
TFT_eSprite PauseButtons = TFT_eSprite(&tft); // Heart Pict
TFT_eSprite Text = TFT_eSprite(&tft);         // Heart Pict

#define color2 0x8410
//-----------------GRAPH VAR------
#define gw 200      // graph width
#define gh 70       // graph heigth
#define gx 0        // graph x
#define gy 70       // graph Y
#define lx 0        // HR line x
#define ly 70       // HR line Y
#define gw2 200     //  PPG graph width
#define gh2 70      //  PPG graph heigth
#define gx2 0       // PPG graph x
#define gy2 70      // PPG graph Y
#define lx2 0       // PPG line x
#define ly2 70 + 15 // PPG line Y
//--------------------------------
int ECG_Y;
// -----------------Back ground---------------------------
#define iW 640
#define iH 360
#define w 320
#define h 170
int m = w;
#define xt 8
#define yt 8
int pos = 0;
int x = 0;
int y = 30;
int changeX = 1;
int changeY = 1;
//----------------SPO2 Parameters-------------
#define MAX_BRIGHTNESS 255
int8_t Cur_SpO2;
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
int32_t bufferLength;    // data length
int32_t spo2;            // SPO2 value
bool validSPO2;          // indicator to show if the SPO2 calculation is valid
int32_t heartRate;       // heart rate value
byte i = 0;
bool Sdvalid = true;
bool SCSaverOn;
//-----------------------------------SERVER PARAM--------------------------------------------------
const char *ssid = "IC-3 TCP";
const char *password = "88888888";
const char *PARAM_INPUT_1 = "SSID";
const char *PARAM_INPUT_2 = "PW";
AsyncWebServer server(80);
bool FinalisedWIFI, getSSID, getPW;
String NewSSID, NewPW;
bool WFfirstrun;
bool WifiSL;
String SLWifi, SLPW, SLST;
int WifiPage; // 0 home, 1 Selected
int Wifirow, InternalWifiRw;
float Inttime;
// String CurWifi, CurPW; //use var from FireBaseC.Cpp
//--------------------------Settings VAR---------------------------------- update these on start using file in SD
int SCBright = 255; // varible for Screen brightness
bool Vibrate;       // check if the buzzer works or just use the vibrator
int ScreenOff = 60; // 15,30,60,120,300,600,infinity
int SettingsPage = 0;
int MoveX; // for brightness settings
//------------------------------------------------------------------------
int start = 1;
bool Wifisettings;
bool WebINI;
bool WifiSettingFR; // Wifi settings First run
unsigned short imageS[54400] = {0};
int ScreenNum = 1;
int XCoord, YCoord, PrCoord;
bool StartRec = false;
bool SCRHoldL = false;
bool SCRHoldR = false;
bool MeasureHold = false;
bool ProfileHold = false;
bool StartScr = false;
bool boolMeasureHR = false;
bool boolMeasureECG = false;
bool FirstRead = false;
bool Beat;
bool Profileset = false;
bool Pause = false;
bool PauseHold = false;
bool ResumeHOLD, StopHOLD, PFHOLD, SaveHOLD, SaveDATA, DelSave; // Pause screen buttons bool
bool RecDel, RecDelConfirm, RecServer;
bool CancelHOLD, ConFirmHOLD;
bool BLEOn;
bool CancelDeleteHOLD;
bool STOP, CFSAVE; // stop and confirm stop bools for stop button
int latestMIll;
int XCoordLS, XCoordRS;
int YCoordLS;
int ProfileRw;
int ModeRw;
int InternalRw = 0;
int InternalRRw = 0;
bool MaxConnect = false;
bool WifiModconnect = false;
bool VibeOn;
#define why 150
int PausePG = 1;  // 1 and 2
bool TextCreated; // Checking if the latest Text file has been made
uint64_t cardSize;
uint64_t CurDisplayMS;
uint64_t bytesAvailable;
uint64_t bytesUsed;
Scheduler userScheduler; // to control your personal task

void BGRefresh();
void TouchCode();
void BGimage();
void InfoSet();
void ButtonTouch();
void MeasureTouch();
void MeasureHR();
void ScreenMulti();
void ScreenHR();
void ScreenECG();
void PPGGraphV();
void HRgraphV();
void Topbar();
void PauseFunction();
void DataLogger();
void BasicFolders();
void FolderCreate();
void ScreenProfile();
void ProfileTouch();
void SettingScreen();
void SettingAPI();
void SettingTouch();
void Profilelist(File dir2);
void Wifilist(File dir2);
void ScanWifiST(File dir2);
void ProfileCreate();
void draw_ECG(int16_t *ecg_buf, int ecg_buf_pos, int ecg_buf_len, int BPM);
void Backbutton();
void DataEdit();
void LoadingBar();
void ScreenSaver();
void ScreenRecords();
void RecordTouch();
unsigned long getTime();
void VibrateCD();
void ButtonCK();
void ConCheck();
bool SizeChk(String Path);
void WebHostinit();
void update_ecg_buffer(int CURMilli);
void WifiScr();
void WifiTouch();
Task taskBGRefresh(TASK_SECOND * 0.05, TASK_FOREVER, &BGRefresh);
Task taskTouchCode(TASK_SECOND * 0.01, TASK_FOREVER, &TouchCode);
Task taskMeasureHR(TASK_SECOND * 0.05, TASK_FOREVER, &MeasureHR);
Task taskDataLogger(TASK_SECOND * 0.05, TASK_FOREVER, &DataLogger);
Task taskVibrate(TASK_SECOND * 0.5, TASK_FOREVER, &VibrateCD);
Task taskButtonCK(TASK_SECOND * 0.5, TASK_FOREVER, &ButtonCK);
Task taskConCheck(TASK_SECOND * 1, TASK_FOREVER, &ConCheck);
int HRvalues[30] = {0};
int HRvalues2[30] = {0};
int PPGvalues[30] = {0};
int PPGvalues2[30] = {0};
int curent = 0;
#define calib 20
int mbpm;
int BARLENTH;
int Multipage = 1; // Page for multi measuring
//-------------------------------------------------------------------------------
#define SCALING 9                          // Scale height of trace, reduce value to make trace height
#define TRACE_MIDDLE_Y_POSITION 41         // y pos on screen of approx middle of trace
#define TRACE_HEIGHT 64                    // Max height of trace in pixels
#define HALF_TRACE_HEIGHT TRACE_HEIGHT / 2 // half Max height of trace in pixels (the trace amplitude)

//---------------SD CARD VALLS------------------------------------
SPIClass spi = SPIClass(VSPI);
File SDdump;
File SD1;
File PFdump;
String OBJname, FinalString, FinalDATA;
String FileData;
int OBJLenth, Finalint, MaxNum, OBJSize;
uint64_t StartMillis, CURMillis, LatestTouch;
String User;                 // Username
unsigned int LastDataUpdate; // Last time data SD string was updated
int Interval;                // Interval for each mode
bool DidRec;
bool PageSWHold;
int TimeStart, CurrentSec, CurrentMin;
//------------------------For profile-----------------------------
bool PFfirstrun = false;
int Profileamount;
bool FileFound;
String Row2N, Row2S, Row2A, Row2Y, Row2T;
String Row3N, Row3S, Row3A, Row3Y, Row3T;
String Row4N, Row4S, Row4A, Row4Y, Row4T;
String Row5N, Row5S, Row5A, Row5Y, Row5T;
String Row6N, Row6S, Row6A, Row6Y, Row6T;

String Row2W, Row2P, Row2ST;
String Row3W, Row3P, Row3ST;
String Row4W, Row4P, Row4ST;
String Row5W, Row5P, Row5ST;
String Row6W, Row6P, Row6ST;
bool BOneP, BTwoP, BThreeP, BFourP; // Buttons
bool ButtonTouched;
unsigned int lastupdate;
bool ONEran = false;
bool TWOran = false;
bool THREEran = false;
bool FOURran = false;
bool FIVEran = false;
String CurName, CurSex, CurAge;
bool ProfileUPHold, ProfileDWHold, ProfileSLHold, ProfileSL;
int ProfileCRmode = 0; // 0 normal 1 set name 2 set AGE 3 set gender 4 submit
bool ValidProfileSL = false;
bool Noprofilehold;
String DMY, NewDMY;
int YearMonthDate = 1; // 1 is date 2 is month 3 is year
//------------------------------------RECORDS SYS--------------------------------
int RecordsPage;
String RecName, RecSex, RecAge, RecYear, RecTime;
int RecRow;
bool StopCF;
//-------------------------------------NAME INPUT-------------------------------
int NameLenth;
String OutName;
bool FAlphaHold, SAlphaHold, TAlphaHold, FRAlphaHold, BackSpcHold, NextHold, BackHold;
int PageAlpha = 1;
int Yearint;
int YearPg = 1;
String FDigit, SDigit, TDigit, FrDigit, NewNameStr, NewGenderStr;
int NewAge, NewDate, NewMonth;
String NewYear;
bool FinaliseInput;
bool confirmDEL, EditData, ConfirmEDIT;
//-------------------------AGE and GENDER INPUT--------------------------------------------
int Ones, Tens, EditOnes, EditTens;
String AgeOns, AgeTens, NewSex;
bool OneUpHold, TenUpHold, OneDwHold, TenDwHold, MaleHold, FemaleHold;
//------------------------------EDIT DATA-------------------------
int editPage, EditNameLenth, EditAge;
String EditName, EditGender;
bool EditBackHold;
//----------------------For ECG GRAPH------------------------------
int BoxXcoord;
int ECG_BPM;
int ECG_batt;
int16_t *ECG_ecg_buf;
int ECG_ecg_bp;
int ECG_ecg_blen;
int min_y, max_y;
int prev_x = 0;
int prev_y = 0;
float Uv;
//-------------------------FOR DATA LOGGER--------------------------
bool ECGfoundbf; // Start logging because the ecg has been found before
int LastCommit = 0;
bool ECGRec;
bool PauseRecord;
//--------------------------For SD data parse and server-----------------------
#define BUFSIZE 50
char package[BUFSIZE] = {};
int Posi;
bool WIFIOn;
bool BatchDone;
int MaxBatch;
String SSID, WIFIPW; // re-implement
String OUTName;
int FileAmount;
bool WifiEmpty;
//-----------------SDCARD COMMANDS----------------------------------
void printDirectory(File dir, String Path)
{
  if ((ButtonTouched) || (PFfirstrun))
  {
    ONEran = false; // RESET EVERYTHING
    TWOran = false;
    THREEran = false;
    FOURran = false;
    FIVEran = false;
    PFfirstrun = false;
    FileAmount = 0;
    ButtonTouched = false;
    File Entry;
    while (true)
    {
      Entry = dir.openNextFile();
      if (!Entry)
      {
        // no more files
        break;
      }
      OBJname = Entry.name();
      OBJLenth = OBJname.length();
      String FinalPath = Path + "/" + OBJname;
      File SizeCK = SD.open(FinalPath);
      OBJSize = SizeCK.size();
      FileAmount++;
      String FilePath;
      if (FileAmount == (1 + InternalRRw)) // 1+0 is first data 1+1 is second
      {
        Row2N = OBJname;
        Row2A = OBJLenth;
        Row2S = OBJSize;
        // Serial.println("Row2N = "+Row2N);
        FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + Row2N;
        SizeChk(FilePath);
        Row2T = Inttime / 60000;
        ONEran = true;
        TWOran = false;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (FileAmount == (2 + InternalRRw))
      {
        Row3N = OBJname;
        Row3A = OBJLenth;
        Row3S = OBJSize;
        FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + Row3N;
        SizeChk(FilePath);
        Row3T = Inttime / 60000;
        TWOran = true;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (FileAmount == (3 + InternalRRw))
      {
        Row4N = OBJname;
        Row4A = OBJLenth;
        Row4S = OBJSize;
        FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + Row4N;
        SizeChk(FilePath);
        Row4T = Inttime / 60000;
        THREEran = true;
        FOURran = false;
        FIVEran = false;
      }
      else if (FileAmount == (4 + InternalRRw))
      {
        Row5N = OBJname;
        Row5A = OBJLenth;
        Row5S = OBJSize;
        FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + Row5N;
        SizeChk(FilePath);
        Row5T = Inttime / 60000;
        FOURran = true;
        FIVEran = false;
      }
      else if (FileAmount == (5 + InternalRRw))
      {
        Row6N = OBJname;
        Row6A = OBJLenth;
        Row6S = OBJSize;
        FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + Row6N;
        SizeChk(FilePath);
        Row6T = Inttime / 60000;
        FIVEran = true;
      }
    }
    Entry.close();
  }
  if (Profileamount <= (4 + InternalRRw))
  {
    if (!ONEran) // remove empty results
    {
      Row2N = "";
      Row2S = "";
      Row2A = "";
      Row2T = "";
    }
    if (!TWOran)
    {
      Row3N = "";
      Row3S = "";
      Row3A = "";
      Row3T = "";
    }
    if (!THREEran)
    {
      Row4N = "";
      Row4S = "";
      Row4A = "";
      Row4T = "";
    }
    if (!FOURran)
    {
      Row5N = "";
      Row5S = "";
      Row5A = "";
      Row5T = "";
    }
    if (!FIVEran)
    {
      Row6N = "";
      Row6S = "";
      Row6A = "";
      Row6T = "";
    }
  }
}
void createDir(fs::FS &fs, const char *path) // create folder
{
  if (!fs.exists(path)) // if not found
  {
    if (fs.mkdir(path)) // create
    {
    }
    else
    {
      // Serial.println("mkdir failed");
    }
  }
}

void removeFile(fs::FS &fs, const char *path)
{
  // Serial.printf("Removing Dir: %s\n", path);
  if (fs.remove(path))
  {
    // Serial.println("File removed");
  }
}

void removeDir(fs::FS &fs, const char *path)
{
  // Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path))
  {
    // Serial.println("Dir removed");
  }
}

String readFile(fs::FS &fs, const char *path)
{
  // Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file)
  {
    // Serial.println("Failed to open file for reading");
    FileFound = false;
    return "";
  }
  // Serial.print("Read from file: ");
  String OUTPUT1 = "";
  while (file.available())
  {
    OUTPUT1 = file.readString();
  }
  file.close();
  FileFound = true;
  return OUTPUT1;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  // Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    // Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    // Serial.println("File written");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    return;
  }
  file.print(message);
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
  if (fs.rename(path1, path2))
  {
    Serial.println("File renamed");
  }
}

int FindMAXName(File dir, int numTabs)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    OBJname = entry.name();
    OBJLenth = OBJname.length();
    for (int digit = 0; digit < OBJLenth - 1; digit++)
    {
      if (isDigit(OBJname[digit]))
      {
        FinalString = FinalString + String(OBJname[digit]);
      }
      if (FinalString.toInt() > MaxNum)
      {
        MaxNum = FinalString.toInt();
      }
    }
    FinalString = ""; // reset Final String for new file name
    // Serial.println(entry.name()); // print out name of file
    entry.close();
  }
  // Serial.println("MAXNUM = " + String(MaxNum));
  return MaxNum;
}

void SendtoServer(String FilePath)
{
  WifiInit();
  if (!WifiNotFound)
  {
    File Fifo = SD.open(FilePath);
    String FileName = Fifo.name();
    WifiCycle(FilePath, FileName);
    WiFi.disconnect();
    WiFi.mode(WIFI_MODE_NULL); // Turn off wifi
  }
  else
  {
    sprite.fillRoundRect(160 - 155, 24 + 45, 310, 30, 4, TFT_LIGHTGREY);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("WIFI NOT FOUND", 160, 35 + 45);
    sprite.pushSprite(0, 0);
    WifiNotFound = false;
    delay(1000);
  }
}
//-----------------------------------------------------------------------------------
TaskHandle_t Loop2;
TaskHandle_t BLE;
uint32_t last_draw_ms = 0;
int n = 0;
void BLEcode(void *pvParameters)
{
  for (;;)
  {
    if (BLEOn)
    { // run only if BLE is on
      ble_cycle();
      // Serial.print("Free Heap : " + String(ESP.getFreeHeap()));
      // Serial.println("This task watermark: " + String(uxTaskGetStackHighWaterMark(NULL)) + " bytes");
    }
    vTaskDelay(1); // Don't remove this it will crash
  }
}
void Loop2code(void *pvParameters)
{
  for (;;)
  {
    userScheduler.execute();
    ScreenSaver();
    if (analogRead(APin) >= 3000)
    { // top bt
      taskVibrate.enable();
      BOneP = true;
      LatestTouch = millis();
    }
    if (analogRead(BPin) >= 3000)
    {
      taskVibrate.enable();
      BTwoP = true;
      LatestTouch = millis();
    } // bottom bt

    if (digitalRead(PIN_BUTTON_1) == 0)
    {
      taskVibrate.enable();
      BThreeP = true;
      LatestTouch = millis();
    } // left

    if (analogRead(PIN_BUTTON_2) <= 100)
    {
      taskVibrate.enable();
      BFourP = true;
      LatestTouch = millis();
    } // right

    // Serial.println("B1 = " + String(analogRead(APin)) + " B2 = " + String(analogRead(BPin)) + "B3 = " + String(digitalRead(PIN_BUTTON_1)) + " B4 = " + String(analogRead(PIN_BUTTON_2)));
    if (flags_sleep)
    {
      flags_sleep = false;
      tft.fillScreen(TFT_GREEN);
      tft.drawString("about to go to sleep", 0, 15);
      touch.enableSleep();
      delay(2000);
      digitalWrite(PIN_POWER_ON, LOW);
      gpio_hold_en((gpio_num_t)PIN_TOUCH_RES);
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_TOUCH_INT, 0);
      esp_deep_sleep_start();
    }
  }
}

void setup()
{
  gpio_hold_dis((gpio_num_t)PIN_TOUCH_RES);
  pinMode(PIN_POWER_ON, OUTPUT);
  pinMode(Vibepin, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  pinMode(PIN_TOUCH_RES, OUTPUT);
  digitalWrite(PIN_TOUCH_RES, LOW);
  delay(500);
  digitalWrite(PIN_TOUCH_RES, HIGH);
  Serial.begin(115200);
  delay(2500);
  tft.begin();
  tft.setRotation(3);
  tft.setSwapBytes(true);
  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, SCBright); // use this for writing sc brightness 0-255
  // SPIClass spi = SPIClass(HSPI);
  spi.begin(HSPI_SCLK /* SCK */, HSPI_SDO /* MISO */, HSPI_SDI /* MOSI */, HSPI_CS /* SS */);
  if (!SD.begin(HSPI_CS /* SS */, spi))
  {
    Sdvalid = false;
  }
  else
  {
    Sdvalid = true;
  }
  BasicFolders();
  SettingAPI();
  sprite.createSprite(320, 170);
  PauseButtons.createSprite(200, 30);
  spr2.createSprite(320, 170);
  HRgraph.createSprite(201, 71);
  ECGgraph.createSprite(201, 71);
  PPGgraph.createSprite(201, 71);
  Text.createSprite(320, 170);
  ECGgraph.setTextDatum(4);
  HRgraph.setTextDatum(4);
  PPGgraph.setTextDatum(4);
  sprite.setSwapBytes(true);
  PauseButtons.setSwapBytes(true);
  Text.setSwapBytes(true);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL); // touch funtion uses pin 18,17
  Wire1.begin(I2C_SDA, I2C_SCL, 400000);

  if (!touch.init())
  {
    Serial.println("Touch IC not found");
  }
  ProfileRw = 1;
  LatestTouch = millis();
  button.attachClick([]
                     { SettingAPI(); });
  button2.attachClick([]
                      { taskVibrate.enable(); });
  // spr2.fillSprite(TFT_GREEN);
  sprite.setTextDatum(4);
  spr2.setTextDatum(4);
  spr2.setFreeFont(&Orbitron_Light_24);
  sprite.setFreeFont(&Orbitron_Light_24);
  userScheduler.addTask(taskTouchCode);
  userScheduler.addTask(taskBGRefresh);
  userScheduler.addTask(taskMeasureHR);
  userScheduler.addTask(taskDataLogger);
  userScheduler.addTask(taskVibrate);
  userScheduler.addTask(taskButtonCK);
  userScheduler.addTask(taskConCheck);
  // taskConCheck.enable();
  taskTouchCode.enable();
  taskBGRefresh.enable();
  if (Sdvalid)
  {
    taskDataLogger.enable();
    // Serial.println("Found SD : Datalog on");
  }
  else
  {
    // Serial.println("NO SD : Datalog off");
  }

  taskButtonCK.enable();
  xTaskCreatePinnedToCore(
      BLEcode, /* Task function. */
      "BLE",   /* name of task. */
      10000,   /* Stack size of task */
      NULL,    /* parameter of the task */
      1,       /* priority of the task */
      &BLE,    /* Task handle to keep track of created task */
      0);      /* pin task to core 0 */
  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Loop2code, /* Task function. */
      "Loop2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Loop2,    /* Task handle to keep track of created task */
      1);        /* pin task to core 1 */
}

void BGimage()
{
  pos = x + iW * y;
  start = pos;
  m = w + pos;
  for (int i = 0; i < 54400; i++)
  {
    if (start % m == 0)
    {
      start = start + (iW - w);
      m = m + iW;
    }
    imageS[i] = Pict[start];
    start++;
  }
  x = x + changeX;
  if (x == iW - w - 1 || x < 0)
    changeX = changeX * -1;

  y = y + changeY;
  if (y == iH - h - 1 || y < 1)
    changeY = changeY * -1;
  sprite.pushImage(0, 0, 320, 170, imageS);
}
void BGRefresh()
{
  if (!SCSaverOn)
    BGimage();
  if (ScreenNum == 4) // Home page Start and profile become records and settings
  {
    if (!StartRec && !Profileset) // Removes home buttons when Measuring
    {
      PFfirstrun = true; // for running the Profile the first time;
      MaxNum = 0;        // reset Maxnum
      if (!SCSaverOn)
        InfoSet();
      ButtonTouch();
      spr2.pushToSprite(&sprite, xt, yt, TFT_BLACK); // Pushes stuff into sprite
    }
    else if (Profileset) // press the settings
    {
      if (!SCSaverOn)
      {
        SettingScreen();
        SettingTouch();
      }
    }
    else if (StartRec) // RECORD Pressed
    {
      ScreenRecords();
      RecordTouch();
    }
  }
  else
  {
    if (!StartRec && !Profileset) // Removes home buttons when Measuring
    {
      PauseRecord = false;
      if (ScreenNum == 2)
      {
        Interval = 500; // Hr interval
      }
      else if (ScreenNum == 1)
      {
        Interval = 100; // ECG interval
      }
      else if (ScreenNum == 3)
      {
        Interval = 100; // ECG interval
      }
      if (BLEOn)
      {
        // BLE_Reset();
        BoxXcoord = 0;
        ECGfoundbf = false;
        ECGRec = false;
        btStop(); // Turn off BLE

        // Serial.print("BLE OFF");
        // esp_bt_controller_disable();
        BLEOn = false;
      }
      PFfirstrun = true; // for running the Profile the first time;
      MaxNum = 0;        // reset Maxnum
      if (!SCSaverOn)
      {
        InfoSet();
        ButtonTouch();
      }
      spr2.pushToSprite(&sprite, xt, yt, TFT_BLACK); // Pushes stuff into sprite
      if (DidRec)
      {
        // Serial.println("Turned bool off");
        boolMeasureHR = false;
        boolMeasureECG = false;
        taskMeasureHR.disable();
        DidRec = false;
      }
    }
    else if (Profileset) // press the profile button
    {
      if (!SCSaverOn)
      {
        ScreenProfile();
        ProfileTouch();
      }
    }
    else
    {
      if ((!Sdvalid) && (CurName == "")) // if no SD card
      {
        Topbar();
        CurName = "NO-SD";
        delay(1500);
      }
      if (CurName == "")
      { // noprofile selected
        Topbar();
        sprite.fillRoundRect(32, 40, 255, 40, 4, TFT_LIGHTGREY);
        sprite.fillRoundRect(38, 85, 245, 40, 4, TFT_LIGHTGREY);
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("PLEASE SELECT", 160, 57);
        sprite.drawString("VALID PROFILE", 158, 102);
        if (!Noprofilehold) // continue button
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 200) && (YCoord >= 100))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_DARKCYAN);
            Noprofilehold = true;
          }
          else
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            StartRec = false;
            Profileset = true;
            Noprofilehold = false;
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_DARKCYAN);
          }
          else
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
            Noprofilehold = false;
          }
        }
        sprite.drawString("Continue", 160, 150, 2);
      }
      else
      {
        DidRec = true;
        if (ScreenNum == 2) // Measure SpO2 and HR
        {
          if (!boolMeasureHR)
          {
            CurrentSec = millis();
            taskMeasureHR.enable();
            FolderCreate();
            Pause = false;   // Starts without pauseing
            DelSave = false; // start every recording with BG saving
            StartMillis = millis();
          }
          boolMeasureHR = true;
          if (!SCSaverOn)
            ScreenHR();
        }
        else if (ScreenNum == 1) // Measure ECG
        {
          if (!SCSaverOn)
            ScreenECG(); // Write the screen first to avoid delays from ble code (Doesn't seems to help)
                         // Serial.print("RUNNING\n");
          if (ECGRec)
          {
            update_ecg_buffer(CURMillis);
          }
          if (!boolMeasureECG)
          {
            TimeStart = millis();
            BLEOn = true;
            ble_functions_init(); // Starts BLE
            FolderCreate();
            Pause = false;   // Starts without pauseing
            DelSave = false; // start every recording with BG saving
            StartMillis = millis();
          }
          boolMeasureECG = true;
        }
        else if (ScreenNum == 3) // Measure ALL
        {
          if (ECGRec)
          {
            update_ecg_buffer(CURMillis);
          }
          if (!SCSaverOn)
          {
            if (Multipage == 1)
            {
              ScreenMulti();
            }
            else if (Multipage == 2)
            {
              ScreenECG();
            }
            else if (Multipage == 3)
            {
              ScreenHR();
            }
          }
          if (!boolMeasureHR)
          {
            CurrentSec = millis();
            taskMeasureHR.enable();
            BLEOn = true;
            ble_functions_init(); // Starts BLE
            taskMeasureHR.enable();
            FolderCreate();
            boolMeasureHR = true;
            Pause = false;   // Starts without pauseing
            DelSave = false; // start every recording with BG saving
            StartMillis = millis();
          }
          boolMeasureECG = true;
        }
        if (!SCSaverOn)
          MeasureTouch();
      }
    }
  }
  sprite.pushSprite(0, 0); // This write all the sprites?
}
void ScreenMulti()
{
  ECGgraph.fillRect(gx, 0, gw, gh, TFT_BLUE);
  int ECG_BPM = get_BPM();
  int bat_lvl = get_uecg_batt();
  int16_t *ECG_ecg_buf = get_ecg_buf();
  int ECG_ecg_bp = get_ecg_buf_pos();
  int ECG_ecg_blen = get_ecg_buf_len();
  if (uECGConnected == 2) // Scanning
  {
  }
  else if (uECGConnected == 3) // Connecting change back to 3
  {
  }
  else if (uECGConnected == 1) // change back to 1
  {
    for (int i = 1; i < 10; i++)
      ECGgraph.drawLine(gx + (i * gw / 10), gy, gx + (i * gw / 10), gh - gy, color2); // Y axis line

    for (int i = 1; i < 9; i++)
    {
      ECGgraph.drawLine(gx, gy - (i * 9), gx + gw, gy - (i * 9), color2); // X axis line
    }
    Topbar();
    sprite.setTextColor(TFT_BLACK);
    ECGgraph.drawLine(gx, gy, gx, gy - gh, TFT_WHITE);           // X axis box left most
    ECGgraph.drawLine(gx + gw, gy, gx + gw, gy - gh, TFT_WHITE); // X axis box right most
    ECGgraph.drawLine(gx, gy - gh, gx + gw, gy - gh, TFT_WHITE); // X axis box top?
    ECGgraph.drawLine(gx, gy, gx + gw, gy, TFT_WHITE);           // X axis box bot?
    ECGgraph.pushToSprite(&sprite, 110, 30);
    draw_ECG(ECG_ecg_buf, ECG_ecg_bp, ECG_ecg_blen, ECG_BPM); // ECG line
    sprite.fillRect(10, 30, 90, 60, TFT_SKYBLUE);
    sprite.fillRect(10, 100, 90, 62, TFT_SKYBLUE);
    sprite.drawString("BPM", 53, 70);
    sprite.drawWideLine(20, 58, 90, 58, 2, TFT_BLUE);
    if ((ECG_BPM >= 0) && (!PauseRecord))
    {
      sprite.drawNumber(ECG_BPM, 55, 40);
    }
    else
      sprite.drawString(" 0 ", 55, 40);
    sprite.drawWideLine(20, 128, 90, 128, 2, TFT_BLUE);
    sprite.drawString("Uv", 53, 140);
    if (!PauseRecord)
    {
      sprite.drawNumber(Uv, 55, 110);
    }
  }
}
void ScreenHR()
{
  if (!Pause)
  {
    if (!PauseRecord)
    {
      if (millis() - TimeStart >= 1000)
      {
        CurrentSec++;
        TimeStart = millis();
      }
    }
    if (CurrentSec >= 60)
    {
      CurrentSec = CurrentSec - 60;
      CurrentMin++;
    }
    sprite.fillSmoothRoundRect(5, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK); // TimerBG
    if (String(CurrentSec).length() < 2)
    {
      sprite.drawString("0" + String(CurrentSec), 37, 8, 2);
    }
    else
      sprite.drawNumber(CurrentSec, 37, 8, 2);
    if (String(CurrentMin).length() < 2)
    {
      sprite.drawString("0" + String(CurrentMin), 17, 8, 2);
    }
    else
      sprite.drawNumber(CurrentMin, 17, 8, 2);
    sprite.drawString(":", 27, 8, 2);
    HRgraphV();
    PPGGraphV();
    sprite.drawLine(gx + 110, gy - 40, gx + 110 + gw, gy - 40, TFT_WHITE);
    sprite.drawLine(gx + 110, gy + 11, gx + 110 + gw, gy + 11, TFT_WHITE);
    sprite.fillRect(0, 0, 320, 20, TFT_BLACK);
    sprite.fillRect(10, 30, 90, 60, TFT_SKYBLUE);
    sprite.fillRect(10, 100, 90, 62, TFT_SKYBLUE);
    sprite.setTextSize(1);
    sprite.setTextColor(TFT_BLUE);
    sprite.setFreeFont(&Orbitron_Light_24);
    float IR = particleSensor.getIR();
    if (!PauseRecord)
    {
      if ((mbpm != 0) && (IR > 10000))
        sprite.drawNumber(mbpm, 55, 40);
      else
        sprite.drawString("---", 55, 40);
      sprite.drawWideLine(20, 58, 90, 58, 2, TFT_BLUE);
      sprite.drawString("BPM", 53, 70);
      if ((validSPO2) && (IR > 10000))
        sprite.drawNumber(Cur_SpO2, 55, 110);
      else
        sprite.drawString("---", 55, 110);
      sprite.drawWideLine(20, 128, 90, 128, 2, TFT_BLUE);
      sprite.drawString("SPO2", 53, 140);
      sprite.fillSmoothRoundRect(7, 0, 50, 16, 4, TFT_DARKGREY, TFT_BLACK); // Profile???
    }
    else
    {
      sprite.drawString("---", 55, 40);
      sprite.drawWideLine(20, 58, 90, 58, 2, TFT_BLUE);
      sprite.drawString("BPM", 53, 70);
      sprite.drawString("---", 55, 110);
      sprite.drawWideLine(20, 128, 90, 128, 2, TFT_BLUE);
      sprite.drawString("SPO2", 53, 140);
      sprite.fillSmoothRoundRect(7, 0, 50, 16, 4, TFT_DARKGREY, TFT_BLACK); // Profile???
    }
  }
  Topbar(); // print top bar status
}
void LoadingBar()
{
  if (BoxXcoord <= 248 + 60)
  {
    BoxXcoord += 8;
  }
  else
  {
    BoxXcoord = 0;
  }
  sprite.fillRect(20, 100, 280, 30, TFT_BLACK);
  sprite.fillRect(24, 104, 272, 22, TFT_DARKGREY);
  if (BoxXcoord > 248)
  {
    sprite.fillRect(24, 104, 248 + 24, 22, TFT_LIGHTGREY);
  }
  else
  {
    sprite.fillRect(24, 104, BoxXcoord + 24, 22, TFT_LIGHTGREY);
  }
  sprite.fillRect(24, 104, BoxXcoord + 24 - 60, 22, TFT_DARKGREY);
  sprite.fillRect(296, 100, 4, 30, TFT_BLACK);
}
void ScreenECG()
{
  ECGgraph.fillRect(gx, 0, gw, gh, TFT_BLUE);
  int ECG_BPM = get_BPM();
  int bat_lvl = get_uecg_batt();
  int16_t *ECG_ecg_buf = get_ecg_buf();
  int ECG_ecg_bp = get_ecg_buf_pos();
  int ECG_ecg_blen = get_ecg_buf_len();
  if (uECGConnected == 2) // Scanning
  {
    LoadingBar();
    sprite.fillRoundRect(20, 36, 280, 38, 4, TFT_LIGHTGREY);
    sprite.drawString("Scanning for uECG", 160, 53);
  }
  else if (uECGConnected == 3) // Connecting change back to 3
  {
    LoadingBar();
    sprite.setTextColor(TFT_BLACK);
    sprite.fillRoundRect(20, 36, 280, 38, 4, TFT_LIGHTGREY);
    sprite.drawString("Connecting to uECG", 160, 53);
  }
  else if (uECGConnected == 1) // Found
  {
    // Serial.println("Freefall = " + String(get_fall_detected()));
    ECGfoundbf = true;
    for (int i = 1; i < 10; i++)
      ECGgraph.drawLine(gx + (i * gw / 10), gy, gx + (i * gw / 10), gh - gy, color2); // Y axis line
    for (int i = 1; i < 9; i++)
    {
      ECGgraph.drawLine(gx, gy - (i * 9), gx + gw, gy - (i * 9), color2); // X axis line
    }
    Topbar();
    if (!PauseRecord)
    {
      if (millis() - TimeStart >= 1000)
      {
        CurrentSec++;
        TimeStart = millis();
      }
    }
    if (CurrentSec >= 60)
    {
      CurrentSec = CurrentSec - 60;
      CurrentMin++;
    }
    sprite.fillSmoothRoundRect(5, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK); // TimerBG
    if (String(CurrentSec).length() < 2)
    {
      sprite.drawString("0" + String(CurrentSec), 37, 8, 2);
    }
    else
      sprite.drawNumber(CurrentSec, 37, 8, 2);
    if (String(CurrentMin).length() < 2)
    {
      sprite.drawString("0" + String(CurrentMin), 17, 8, 2);
    }
    else
      sprite.drawNumber(CurrentMin, 17, 8, 2);
    sprite.drawString(":", 27, 8, 2);
    sprite.setTextColor(TFT_BLACK);
    ECGgraph.drawLine(gx, gy, gx, gy - gh, TFT_WHITE);           // X axis box left most
    ECGgraph.drawLine(gx + gw, gy, gx + gw, gy - gh, TFT_WHITE); // X axis box right most
    ECGgraph.drawLine(gx, gy - gh, gx + gw, gy - gh, TFT_WHITE); // X axis box top?
    ECGgraph.drawLine(gx, gy, gx + gw, gy, TFT_WHITE);           // X axis box bot?
    ECGgraph.pushToSprite(&sprite, 110, 30);
    draw_ECG(ECG_ecg_buf, ECG_ecg_bp, ECG_ecg_blen, ECG_BPM); // ECG line
    sprite.fillRect(10, 30, 90, 60, TFT_SKYBLUE);
    sprite.fillRect(10, 100, 90, 62, TFT_SKYBLUE);
    sprite.fillRect(125, 120, 170, 30, TFT_SKYBLUE); // batteryBox
    sprite.drawString("SENSOR BATTERY : ", 195, 134, 2);
    sprite.drawNumber(bat_lvl, 275, 134, 2);
    // sprite.drawString("100", 275, 134, 2);
    sprite.drawString("BPM", 53, 70);
    sprite.drawWideLine(20, 58, 90, 58, 2, TFT_BLUE);
    if ((ECG_BPM >= 0) && (!PauseRecord))
    {
      sprite.drawNumber(ECG_BPM, 55, 40);
    }
    else
      sprite.drawString(" 0 ", 55, 40);
    sprite.drawWideLine(20, 128, 90, 128, 2, TFT_BLUE);
    sprite.drawString("Uv", 53, 140);
    if (!PauseRecord)
    {
      sprite.drawNumber(Uv, 55, 110);
    }
    else
    {
      sprite.drawString("0", 55, 110);
    }
  }
}
void Topbar()
{
  sprite.fillRect(0, 0, 320, 20, TFT_BLACK); // top BG
  if ((ScreenNum == 3) && (StartRec))        // button for switching pages on multi
  {
    if (!PageSWHold)
    {
      if ((XCoord <= 10) && (XCoord >= 0) && (YCoord <= 65) && (YCoord >= 3))
      {
        sprite.fillSmoothRoundRect(6, 0, 65, 16, 4, TFT_LIGHTGREY, TFT_BLACK); // page sw bg
        XCoordLS = XCoord;
        PageSWHold = true;
      }
      else
      {
        sprite.fillSmoothRoundRect(6, 0, 65, 16, 4, TFT_DARKGREY, TFT_BLACK); // page sw bg
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillSmoothRoundRect(6, 0, 65, 16, 4, TFT_DARKGREY, TFT_BLACK); // page sw bg
        if (Multipage <= 2)                                                   // Loop 1-3
        {
          Multipage++;
        }
        else
        {
          Multipage = 1;
        }
        PageSWHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillSmoothRoundRect(6, 0, 65, 16, 4, TFT_LIGHTGREY, TFT_BLACK); // page sw bg
      }
      else
      {
        sprite.fillSmoothRoundRect(6, 0, 65, 16, 4, TFT_DARKGREY, TFT_BLACK); // page sw bg
        PageSWHold = false;
      }
    }
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("Page : " + String(Multipage), 38, 8, 2);
  }
  sprite.fillSmoothRoundRect(273, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK); // Batt bg
  if (BL.getBatteryVolts() >= MIN_USB_VOL)                                // if charging
  {
    sprite.setTextColor(TFT_GREEN);
  }
  else
  {
    sprite.setTextColor(TFT_BLACK);
  }
  sprite.drawString(String(BL.getBatteryChargeLevel()), 290, 8, 2);
  sprite.drawString("%", 307, 8, 2);
  sprite.setTextColor(TFT_BLACK);
}
void InfoSet()
{
  spr2.fillSprite(TFT_BLACK);
  spr2.setFreeFont(&Orbitron_Light_24);
  if (ScreenNum == 1)
  {
    spr2.fillRoundRect(2, 0, 73, 20, 3, TFT_MAGENTA);
    spr2.setTextColor(TFT_BLACK, TFT_MAGENTA);
  }
  else
  {
    spr2.fillRoundRect(2, 0, 73, 20, 3, TFT_WHITE);
    spr2.setTextColor(TFT_BLACK, TFT_WHITE);
  }
  spr2.drawString("EKG", 38, 10, 2);

  if (ScreenNum == 2)
  {
    spr2.fillRoundRect(77, 0, 73, 20, 3, TFT_MAGENTA);
    spr2.setTextColor(TFT_BLACK, TFT_MAGENTA);
  }
  else
  {
    spr2.fillRoundRect(77, 0, 73, 20, 3, TFT_WHITE);
    spr2.setTextColor(TFT_BLACK, TFT_WHITE);
  }
  spr2.drawString("SpO2+HR", 114, 10, 2);
  if (ScreenNum == 3)
  {
    spr2.fillRoundRect(152, 0, 73, 20, 3, TFT_MAGENTA);
    spr2.setTextColor(TFT_BLACK, TFT_MAGENTA);
  }
  else
  {
    spr2.fillRoundRect(152, 0, 73, 20, 3, TFT_WHITE);
    spr2.setTextColor(TFT_BLACK, TFT_WHITE);
  }
  spr2.drawString("Multi", 190, 10, 2);
  if (ScreenNum == 4)
  {
    spr2.fillRoundRect(227, 0, 73, 20, 3, TFT_MAGENTA);
    spr2.setTextColor(TFT_BLACK, TFT_MAGENTA);
  }
  else
  {
    spr2.fillRoundRect(227, 0, 73, 20, 3, TFT_WHITE);
    spr2.setTextColor(TFT_BLACK, TFT_WHITE);
  }
  spr2.drawString("HOME", 263, 10, 2);
  sprite.fillRoundRect(55, 45, 210, 103, 4, TFT_LIGHTGREY); // BG
}
void HRgraphV() // Prints a graph for HR
{
  int num;
  HRgraph.fillRect(gx, 0, gw, gh, TFT_BLUE);
  if (Beat)
  {
    num = 150;
    Beat = false;
  }
  else
  {
    num = 0;
  }
  curent = map(num, 0, 180, 0, gh);
  for (int i = 0; i < 26; i++)
    HRvalues2[i] = HRvalues[i];
  for (int i = 26; i > 0; i--)
    HRvalues[i - 1] = HRvalues2[i];
  HRvalues[25] = curent; // add data to last unit

  for (int i = 1; i < 10; i++)
  {
    HRgraph.drawLine(gx + (i * gw / 10), gy, gx + (i * gw / 10), gh - gy, color2); // Y axis line
  }

  for (int i = 1; i < 9; i++)
  {
    HRgraph.drawLine(gx, gy - (i * 9), gx + gw, gy - (i * 9), color2); // X axis line
    if (i == 1)
    {
      // HRgraph.drawString(String("Nan"), gx - 10, gy - (i * 9)); // Title
    }
    else if (i == 5)
    {
      // HRgraph.drawString(String("BT"), gx - 10, gy - (i * 9)); // Title
    }
  }

  for (int i = 0; i < 25; i++) // the graph add i for longer graph :)
  {
    if (!PauseRecord)
    {
      HRgraph.drawLine(gx + (i * 8), ly - HRvalues[i] - calib, gx + ((i + 1) * 8), ly - HRvalues[i + 1] - calib, TFT_ORANGE);
      HRgraph.drawLine(gx + (i * 8), ly - HRvalues[i] - 1 - calib, gx + ((i + 1) * 8), ly - HRvalues[i + 1] - 1 - calib, TFT_ORANGE);
    }
  }
  HRgraph.drawLine(gx, gy, gx, gy - gh, TFT_WHITE);           // X axis left most
  HRgraph.drawLine(gx + gw, gy, gx + gw, gy - gh, TFT_WHITE); // X axis right most
  HRgraph.drawLine(gx, gy + gh, gx, gy + gh, TFT_WHITE);      // X axis bottom?
  HRgraph.drawLine(gx, gy, gx + gw, gy, TFT_WHITE);           // X axis top?
  //--------------------------------------------
  HRgraph.pushToSprite(&sprite, 110, 30);
}
void PPGGraphV() // Prints a graph for PPG
{
  int16_t Diff = 0;                           // The difference between the Infra Red (IR) and Red LED raw results
  uint16_t ir, red;                           // raw results returned in these
  static float lastx = 1;                     // Last x position of trace
  static int lasty = TRACE_MIDDLE_Y_POSITION; // Last y position of trace, default to middle
  static float x = 1;                         // current x position of trace
  int32_t y = 0;                              // current y position of trace
  static uint32_t tsLastReport = 0;           // Last time BMP/O2 were checked
  static int32_t SensorOffset = 10000;

  ir = particleSensor.getIR();
  // Serial.print("IR = " + String(ir));
  red = particleSensor.getRed();
  // Serial.print("Red = " + String(red));
  particleSensor.nextSample();
  if (red < 10000)               // No pulse
    y = TRACE_MIDDLE_Y_POSITION; // Set Y to default flat line in middle
  else
  {
    // Plot our new point
    Diff = (ir - red);          // Get raw difference between the 2 LEDS
    Diff = Diff - SensorOffset; // Adjust the baseline of raw values by removing the offset (moves into a good range for scaling)
    Diff = Diff / SCALING;      // Scale the difference so that it appears at a good height on screen
    if (Diff < -HALF_TRACE_HEIGHT)
      SensorOffset += (SCALING * (abs(Diff) - 32));
    if (Diff > HALF_TRACE_HEIGHT)
      SensorOffset += (SCALING * (abs(Diff) - 32));

    y = Diff + (TRACE_MIDDLE_Y_POSITION - HALF_TRACE_HEIGHT); // These two lines move Y pos of trace to approx middle of display area
    y += TRACE_HEIGHT / 4;
  }
  // Serial.println("DIFF = " + String(y));
  PPGgraph.fillSprite(TFT_BLUE);
  // PPGgraph.fillRect(gx2, 70, gw2, gh2, TFT_BLUE);
  y = map(y, 0, 6800, 0, gh);
  curent = y + (gh2 / 3);
  // Serial.println("Final = " + String(y));
  for (int i = 0; i < 26; i++)
    PPGvalues2[i] = PPGvalues[i];
  for (int i = 25; i > 0; i--)
    PPGvalues[i - 1] = PPGvalues2[i];
  PPGvalues[25] = curent; // add data to last unit

  for (int i = 1; i < 10; i++)
    PPGgraph.drawLine(gx2 + (i * gw2 / 10), gy2, gx2 + (i * gw2 / 10), gy2 - gh2, color2); // Y axis line

  for (int i = 1; i < 9; i++)
  {
    PPGgraph.drawLine(gx2, gy2 - (i * 9), gx2 + gw2, gy2 - (i * 9), color2); // X axis line
    if (i == 1)
    {
      // HRgraph.drawString(String("Nan"), gx - 10, gy - (i * 9)); // Title
    }
    else if (i == 5)
    {
      // HRgraph.drawString(String("BT"), gx - 10, gy - (i * 9)); // Title
    }
  }

  for (int i = 0; i < 25; i++) // the graph
  {
    if (!PauseRecord)
    {
      PPGgraph.drawLine(gx2 + (i * 8), ly2 - PPGvalues[i] - calib, gx2 + ((i + 1) * 8), ly2 - PPGvalues[i + 1] - calib, TFT_ORANGE);
      PPGgraph.drawLine(gx2 + (i * 8), ly2 - PPGvalues[i] - 1 - calib, gx2 + ((i + 1) * 8), ly2 - PPGvalues[i + 1] - 1 - calib, TFT_ORANGE);
    }
  }
  PPGgraph.drawLine(gx2, gy2, gx2, gy2 - gh2, TFT_WHITE);             // X axis left most
  PPGgraph.drawLine(gx2 + gw2, gy2, gx2 + gw2, gy2 - gh2, TFT_WHITE); // X axis right most
  PPGgraph.drawLine(gx, gy + gh, gx, gy + gh, TFT_WHITE);             // X axis bottom?
  PPGgraph.drawLine(gx, gy, gx + gw, gy, TFT_WHITE);                  // X axis top?
                                                                      //--------------------------------------------
  PPGgraph.pushToSprite(&sprite, 110, 90);
}
void ButtonTouch()
{
  // For Left Arrow
  if (!SCRHoldL)
  {
    if ((XCoord <= 100) && (XCoord >= 50) && (YCoord <= 60) && (YCoord >= 0))
    {
      sprite.fillTriangle(15, 85, 35, 65, 35, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(20, 85, 31, 74, 31, 96, TFT_BLACK);
      XCoordLS = XCoord;
      SCRHoldL = true;
    }
    else
    {
      sprite.fillTriangle(15, 85, 35, 65, 35, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(20, 85, 31, 74, 31, 96, TFT_DARKGREY);
    }
  }
  else
  {
    if ((XCoord == 0) && (YCoord == 0))
    {
      sprite.fillTriangle(15, 85, 35, 65, 35, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(20, 85, 31, 74, 31, 96, TFT_BLACK);
      if (ScreenNum >= 2) // Loop 1-4 temp untill touch buttons
      {
        ScreenNum--;
      }
      else
      {
        ScreenNum = 4;
      }
      SCRHoldL = false;
    }
    else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
    {
      sprite.fillTriangle(15, 85, 35, 65, 35, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(20, 85, 31, 74, 31, 96, TFT_BLACK);
    }
    else
    {
      sprite.fillTriangle(15, 85, 35, 65, 35, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(20, 85, 31, 74, 31, 96, TFT_WHITE);
      SCRHoldL = false;
    }
  }
  // For Right Arrow
  if (!SCRHoldR)
  {
    if ((XCoord <= 100) && (XCoord >= 60) && (YCoord <= 320) && (YCoord >= 270))
    {
      XCoordRS = XCoord;
      sprite.fillTriangle(305, 85, 285, 65, 285, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(300, 85, 289, 74, 289, 96, TFT_BLACK);
      SCRHoldR = true;
    }
    else
    {
      sprite.fillTriangle(305, 85, 285, 65, 285, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(300, 85, 289, 74, 289, 96, TFT_DARKGREY);
    }
  }
  else
  {
    if ((XCoord == 0) && (YCoord == 0))
    {
      sprite.fillTriangle(305, 85, 285, 65, 285, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(300, 85, 289, 74, 289, 96, TFT_BLACK);
      if (ScreenNum <= 3) // Loop 1-4 temp untill touch buttons
      {
        ScreenNum++;
      }
      else
      {
        ScreenNum = 1;
      }
      SCRHoldR = false;
    }
    else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
    {
      sprite.fillTriangle(305, 85, 285, 65, 285, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(300, 85, 289, 74, 289, 96, TFT_BLACK);
    }
    else
    {
      sprite.fillTriangle(305, 85, 285, 65, 285, 105, TFT_LIGHTGREY);
      sprite.fillTriangle(300, 85, 289, 74, 289, 96, TFT_DARKGREY);
      SCRHoldR = false;
    }
  }
  if ((ScreenNum == 1) || (ScreenNum == 2) || (ScreenNum == 3))
  {
    if (!MeasureHold)
    {
      if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 156) && (YCoord >= 80))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_BLACK); // Measure
        MeasureHold = true;
      }
      else
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        if (ScreenNum == 2)
        {
          if (MaxConnect)
          {
            StartRec = true; // Start Measure
          }
          else
          {
            BGimage();
            sprite.fillRoundRect(32 - 10, 40, 255 + 20, 40, 4, TFT_LIGHTGREY);
            sprite.fillRoundRect(38 - 25, 85, 245 + 50, 40, 4, TFT_LIGHTGREY);
            sprite.setTextColor(TFT_BLACK);
            sprite.drawString("PLEASE CONNECT", 160, 57);
            sprite.drawString("EXTERNAL SENSOR", 158, 102);
            sprite.pushSprite(0, 0);
            delay(1500);
          }
          MeasureHold = false;
        }
        else if (ScreenNum == 3)
        {
          if (MaxConnect)
          {
            StartRec = true; // Start Measure
          }
          else
          {
            BGimage();
            sprite.fillRoundRect(32 - 10, 40, 255 + 20, 40, 4, TFT_LIGHTGREY);
            sprite.fillRoundRect(38 - 25, 85, 245 + 50, 40, 4, TFT_LIGHTGREY);
            sprite.setTextColor(TFT_BLACK);
            sprite.drawString("PLEASE CONNECT", 160, 57);
            sprite.drawString("EXTERNAL SENSOR", 158, 102);
            sprite.pushSprite(0, 0);
            delay(1500);
          }
          MeasureHold = false;
        }
        else
        {
          StartRec = true; // Start Measure
          MeasureHold = false;
        }
      }
      else if ((XCoord <= XCoordRS + 5) && (XCoord >= XCoordRS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_BLACK); // Measure
      }
      else
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        MeasureHold = false;
      }
    }
    int Xoffset = 100;
    if (!ProfileHold)
    {
      if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 256) && (YCoord >= 180))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_BLACK); // Measure
        ProfileHold = true;
      }
      else
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        Profileset = true;                                               // Start Measure
        ProfileHold = false;
      }
      else if ((XCoord <= XCoordRS + 5) && (XCoord >= XCoordRS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_BLACK); // Measure
      }
      else
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure

        ProfileHold = false;
      }
    }
    spr2.pushImage(72, 50, 60, 60, Heart);
    spr2.pushImage(172, 55, 60, 60, ProfileImg);
    sprite.setTextDatum(4);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("MEASURE", 112, 130, 2);
    sprite.drawString("PROFILE", 212, 130, 2);
  }
  else if (ScreenNum == 4)
  {
    if (!MeasureHold)
    {
      if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 156) && (YCoord >= 80))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_BLACK); // Measure
        MeasureHold = true;
      }
      else
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        StartRec = true;                                       // Start Measure
        MeasureHold = false;
      }
      else if ((XCoord <= XCoordRS + 5) && (XCoord >= XCoordRS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_BLACK); // Measure
      }
      else
      {
        sprite.fillRoundRect(67, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        MeasureHold = false;
      }
    }
    int Xoffset = 100;
    if (!ProfileHold)
    {
      if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 256) && (YCoord >= 180))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_BLACK); // Measure
        ProfileHold = true;
      }
      else
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure
        Profileset = true;                                               // Start Measure
        ProfileHold = false;
      }
      else if ((XCoord <= XCoordRS + 5) && (XCoord >= XCoordRS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_BLACK); // Measure
      }
      else
      {
        sprite.fillRoundRect(67 + Xoffset, 54, 85, 85, 4, TFT_DARKGREY); // Measure

        ProfileHold = false;
      }
    }
    spr2.pushImage(72, 50, 60, 60, RecordIcon);
    spr2.pushImage(172, 55, 60, 60, SettingGear);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("RECORDS", 112, 130, 2);
    sprite.drawString("SETTINGS", 212, 130, 2);
  }
}
void MeasureTouch() // Touch button for after start measuring
{
  if (!Pause) // all screens have same pause button
  {
    if (!PauseHold) // Entire Pause button
    {

      if ((XCoord <= 30) && (XCoord >= 0) && (YCoord <= 260) && (YCoord >= 80))
      {
        sprite.fillRect(120, 0, 80, 15, TFT_CYAN);
        sprite.fillTriangle(120 - 40, 0, 120, 0, 120, 15 - 2, TFT_CYAN);
        sprite.fillTriangle(120 + 80 + 40, 0, 120 + 80, 0, 120 + 80, 15 - 2, TFT_CYAN);
        sprite.drawWideLine(120 + 5, 8, 200 - 5, 8, 2, TFT_BLUE, TFT_DARKGREY);
        XCoordLS = YCoord;
        PauseHold = true;
      }
      else
      {
        sprite.fillRect(120, 0, 80, 15, TFT_BLUE);
        sprite.fillTriangle(120 - 40, 0, 120, 0, 120, 15 - 2, TFT_BLUE);
        sprite.fillTriangle(120 + 80 + 40, 0, 120 + 80, 0, 120 + 80, 15 - 2, TFT_BLUE);
        sprite.drawWideLine(120 + 5, 8, 200 - 5, 8, 2, TFT_CYAN, TFT_DARKGREY);
      }
    }
    else if ((XCoord == 0) && (YCoord == 0))
    {

      Pause = true;
      PauseHold = false;
    }
    else if ((XCoord <= XCoordLS + 10) && (XCoord >= XCoordLS - 10)) // Check X
    {
      if ((YCoord <= YCoordLS + 100) && (YCoord >= YCoordLS - 100)) // Check Y
      {

        sprite.fillRect(120, 0, 80, 15, TFT_CYAN);
        sprite.fillTriangle(120 - 40, 0, 120, 0, 120, 15 - 2, TFT_CYAN);
        sprite.fillTriangle(120 + 80 + 40, 0, 120 + 80, 0, 120 + 80, 15 - 2, TFT_CYAN);
        sprite.drawWideLine(120 + 5, 8, 200 - 5, 8, 2, TFT_BLUE, TFT_DARKGREY);
      }
      else
      {
        sprite.fillRect(120, 0, 80, 15, TFT_CYAN);
        sprite.fillTriangle(120 - 40, 0, 120, 0, 120, 15 - 2, TFT_CYAN);
        sprite.fillTriangle(120 + 80 + 40, 0, 120 + 80, 0, 120 + 80, 15 - 2, TFT_CYAN);
        sprite.drawWideLine(120 + 5, 8, 200 - 5, 8, 2, TFT_BLUE, TFT_DARKGREY);
        PauseHold = false;
      }
    }
    else
    {
      sprite.fillRect(120, 0, 80, 15, TFT_CYAN);
      sprite.fillTriangle(120 - 40, 0, 120, 0, 120, 15 - 2, TFT_CYAN);
      sprite.fillTriangle(120 + 80 + 40, 0, 120 + 80, 0, 120 + 80, 15 - 2, TFT_CYAN);
      sprite.drawWideLine(120 + 5, 8, 200 - 5, 8, 2, TFT_BLUE, TFT_DARKGREY);
      PauseHold = false;
    }
  }
  else
  { // If paused
    PauseFunction();
    Topbar();
    if (!PauseHold) // Pause button below
    {
      if ((XCoord <= 170) && (XCoord >= 140) && (YCoord <= 260) && (YCoord >= 65))
      {
        sprite.fillRect(120, why, 80, 15, TFT_CYAN);
        sprite.fillTriangle(120 - 40, why, 120, why, 120, why + 15 - 2, TFT_CYAN);
        sprite.fillTriangle(120 + 80 + 40, why, 120 + 80, why, 120 + 80, why + 15 - 2, TFT_CYAN);
        sprite.drawWideLine(120 + 5, why + 8, 200 - 5, why + 8, 2, TFT_BLUE, TFT_DARKGREY);
        XCoordLS = XCoord;
        YCoordLS = YCoord;
        PauseHold = true;
      }
      else
      {
        sprite.fillRect(120, why, 80, 15, TFT_BLUE);
        sprite.fillTriangle(120 - 40, why, 120, why, 120, why + 15 - 2, TFT_BLUE);
        sprite.fillTriangle(120 + 80 + 40, why, 120 + 80, why, 120 + 80, why + 15 - 2, TFT_BLUE);
        sprite.drawWideLine(120 + 5, why + 8, 200 - 5, why + 8, 2, TFT_CYAN, TFT_DARKGREY);
      }
    }
    else if ((XCoord == 0) && (YCoord == 0))
    {

      sprite.fillRect(120, why, 80, 15, TFT_BLUE);
      sprite.fillTriangle(120 - 40, why, 120, why, 120, why + 15 - 2, TFT_BLUE);
      sprite.fillTriangle(120 + 80 + 40, why, 120 + 80, why, 120 + 80, why + 15 - 2, TFT_BLUE);
      sprite.drawWideLine(120 + 5, why + 8, 200 - 5, why + 8, 2, TFT_CYAN, TFT_DARKGREY);

      Pause = false;
      STOP = false;
      PauseHold = false;
      CFSAVE = false;
      BARLENTH = 0;
    }
    else if ((XCoord <= XCoordLS + 10) && (XCoord >= XCoordLS - 10))
    {
      if ((YCoord <= YCoordLS + 10) && (YCoord >= YCoordLS - 10))
      { // Check X

        sprite.fillRect(120, why, 80, 15, TFT_CYAN);
        sprite.fillTriangle(120 - 40, why, 120, why, 120, why + 15 - 2, TFT_CYAN);
        sprite.fillTriangle(120 + 80 + 40, why, 120 + 80, why, 120 + 80, why + 15 - 2, TFT_CYAN);
        sprite.drawWideLine(120 + 5, why + 8, 200 - 5, why + 8, 2, TFT_BLUE, TFT_DARKGREY);
      }
    }
    else
    {
      PauseHold = false;
    }
  }
}
void MeasureHR() // HR and SPO2 HR works SPo2 works now
{
  float current_value_red = particleSensor.getRed();
  float current_value_ir = particleSensor.getIR();

  particleSensor.nextSample();                  // We're finished with this sample so move to next sample
  if ((current_value_red > 100000) && (!Pause)) // If pause screen is not on and Finger detected
  {
    if (millis() - finger_timestamp > kFingerCooldownMs)
    {
      finger_detected = true;
    }
  }
  else
  {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    mbpm = 0;     // reset
    Cur_SpO2 = 0; // reset
    finger_detected = false;
    finger_timestamp = millis();
  }
  if (finger_detected)
  {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);
    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if (!isnan(current_diff) && !isnan(last_diff))
    {

      // Detect Heartbeat - Zero-Crossing
      if (last_diff > 0 && current_diff < 0)
      {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0)
      {
        crossed = false;
      }

      // Detect Heartbeat - Falling Edge Threshold
      if (crossed && current_diff < kEdgeThreshold)
      {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300)
        {
          // Show Results
          int bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average(); // stat not working
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          if (50 <= spo2 <= 100)
          {
            validSPO2 = true;
            Cur_SpO2 = spo2;
          }
          else if (spo2 >= 100)
          {
            validSPO2 = true;
            Cur_SpO2 = 100;
          }
          else if (spo2 < 0)
          {
            Cur_SpO2 = 0;
          }
          else
            validSPO2 = false;
          if ((bpm > 0) && (bpm < 250)) // if valid
          {
            mbpm = bpm;
            int average_bpm = averager_bpm.process(bpm);
            int average_r = averager_r.process(r);
            int average_spo2 = averager_spo2.process(spo2);
          }
          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  }
  if (finger_detected) // finger_detected
  {
    if (mbpm != 0) // bpm graph
    {
      float Beatmsc = (60.00 * 1000.0) / mbpm;
      if (millis() - latestMIll >= Beatmsc)
      {
        Beat = true;
        latestMIll = millis();
      }
    }
  }
}
void BasicFolders()
{
  createDir(SD, "/IC3-Results");
  createDir(SD, "/IC3-Results/Profiles");
  createDir(SD, "/IC3-Results/Settings");
  createDir(SD, "/IC3-Results/Settings/WifiList");
}
void FolderCreate()
{
  const char *UserChar;
  String UserString;
  User = CurName;
  UserString = "/IC3-Results/" + User;
  UserChar = UserString.c_str();
  createDir(SD, UserChar);
  UserString = "/IC3-Results/" + User + "/Mode1ECG";
  UserChar = UserString.c_str();
  createDir(SD, UserChar);
  UserString = "/IC3-Results/" + User + "/Mode2HR";
  UserChar = UserString.c_str();
  createDir(SD, UserChar);
  UserString = "/IC3-Results/" + User + "/Mode3Multi";
  UserChar = UserString.c_str();
  createDir(SD, UserChar);
}
void TouchCode()
{
  char str_buf[100];
  static uint8_t last_finger;
#if TOUCH_GET_FORM_INT
  if (get_int)
  {
    get_int = 0;
    touch.read();
#else
  if (touch.read())
  {
#endif
    uint8_t n = touch.getPointNum();
    // sprintf(str_buf, "finger num :%d \r\n", n);
    tft.drawString(str_buf, 0, 15);
    for (uint8_t i = 0; i < n; i++)
    {
      TP_Point t = touch.getPoint(i);
      sprintf(str_buf, "x:%04d y:%04d p:%04d \r\n", t.x, t.y, t.pressure);
      // tft.drawString(str_buf, 0, 35 + (20 * i));
      XCoord = t.x;
      YCoord = t.y;
      PrCoord = t.pressure;
      LatestTouch = millis();
      // Serial.println("Xcoord = " + String(XCoord) + " Ycoord = " + String(YCoord) + " Prcoord = " + String(PrCoord));
    }
    last_finger = n;
  }
  else
  {
    // No finger detected
    XCoord = 0;
    YCoord = 0;
    PrCoord = 0;
    // Serial.println("Xcoord = " + String(XCoord) + " Ycoord = " + String(YCoord) + " Prcoord = " + String(PrCoord));
    //  tft.fillScreen(TFT_GREEN);
  }
}
void loop()
{
}
void PauseFunction()
{
  sprite.fillRect(0, 130, 320, 20, TFT_BLUE);
  sprite.fillRect(0, 0, 320, why - 10, TFT_LIGHTGREY);
  if (CFSAVE) // Check for Confirm Stop
  {
    sprite.setTextSize(1);
    sprite.fillRoundRect(35, 30, 250, 50, 4, TFT_BLACK);
    sprite.setTextColor(TFT_WHITE);
    if (BARLENTH < 100)
    {
      sprite.fillRect(38, 40, BARLENTH * 2.4, 30, TFT_BLUE);
      sprite.drawString("SAVING", 160, 53);
      BARLENTH = BARLENTH + 3;
      CurDisplayMS = millis();
    }
    else
    {
      BARLENTH = 100;
      if (millis() - CurDisplayMS < 2000)
      {
        if ((Sdvalid) && (ECGfoundbf))
        {
          sprite.drawString("FILE SAVED", 160, 55);
          sprite.setTextColor(TFT_BLACK);
          if (ScreenNum == 3)
            sprite.drawString("/IC3-Results/" + CurName + "/Mode3Multi/" + CurName + "_MULTI_" + String(MaxNum + 1) + ".txt", 160, 100, 2);
          else if (ScreenNum == 1)
            sprite.drawString("/IC3-Results/" + CurName + "/Mode1ECG/" + CurName + "_ECG_" + String(MaxNum + 1) + ".txt", 160, 100, 2);
          else if (ScreenNum == 2)
            sprite.drawString("/IC3-Results/" + CurName + "/Mode2HR/" + CurName + "_HR_" + String(MaxNum + 1) + ".txt", 160, 100, 2);
        }
        else if (!ECGfoundbf)
        {
          sprite.drawString("NOT SAVED", 160, 55);
          sprite.setTextColor(TFT_BLACK);
          sprite.drawString("NO uECG FOUND", 160, 100, 2);
        }
        else
        {
          sprite.drawString("NOT SAVED", 160, 55);
          sprite.setTextColor(TFT_BLACK);
          sprite.drawString("NO SD CARD DETECTED", 160, 100, 2);
        }
      }
      else
      {
        taskVibrate.enable();
        Pause = false;    // close pause screen
        StartRec = false; // close the Rec screen
        CFSAVE = false;
        STOP = false;
      }
    }
  }
  else if (StopCF)
  {
    sprite.fillRoundRect(35, 30, 250, 50, 4, TFT_BLACK);
    if (BARLENTH < 100)
    {
      sprite.fillRect(38, 40, BARLENTH * 2.4, 30, TFT_BLUE);
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("CLOSING", 160, 53);
      BARLENTH += 10;
      CurDisplayMS = millis();
    }
    else
    {
      BARLENTH = 100;
      STOP = false;
      CFSAVE = false;
      DelSave = true;
      StopHOLD = false;
      Pause = true;
      StartRec = false;
    }
  }
  else if (STOP)
  {
    if (1)
    {
      sprite.setTextSize(1);
      sprite.fillRoundRect(35, 30, 250, 50, 4, TFT_BLACK);
      sprite.setTextColor(TFT_WHITE);
      if (StopHOLD)
      {
        sprite.fillRect(38, 40, BARLENTH * 2.4, 30, TFT_GREEN);
      }
      else if (SaveHOLD)
      {
        sprite.fillRect(38, 40, BARLENTH * 2.4, 30, TFT_GOLD);
      }
      else
      {
        sprite.drawString("CONFIRM STOP?", 160, 48);
      }
      sprite.fillRoundRect(23, 110, 55, 20, 4, TFT_RED);
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("CANCEL", 50, 120, 2);
      sprite.fillRoundRect(131, 110, 60, 20, 4, TFT_GREEN);
      sprite.drawString("CONFIRM", 160, 120, 2);
      sprite.fillRoundRect(235, 110, 55, 20, 4, TFT_GOLD);
      sprite.drawString("SAVE", 260, 120, 2);
    }
    if (1) // All the buttons for SAVE CANCEL and SEND
    {
      if (!CancelHOLD)
      {
        if ((XCoord <= 134) && (XCoord >= 80) && (YCoord <= 76) && (YCoord >= 20))
        {
          BARLENTH = 0; // reset on press
          XCoordLS = XCoord;
          CancelHOLD = true;
        }
        else
        {
        }
      }
      else if ((XCoord == 0) && (YCoord == 0))
      {
        CancelHOLD = false;
        Pause = false;
        STOP = false;
        DelSave = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5))
      {
      }
      else
      {
        BARLENTH = 0;
        CancelHOLD = false;
      }
      if (!StopHOLD)
      {
        if ((XCoord <= 134) && (XCoord >= 80) && (YCoord <= 180) && (YCoord >= 120))
        {
          BARLENTH = 0; // reset on press
          XCoordLS = XCoord;
          StopHOLD = true;
        }
        else
        {
        }
      }
      else if ((XCoord == 0) && (YCoord == 0))
      {
        StopHOLD = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5))
      {
        if ((BARLENTH < 90) && (!StopCF))
        {
          BARLENTH += 10;
        }
        else
        {
          StopCF = true;
          BARLENTH = 0;
        }
      }
      else
      {
        BARLENTH = 0;
        StopHOLD = false;
      }
      if (!SaveHOLD)
      {
        if ((XCoord <= 134) && (XCoord >= 80) && (YCoord <= 300) && (YCoord >= 230))
        {
          BARLENTH = 0; // reset on press
          XCoordRS = XCoord;
          SaveHOLD = true;
        }
        else
        {
        }
      }
      else if ((XCoord == 0) && (YCoord == 0))
      {
        SaveHOLD = false;
      }
      else if ((XCoord <= XCoordRS + 5) && (XCoord >= XCoordRS - 5))
      {
        if (BARLENTH < 90)
        {
          BARLENTH += 10;
        }
        else
        { // end with save
          CFSAVE = true;
          DelSave = false;
          SaveHOLD = false;
          BARLENTH = 0;
        }
      }
      else
      {
        BARLENTH = 0;
        SaveHOLD = false;
      }
    }
  }
  else // IF no funtion is slected show options
  {
    if (PausePG == 1) // if 1 show resume and quit // if 2 show PF and Send
    {
      if (!ResumeHOLD)
      {
        if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 160) && (YCoord >= 40))
        {
          sprite.fillRoundRect(67, 40, 85, 85, 4, TFT_BLACK);
          XCoordLS = YCoord;
          ResumeHOLD = true;
        }
        else
        {
          sprite.fillRoundRect(67, 40, 85, 85, 4, TFT_DARKGREY);
        }
      }
      else if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(67, 40, 85, 85, 4, TFT_BLACK);
        if (!PauseRecord)
          PauseRecord = true;
        else
          PauseRecord = false;
        ResumeHOLD = false;
      }
      else if ((XCoord <= XCoordRS + 10) && (XCoord >= XCoordRS - 10))
      {
        sprite.fillRoundRect(67, 40, 85, 85, 4, TFT_BLACK);
      }
      else
      {
        sprite.fillRoundRect(67, 40, 85, 85, 4, TFT_DARKGREY);
        ResumeHOLD = false;
      }
      //-------------------------End------------------------------------
      if (!StopHOLD)
      {
        if ((XCoord <= 140) && (XCoord >= 50) && (YCoord <= 250) && (YCoord >= 180))
        {
          sprite.fillRoundRect(179, 40, 85, 85, 4, TFT_BLACK);
          XCoordLS = XCoord;
          StopHOLD = true;
        }
        else
        {
          sprite.fillRoundRect(179, 40, 85, 85, 4, TFT_DARKGREY);
        }
      }
      else if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(179, 40, 85, 85, 4, TFT_DARKGREY);
        STOP = true;
        CFSAVE = false;
        StopHOLD = false;
      }
      else if ((XCoord <= XCoordRS + 10) && (XCoord >= XCoordRS - 10))
      {
        sprite.fillRoundRect(179, 40, 85, 85, 4, TFT_BLACK);
      }
      else
      {
        sprite.fillRoundRect(179, 40, 85, 85, 4, TFT_DARKGREY);
        StopHOLD = false;
      }
      if (!PauseRecord)
      {
        sprite.fillRect(140 - 60, 54, 20, 50, TFT_LIGHTGREY); // Pause graphic
        sprite.fillRect(140 - 20, 54, 20, 50, TFT_LIGHTGREY); // Pause graphic
        sprite.drawString("PAUSE", 140 - 15 - 14, 115, 2);
      }
      else
      {
        sprite.fillTriangle(90 - 7, 76, 140 - 15, 54, 140 - 15, 114 - 15, TFT_LIGHTGREY); // Resume graphic
        sprite.drawString("RESUME", 140 - 15 - 14, 115, 2);
      }
      sprite.fillRect(140 + 62, 55, 40, 40, TFT_LIGHTGREY); // Stop graphic
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("STOP", 140 + 82, 115, 2);
    }
  }
}
void Backbutton()
{
  if (!BackHold) // BACK
  {
    if ((XCoord <= 5) && (XCoord >= 0) && (YCoord <= 50) && (YCoord >= 1))
    {
      XCoordRS = XCoord;
      sprite.fillSmoothRoundRect(10, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK);
      BackHold = true;
    }
    else
    {
      sprite.fillSmoothRoundRect(10, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
    }
  }
  else
  {
    if ((XCoord == 0) && (YCoord == 0))
    {
      if (ScreenNum == 4) // Home page Start and profile become records and settings
      {
        if (Profileset) // press the settings
        {
          if (!Wifisettings)
          {
            Profileset = false;
          }
          else
          {
            if (WifiPage == 0)
            {
              Wifisettings = false;
            }
            else if (WifiPage == 1)
            {
              WifiPage--;
            }
            else if (WifiPage == 2)
            {
              WifiPage = 0;
              WiFi.mode(WIFI_MODE_NULL); // Turn off wifi
              WifiSettingFR = false;
            }
            else if (WifiPage == 3)
            {
              WifiPage = 2;
              getSSID = false;
              getPW = false;
              FinalisedWIFI = false;
            }
          }
        }
        else if (StartRec) // RECORD Pressed
        {
          if (RecordsPage == 0)
          {
            StartRec = false;
          }
          else if (RecordsPage == 1)
          {
            PFfirstrun = true;
            RecordsPage--;
          }
          else if (RecordsPage == 2)
          {
            PFfirstrun = true;
            RecordsPage--;
          }
          else if (RecordsPage == 3)
          {
            if (!RecServer)
            {
              if (WifiEmpty)
              {
                WifiEmpty = false;
              }
              else
              {
                RecDel = false;
                PFfirstrun = true;
                RecordsPage--;
              }
            }
          }
        }
      }
      else
      {
        if (!StartRec && !Profileset) // Removes home buttons when Measuring
        {
        }
        else if (Profileset) // press the profile button
        {
          if (ProfileCRmode == 0)
          {
            PFfirstrun = false;
            if (ProfileSL)
            {
              ProfileSL = false;
            }
            else
            {
              Profileset = false;
            }
          }
          else if (ProfileCRmode == 1)
          {
            PFfirstrun = false;
            ProfileCRmode--;
            NewNameStr = "";
          }
          else if (ProfileCRmode == 2)
          {
            if (YearMonthDate > 1)
            {
              YearMonthDate--;
            }
            else
              ProfileCRmode--;
            NewGenderStr = "";
            NewAge = 0;
            Tens = 0;
            Ones = 0;
          }
          else if (ProfileCRmode == 3)
          {
            ProfileCRmode--;
          }
          else if (ProfileCRmode == 4)
          {
            ProfileCRmode--;
          }
          else if (ProfileCRmode == 5)
          {
            if (editPage == 1)
            {
              ProfileCRmode = 0;
              ProfileSL = true;
            }
            else if (editPage == 2)
            {
              if (YearMonthDate > 1)
              {
                YearMonthDate--;
              }
              else
                editPage--;
            }
            else if (editPage == 3)
            {
              editPage--;
            }
            else if (editPage == 4)
            {
              editPage--;
            }
          }
        }
      }
      ProfileSL = false;
      sprite.fillSmoothRoundRect(10, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
      BackHold = false;
    }
    else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
    {
      sprite.fillSmoothRoundRect(10, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK);
    }
    else
    {
      sprite.fillSmoothRoundRect(10, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
      BackHold = false;
    }
  }
  if ((ScreenNum == 4) && (Profileset))
  {
    if (!Wifisettings)
    {
      if (Sdvalid)
      {
        if (!MaleHold) // save on settings page
        {
          if ((XCoord <= 5) && (XCoord >= 0) && (YCoord <= 1000) && (YCoord >= 55))
          {
            XCoordRS = XCoord;
            sprite.fillSmoothRoundRect(63, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK);
            MaleHold = true;
          }
          else
          {
            sprite.fillSmoothRoundRect(63, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            File file = SD.open("/IC3-Results/Settings/Config.txt", FILE_WRITE); // recreate settings file
            String SettingsMsg = "Screen Brightness : " + String(SCBright) + ",\nScreen Timeout : " + String(ScreenOff) + ",\nVibrate : " + String(Vibrate) + "\n";
            file.print(SettingsMsg);
            file.close();
            sprite.fillSmoothRoundRect(63, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
            MaleHold = false;
            Profileset = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillSmoothRoundRect(63, 0, 43, 16, 4, TFT_DARKGREY, TFT_BLACK);
          }
          else
          {
            sprite.fillSmoothRoundRect(63, 0, 43, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
            MaleHold = false;
          }
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("SAVE", 32 + 53, 8, 2);
      }
    }
  }
  sprite.drawString("BACK", 32, 8, 2);
}
void DataLogger()
{
  if ((Sdvalid) && (ScreenNum <= 3) && (!PauseRecord)) // run only if theres a card and in page 1-3 and is not paused
  {
    if (StartRec) // If Start recorded
    {
      SDdump = SD.open("/"); // for updating bool
      if (!SDdump)
      {
        // Serial.println("NO SD detected");
      }
      else
      {
        CURMillis = millis() - StartMillis;
        float IR;
        if ((ScreenNum == 2) || (ScreenNum == 3))
          IR = particleSensor.getIR();
        // Serial.println("IR =" + String(IR));
        User = CurName;
        if ((millis() - LastCommit > 5000) || (LastCommit = 0)) // 5 sec
        {
          LastCommit = millis();
          if ((ScreenNum == 1) && (ECGfoundbf)) // ecg
          {
            if (!TextCreated)
            {
              ECGRec = true;
              StartMillis = millis();
              SDdump = SD.open("/IC3-Results/" + User + "/Mode1ECG");
              FindMAXName(SDdump, 0);
              TextCreated = true;
              SDdump = SD.open("/IC3-Results/" + User + "/Mode1ECG/" + User + "_ECG_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
              SDdump.print(RecYear + "\n");
              SDdump.print("Hr,Millis,Y-axis,Fall\n");
              SDdump.close();
              CURMillis = 0; //hopefully remove odd timing issue 
              StartMillis = millis();
            }
            SDdump = SD.open("/IC3-Results/" + User + "/Mode1ECG/" + User + "_ECG_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
            // Serial.println(FinalDATA); // delete later
            //  add data
            SDdump.print(FinalDATA);
            SDdump.close();
          }
          else if (ScreenNum == 2) // HR
          {
            if (!TextCreated)
            {
              SDdump = SD.open("/IC3-Results/" + User + "/Mode2HR");
              FindMAXName(SDdump, 0);
              TextCreated = true;
              SDdump = SD.open("/IC3-Results/" + User + "/Mode2HR/" + User + "_HR_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
              SDdump.print(RecYear + "\n");
              SDdump.print("millis,Profile,Mode,Hr,ECG,SpO2\n");
              SDdump.close();
            }
            SDdump = SD.open("/IC3-Results/" + User + "/Mode2HR/" + User + "_HR_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
            // add data
            SDdump.print(FinalDATA);
            SDdump.close();
          }
          else if ((ScreenNum == 3) && (ECGfoundbf)) // multi
          {
            if (!TextCreated)
            {
              ECGRec = true;
              StartMillis = millis();
              SDdump = SD.open("/IC3-Results/" + User + "/Mode3Multi");
              FindMAXName(SDdump, 0);
              TextCreated = true;
              SDdump = SD.open("/IC3-Results/" + User + "/Mode3Multi/" + User + "_ECG_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
              SDdump.print(RecYear + "\n");
              SDdump.print("millis,Profile,Mode,Hr,ECG,SpO2,IMU\n");
              SDdump.close();
            }
            SDdump = SD.open("/IC3-Results/" + User + "/Mode3Multi/" + User + "_Multi_" + String(MaxNum + 1) + ".txt", FILE_APPEND);
            // add data
            SDdump.print(FinalDATA);
            SDdump.close();
          }
          FinalDATA = ""; // empty string
                          // Serial.println("FINAL DATA =" + String(FinalDATA));
        }
        if (millis() - LastDataUpdate >= Interval)
        {
           if ((ScreenNum == 1) && (ECGRec)) // check if uecg has been found before starting
          {
            // pol all data from uecg
          }
          else if (ScreenNum == 2)
          {
            if (IR < 10000) // hand not on sen*
              FinalDATA = FinalDATA + CURMillis + "," + ",HR,-,-," + "\n";
            else
              FinalDATA = FinalDATA + CURMillis + "," + ",HR," + String(mbpm) + "," + String(Cur_SpO2) + "\n";
          }
          else if ((ScreenNum == 3) && (ECGRec)) // multi
          {
          }
          LastDataUpdate = millis();
          // Serial.println("POLL DATA");
        }
      }
    }
    else
    {
      TextCreated = false;
    }
    if (DelSave) // If hit quit without saving
    {
      // Serial.println("REMOVED data " + User + String(MaxNum + 1));
      if (ScreenNum == 3)
        SD.remove("/IC3-Results/" + User + "/Mode3Multi/" + User + "_ECG_" + String(MaxNum + 1) + ".txt");
      else if (ScreenNum == 1)
        SD.remove("/IC3-Results/" + User + "/Mode1ECG/" + User + "_ECG_" + String(MaxNum + 1) + ".txt");
      else if (ScreenNum == 2)
        SD.remove("/IC3-Results/" + User + "/Mode2HR/" + User + "_ECG_" + String(MaxNum + 1) + ".txt");
      DelSave = false; // delete only once
    }
  }
}

void draw_ECG(int16_t *ecg_buf, int ecg_buf_pos, int ecg_buf_len, int BPM)
{
  min_y = 123456;
  max_y = -123456;
  for (int ep = 0; ep < 199; ep++)
  {
    int ecg_bp = ecg_buf_pos - 199 + ep;
    if (ecg_bp < 0)
      ecg_bp += ecg_buf_len;
    int x = ep;
    int y = ecg_buf[ecg_bp];
    if (y < min_y)
      min_y = y;
    if (y > max_y)
      max_y = y;
  }
  max_y++;
  for (int ep = 0; ep < 199; ep++)
  {
    int ecg_bp = ecg_buf_pos - 199 + ep;
    if (ecg_bp < 0)
      ecg_bp += ecg_buf_len;
    int x = ep;
    int y = 50 - (ecg_buf[ecg_bp] - min_y) * 50 / (max_y - min_y);
    if (ep > 0)
    {
      if (!PauseRecord)
      {
        sprite.drawLine(prev_x + 111, prev_y + 40, x + 111, y + 40, TFT_ORANGE);         // Print out
        sprite.drawLine(prev_x + 111, prev_y + 40 - 1, x + 111, y + 40 - 1, TFT_ORANGE); // Print out
      }
    }
    prev_x = x;
    prev_y = y;
  }
}

void SettingScreen()
{
  Topbar();
  Backbutton();
  if (SettingsPage == 0)
  {
    sprite.fillRoundRect(160 - 100, 55, 200, 30, 10, TFT_LIGHTGREY);
    sprite.drawString("BRIGHTNESS", 160, 65);
    sprite.fillRoundRect(160 - 100, 120, 200, 20, 4, TFT_LIGHTGREY);
    sprite.fillCircle(70 + MoveX, 130, 20, TFT_BLACK);
  }
  else if (SettingsPage == 1)
  {
    sprite.fillRoundRect(160 - 110, 55, 220, 30, 10, TFT_LIGHTGREY);
    sprite.drawString("SLEEP-DELAY", 160, 65);
    // sprite.fillCircle(70 + MoveX, 130, 20, TFT_BLACK);
  }
  else if (SettingsPage == 2)
  {
    if (!Wifisettings)
    {
      WFfirstrun = true;
      sprite.fillRoundRect(160 - 120 + 10, 55, 130, 30, 10, TFT_LIGHTGREY);
      sprite.drawString("SOUND :", 100 + 10, 65);
    }
    else
    {
      WifiScr();
      WifiTouch();
    }
  }
}

void SettingAPI()
{
  File file;
  String SettingData = readFile(SD, "/IC3-Results/Settings/Config.txt");
  if (FileFound)
  {
    int str_len = SettingData.length() + 1;
    char char_array[str_len]; // input data from txt
    SettingData.toCharArray(char_array, str_len);
    char *pch;
    // printf("Splitting string \"%s\" into tokens:\n", char_array);
    pch = strtok(char_array, ",");
    int n = 0;
    String SLname, OutString;
    int lenth;
    while (pch != NULL)
    {
      // printf("%s\n", pch);
      SLname = String(pch);
      lenth = SLname.length() + 1;
      OutString = "";
      for (int digit = 0; digit < lenth - 1; digit++)
      {
        if (isDigit(SLname[digit]))
        {
          OutString = OutString + String(SLname[digit]);
        }
      }
      if (n == 0)
      {
        SCBright = OutString.toInt();
      }
      else if (n == 1)
      {
        ScreenOff = OutString.toInt();
      }
      else if (n == 2)
      {
        if (OutString.toInt() == 1)
        {
          Vibrate = true;
        }
        else
        {
          Vibrate = false;
        }
      }
      n++;
      pch = strtok(NULL, ",");
    }
    File PF = SD.open("/IC3-Results/Settings/WifiList");
    ScanWifiST(PF);
  }
  else
  {                                                           // reset to default settings
    SD.open("/IC3-Results/Settings/Config.txt", FILE_APPEND); // recreate settings file
    SCBright = 255;
    Vibrate = true;
    ScreenOff = 60;
    file = SD.open("/IC3-Results/Settings/Config.txt", FILE_WRITE);
    String SettingsMsg = "Screen Brightness : " + String(SCBright) + ",\nScreen Timeout : " + String(ScreenOff) + ",\nVibrate : " + String(Vibrate) + "\n";
    file.print(SettingsMsg);
    file.close();
  }
  MoveX = map(SCBright, 0, 255, 1, 190);
}
void SettingTouch()
{
  if (!Wifisettings)
  {
    if (!SCRHoldL)
    {
      if ((XCoord <= 120) && (XCoord >= 50) && (YCoord <= 40) && (YCoord >= 0))
      {
        sprite.fillCircle(-10, 100, 50, TFT_NAVY);
        XCoordLS = XCoord;
        SCRHoldL = true;
      }
      else
      {
        sprite.fillCircle(-10, 100, 50, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (SettingsPage >= 1)
        {
          SettingsPage--;
        }
        else
        {
          SettingsPage = 2;
        }
        sprite.fillCircle(-10, 100, 50, TFT_BLUE);
        SCRHoldL = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillCircle(-10, 100, 50, TFT_NAVY);
      }
      else
      {
        sprite.fillCircle(-10, 100, 50, TFT_BLUE);
        SCRHoldL = false;
      }
    }
    // For Right Arrow
    if (!SCRHoldR)
    {
      if ((XCoord <= 150) && (XCoord >= 50) && (YCoord <= 320) && (YCoord >= 280))
      {
        XCoordRS = XCoord;
        sprite.fillCircle(330, 100, 50, TFT_NAVY);
        SCRHoldR = true;
      }
      else
      {
        sprite.fillCircle(330, 100, 50, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {

        if (SettingsPage < 2) // Loop 1-4 temp untill touch buttons
        {
          SettingsPage++;
        }
        else
        {
          SettingsPage = 0;
        }
        sprite.fillCircle(330, 100, 50, TFT_BLUE);
        SCRHoldR = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillCircle(330, 100, 50, TFT_NAVY);
      }
      else
      {
        sprite.fillCircle(330, 100, 50, TFT_BLUE);
        SCRHoldR = false;
      }
    }
    sprite.fillTriangle(15 - 10, 85 + 15, 35 - 15, 65 + 15, 35 - 15, 105 + 15, TFT_CYAN);
    sprite.fillTriangle(305 + 10, 85 + 15, 285 + 15, 65 + 15, 285 + 15, 105 + 15, TFT_CYAN);
  }
  if (SettingsPage == 0)
  {
    // For Right Arrow
    if (!TenDwHold)
    {
      if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 255) && (YCoord >= 55))
      {
        sprite.fillCircle(70 + MoveX, 130, 15, TFT_DARKGREY);
        XCoordRS = XCoord;
        YCoordLS = YCoord;
        TenDwHold = true;
      }
      else
      {
        sprite.fillCircle(70 + MoveX, 130, 15, TFT_LIGHTGREY);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillCircle(70 + MoveX, 130, 15, TFT_LIGHTGREY);
        TenDwHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15) && (YCoord <= 255) && (YCoord >= 55)) // Don't need Y X alone works well
      {
        sprite.fillCircle(70 + MoveX, 130, 15, TFT_DARKGREY);
        MoveX = map(YCoord, 55, 255, 0, 190);
        SCBright = map(YCoord, 50, 255, 0, 255) + 20;
        if (SCBright >= 255)
        {
          SCBright = 255;
        }
      }
      else
      {
        sprite.fillCircle(70 + MoveX, 130, 15, TFT_LIGHTGREY);
        TenDwHold = false;
      }
    }
  }
  else if (SettingsPage == 1)
  {
    sprite.fillCircle(73, 120, 20, TFT_DARKGREY);
    sprite.fillCircle(320 - 73, 120, 20, TFT_DARKGREY);
    sprite.fillRoundRect(160 - 50, 110, 100, 30, 10, TFT_WHITE);
    if (ScreenOff != 900)
    {
      sprite.drawString(String(ScreenOff), 160, 120);
    }
    else
    {
      sprite.drawString("NONE", 160, 120);
    }
    if (!TenDwHold) // minus buttom
    {
      if ((XCoord <= 160) && (XCoord >= 100) && (YCoord <= 90) && (YCoord >= 53))
      {
        sprite.fillCircle(73, 120, 15, TFT_BLACK);
        XCoordRS = XCoord;
        TenDwHold = true;
      }
      else
      {
        sprite.fillCircle(73, 120, 15, TFT_LIGHTGREY);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        switch (ScreenOff)
        {
        case 15:
          ScreenOff = 900;
          break;
        case 30:
          ScreenOff = 15;
          break;
        case 60:
          ScreenOff = 30;
          break;
        case 120:
          ScreenOff = 60;
          break;
        case 300:
          ScreenOff = 120;
          break;
        case 600:
          ScreenOff = 300;
          break;
        case 900:
          ScreenOff = 600;
          break;
        }
        sprite.fillCircle(73, 120, 15, TFT_LIGHTGREY);
        TenDwHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillCircle(73, 120, 15, TFT_BLACK);
      }
      else
      {
        sprite.fillCircle(73, 120, 15, TFT_LIGHTGREY);
        TenDwHold = false;
      }
    }
    if (!OneDwHold) // minus button
    {
      if ((XCoord <= 160) && (XCoord >= 100) && (YCoord <= 267) && (YCoord >= 210))
      {
        sprite.fillCircle(320 - 73, 120, 15, TFT_BLACK);
        XCoordRS = XCoord;
        OneDwHold = true;
      }
      else
      {
        sprite.fillCircle(320 - 73, 120, 15, TFT_LIGHTGREY);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        switch (ScreenOff)
        {
        case 15:
          ScreenOff = 30;
          break;
        case 30:
          ScreenOff = 60;
          break;
        case 60:
          ScreenOff = 120;
          break;
        case 120:
          ScreenOff = 300;
          break;
        case 300:
          ScreenOff = 600;
          break;
        case 600:
          ScreenOff = 900;
          break;
        case 900:
          ScreenOff = 15;
          break;
        }
        sprite.fillCircle(320 - 73, 120, 15, TFT_LIGHTGREY);
        OneDwHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillCircle(320 - 73, 120, 15, TFT_BLACK);
      }
      else
      {
        sprite.fillCircle(320 - 73, 120, 15, TFT_LIGHTGREY);
        OneDwHold = false;
      }
    }
    // sprite.fillRect(60,118+20,20,10,TFT_BLACK);
    // sprite.fillRect(60,118,10,20,TFT_BLACK);
    sprite.drawString("+", 247, 115);
    sprite.drawString("-", 73, 115);
  }
  else if (SettingsPage == 2)
  {
    if (!Wifisettings)
    {
      sprite.fillRoundRect(200, 53, 35, 35, 14, TFT_DARKGREY);
      if (!OneDwHold) // minus button
      {
        if ((XCoord <= 85) && (XCoord >= 50) && (YCoord <= 230) && (YCoord >= 190))
        {
          if (Vibrate)
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_NAVY);
          }
          else
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_DARKGREY);
          }
          XCoordRS = XCoord;
          OneDwHold = true;
        }
        else
        {
          if (Vibrate)
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_BLUE);
          }
          else
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_LIGHTGREY);
          }
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Vibrate)
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_BLUE);
            Vibrate = false;
          }
          else
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_LIGHTGREY);
            Vibrate = true;
          }
          OneDwHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          if (Vibrate)
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_NAVY);
          }
          else
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_DARKGREY);
          }
        }
        else
        {
          if (Vibrate)
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_BLUE);
          }
          else
          {
            sprite.fillRoundRect(203, 56, 30, 30, 14, TFT_LIGHTGREY);
          }
          OneDwHold = false;
        }
      }
      if (!TenDwHold)
      {
        if ((XCoord <= 130) && (XCoord >= 100) && (YCoord <= 260) && (YCoord >= 50))
        {
          sprite.fillRoundRect(160 - 110, 100, 220, 30, 10, TFT_DARKGREY);
          XCoordRS = XCoord;
          YCoordLS = YCoord;
          TenDwHold = true;
        }
        else
        {
          sprite.fillRoundRect(160 - 110, 100, 220, 30, 10, TFT_LIGHTGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          Wifisettings = true;
          sprite.fillRoundRect(160 - 110, 100, 220, 30, 10, TFT_LIGHTGREY);
          TenDwHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15) && (YCoord <= 255) && (YCoord >= 55)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(160 - 110, 100, 220, 30, 10, TFT_DARKGREY);
        }
        else
        {
          sprite.fillRoundRect(160 - 110, 100, 220, 30, 10, TFT_LIGHTGREY);
          TenDwHold = false;
        }
      }
      sprite.drawString("WIFI-SETTINGS", 160, 110);
    }
  }
}
void ProfileTouch()
{
  if (ProfileCRmode != 0)
  {
    if (ProfileCRmode == 1) // If setting name
    {
      // For Left Arrow
      if (!SCRHoldL)
      {
        if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 60) && (YCoord >= 0))
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
          XCoordLS = XCoord;
          SCRHoldL = true;
        }
        else
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_DARKGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
          if (PageAlpha > 1) // Loop 1-4 temp untill touch buttons
          {
            PageAlpha--;
          }
          else
          {
            PageAlpha = 7;
          }
          SCRHoldL = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
        }
        else
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_WHITE);
          SCRHoldL = false;
        }
      }
      // For Right Arrow
      if (!SCRHoldR)
      {
        if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 270))
        {
          XCoordRS = XCoord;
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
          SCRHoldR = true;
        }
        else
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
          if (PageAlpha <= 6) // Loop 1-4 temp untill touch buttons
          {
            PageAlpha++;
          }
          else
          {
            PageAlpha = 1;
          }
          SCRHoldR = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
        }
        else
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
          SCRHoldR = false;
        }
      }
      sprite.fillRoundRect(40 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(110 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(180 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(250 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      if (!FAlphaHold) // first alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 70) && (YCoord >= 0))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
          FAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {

          if (NameLenth <= 9)
            NewNameStr += FDigit;
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
          FAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
        }
        else
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1

          FAlphaHold = false;
        }
      }
      if (!SAlphaHold) // Second alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 160) && (YCoord >= 80))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
          SAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (NameLenth <= 9)
            NewNameStr += SDigit;
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
          SAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
        }
        else
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
          SAlphaHold = false;
        }
      }
      if (!TAlphaHold) // Third alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 220) && (YCoord >= 170))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
          TAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (NameLenth <= 9)
            NewNameStr += TDigit;
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
          TAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
        }
        else
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3

          TAlphaHold = false;
        }
      }
      if (!FRAlphaHold) // Forth alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 290) && (YCoord >= 230))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
          FRAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {

          if (NameLenth <= 9)
            NewNameStr += FrDigit;
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
          FRAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
        }
        else
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4

          FRAlphaHold = false;
        }
      }
      if (!BackSpcHold) // BackSpace
      {
        if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 140) && (YCoord >= 20))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
          BackSpcHold = true;
        }
        else
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          NameLenth = NewNameStr.length();
          String NewNameMinus;
          for (i = 1; i < NameLenth; i++)
          {
            NewNameMinus += NewNameStr[i - 1];
          }
          NewNameStr = NewNameMinus;
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
          BackSpcHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
        }
        else
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
          BackSpcHold = false;
        }
      }
      if (!NextHold) // Next button
      {
        if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 300) && (YCoord >= 160))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
          NextHold = true;
        }
        else
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          String AdrString = "/IC3-Results/Profiles/" + NewNameStr + ".txt";
          const char *AdrChar = AdrString.c_str();
          String SomeData = readFile(SD, AdrChar);
          if (NameLenth == 0)
          {
            for (int time = 0; time < 20; time++)
            {
              sprite.setTextColor(TFT_RED);
              sprite.drawString("Not a valid name", 160, 35, 2);
              delay(15);
            }
          }
          else if (FileFound) // duplicate name
          {
            for (int time = 0; time < 20; time++)
            {
              sprite.setTextColor(TFT_RED);
              sprite.drawString("Duplicate name", 160, 35, 2);
              delay(15);
            }
          }
          else
          {
            ProfileCRmode = 2;
          }
          // change to page 2
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
          NextHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
        }
        else
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
          NextHold = false;
        }
      }
      sprite.setTextColor(TFT_BLACK);
      sprite.fillRoundRect(45, 42, 230, 50, 8, TFT_WHITE); // BG NAME BOX
      NameLenth = NewNameStr.length();
      sprite.drawString(NewNameStr, 60 + (NameLenth * 10), 60);
      sprite.drawString(String(FDigit), 30 + 24, 107 + 8);
      sprite.drawString(String(SDigit), 100 + 24, 107 + 8);
      sprite.drawString(String(TDigit), 170 + 24, 107 + 8);
      sprite.drawString(String(FrDigit), 240 + 24, 107 + 8);
      sprite.drawString("DELETE", 90, 150);
      sprite.drawString("CONFIRM", 230, 155, 2);
      ProfileCreate();
    }
    else if (ProfileCRmode == 2)
    {
      if (YearMonthDate != 3)
      {
        if (!TenUpHold)
        {
          if ((XCoord <= 60) && (XCoord >= 50) && (YCoord <= 60) && (YCoord >= 0))
          {
            sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
            XCoordLS = XCoord;
            TenUpHold = true;
          }
          else
          {
            sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_BLACK); // Ten up
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if (Tens <= 8)
            {
              Tens++;
            }
            else
            {
              Tens = 0;
            }
            sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
            TenUpHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          }
          else
          {
            sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
            TenUpHold = false;
          }
        }
        if (!TenDwHold)
        {
          if ((XCoord <= 165) && (XCoord >= 120) && (YCoord <= 70) && (YCoord >= 10))
          {
            sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
            XCoordLS = XCoord;
            TenDwHold = true;
          }
          else
          {
            sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if (Tens >= 1)
            {
              Tens--;
            }
            else
            {
              Tens = 9;
            }
            sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
            TenDwHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
          }
          else
          {
            sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
            TenDwHold = false;
          }
        }
        if (!OneUpHold)
        {
          if ((XCoord <= 70) && (XCoord >= 40) && (YCoord <= 165) && (YCoord >= 100))
          {
            sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
            XCoordLS = XCoord;
            OneUpHold = true;
          }
          else
          {
            sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if (Ones <= 8)
            {
              Ones++;
            }
            else
            {
              Ones = 0;
            }
            sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
            OneUpHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
          }
          else
          {
            sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
            OneUpHold = false;
          }
        }
        if (!OneDwHold)
        {
          if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 165) && (YCoord >= 100))
          {
            sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
            XCoordLS = XCoord;
            OneDwHold = true;
          }
          else
          {
            sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if (Ones >= 1)
            {
              Ones--;
            }
            else
            {
              Ones = 9;
            }
            sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
            OneDwHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
          }
          else
          {
            sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
            OneDwHold = false;
          }
        }
        if (((NewDate <= 31) && (NewDate != 0)) && (YearMonthDate == 1))
        {
          sprite.drawString("NEXT", 190 + 60, 149, 2);
          if (!NextHold)
          {
            if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
            {
              XCoordLS = XCoord;
              NextHold = true;
            }
            else
            {
            }
          }
          else
          {
            if ((XCoord == 0) && (YCoord == 0))
            {
              YearMonthDate = 2;
              Ones = 0;
              Tens = 0;
              Serial.println("Day =" + String(NewDate));
              NextHold = false;
            }
            else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
            {
            }
            else
            {
              NextHold = false;
            }
          }
        }
        else if (YearMonthDate == 1)
        {
          sprite.drawString("INVALID DATE", 190 + 60, 149, 2);
        }
        if ((YearMonthDate == 2) && ((NewMonth <= 12) && (NewMonth != 0)))
        {
          sprite.drawString("NEXT", 190 + 60, 149, 2);
          if (!NextHold)
          {
            if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
            {
              XCoordLS = XCoord;
              NextHold = true;
            }
            else
            {
            }
          }
          else
          {
            if ((XCoord == 0) && (YCoord == 0))
            {
              Ones = 0;
              Tens = 0;
              Serial.println("Month =" + String(NewMonth));
              YearMonthDate = 3;
              // ProfileCRmode = 3;
              NextHold = false;
            }
            else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
            {
            }
            else
            {
              NextHold = false;
            }
          }
        }
        else if (YearMonthDate == 2)
        {
          sprite.drawString("INVALID MONTH", 190 + 60, 149, 2);
        }
      }
      else // Set name
      {
        // For Left Arrow
        if (!SCRHoldL)
        {
          if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 60) && (YCoord >= 0))
          {
            sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
            XCoordLS = XCoord;
            SCRHoldL = true;
          }
          else
          {
            sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_DARKGREY);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
            if (YearPg > 1)
            {
              YearPg--;
            }
            else
            {
              YearPg = 3;
            }
            SCRHoldL = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
          }
          else
          {
            sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_WHITE);
            SCRHoldL = false;
          }
        }
        // For Right Arrow
        if (!SCRHoldR)
        {
          if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 270))
          {
            XCoordRS = XCoord;
            sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
            SCRHoldR = true;
          }
          else
          {
            sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
            if (YearPg <= 2)
            {
              YearPg++;
            }
            else
            {
              YearPg = 1;
            }
            SCRHoldR = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
          }
          else
          {
            sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
            sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
            SCRHoldR = false;
          }
        }
        sprite.fillRoundRect(40 - 17, 100, 65, 38, 4, TFT_DARKGREY);
        sprite.fillRoundRect(110 - 17, 100, 65, 38, 4, TFT_DARKGREY);
        sprite.fillRoundRect(180 - 17, 100, 65, 38, 4, TFT_DARKGREY);
        sprite.fillRoundRect(250 - 17, 100, 65, 38, 4, TFT_DARKGREY);
        if (!FAlphaHold) // first alphabet
        {
          if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 70) && (YCoord >= 0))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
            FAlphaHold = true;
          }
          else
          {
            sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {

            if (NameLenth <= 9)
              NewYear += String(Yearint);
            sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
            FAlphaHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
          }
          else
          {
            sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
            FAlphaHold = false;
          }
        }
        if (!SAlphaHold) // Second alphabet
        {
          if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 160) && (YCoord >= 80))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
            SAlphaHold = true;
          }
          else
          {
            sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if (NameLenth <= 9)
              NewYear += String(Yearint + 1);
            sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
            SAlphaHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
          }
          else
          {
            sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
            SAlphaHold = false;
          }
        }
        if (!TAlphaHold) // Third alphabet
        {
          if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 220) && (YCoord >= 170))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
            TAlphaHold = true;
          }
          else
          {
            sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {

            if (NameLenth <= 9)
            {
              if (YearPg != 3)
              {
                NewYear += String(Yearint + 2);
              }
              else
              {
                NewYear += String(0);
              }
            }
            sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
            TAlphaHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
          }
          else
          {
            sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3

            TAlphaHold = false;
          }
        }
        if (!FRAlphaHold) // Forth alphabet
        {
          if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 290) && (YCoord >= 230))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
            FRAlphaHold = true;
          }
          else
          {
            sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {

            if (NameLenth <= 9)
            {
              if (YearPg != 3)
              {
                NewYear += String(Yearint + 3);
              }
              else
              {
                NewYear += String(1);
              }
            }
            sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
            FRAlphaHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
          }
          else
          {
            sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4

            FRAlphaHold = false;
          }
        }
        if (!BackSpcHold) // BackSpace
        {
          if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 140) && (YCoord >= 20))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
            BackSpcHold = true;
          }
          else
          {
            sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            NameLenth = NewYear.length();
            String NewYearMinus;
            for (i = 1; i < NameLenth; i++)
            {
              NewYearMinus += NewYear[i - 1];
            }
            NewYear = NewYearMinus;
            sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
            BackSpcHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
          }
          else
          {
            sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
            BackSpcHold = false;
          }
        }
        if (!NextHold) // Next button
        {
          if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 300) && (YCoord >= 160))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
            NextHold = true;
          }
          else
          {
            sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            if ((NewYear == "0") || (NewYear.toInt() > 2050))
            {
              sprite.setTextColor(TFT_RED);
              sprite.drawString("NOT A VALID YEAR", 160, 35, 2);
              delay(500);
            }
            else
            {
              NewDMY = String(NewDate) + "." + String(NewMonth) + "." + NewYear;
              YearPg = 1;
              NewYear = "";
              ProfileCRmode = 3;
            }
            sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
            NextHold = false;
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
          }
          else
          {
            sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
            NextHold = false;
          }
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.fillRoundRect(45, 42, 230, 50, 8, TFT_WHITE); // BG NAME BOX
        NameLenth = NewYear.length();
        sprite.drawString(String(NewYear), 60 + (NameLenth * 10), 60);
        if (YearPg == 1) // 0123
        {
          Yearint = 0;
        }
        else if (YearPg == 2) // 4567
        {
          Yearint = 4;
        }
        else if (YearPg == 3) // 8901
        {
          Yearint = 8;
        }
        sprite.drawString(String(Yearint), 30 + 24, 107 + 8);
        sprite.drawString(String(Yearint + 1), 100 + 24, 107 + 8);
        if (YearPg != 3)
        {
          sprite.drawString(String(Yearint + 2), 170 + 24, 107 + 8);
          sprite.drawString(String(Yearint + 3), 240 + 24, 107 + 8);
        }
        else
        {
          sprite.drawString(String("0"), 170 + 24, 107 + 8);
          sprite.drawString(String("1"), 240 + 24, 107 + 8);
        }
        sprite.drawString("DELETE", 90, 150);
        sprite.drawString("CONFIRM", 230, 155, 2);
        ProfileCreate();
      }
    }
    else if (ProfileCRmode == 3)
    {
      if (!MaleHold)
      {
        if ((XCoord <= 100) && (XCoord >= 60) && (YCoord <= 240) && (YCoord >= 180))
        {
          sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
          XCoordLS = XCoord;
          MaleHold = true;
        }
        else
        {
          if (NewGenderStr != "Male")
          {
            sprite.drawRoundRect(200 + 4, 60, 40, 40, 4, TFT_BLACK); // Male
          }
          else
          {
            sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
          }
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          NewGenderStr = "Male";
          sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
          MaleHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
        }
        else
        {
          sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
          MaleHold = false;
        }
      }
      if (!FemaleHold)
      {
        if ((XCoord <= 100) && (XCoord >= 60) && (YCoord <= 300) && (YCoord >= 260))
        {
          sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
          XCoordLS = XCoord;
          FemaleHold = true;
        }
        else
        {
          if (NewGenderStr != "Female")
          {
            sprite.drawRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLACK); // Female
          }
          else
          {
            sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
          }
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          NewGenderStr = "Female";
          sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
          FemaleHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
        }
        else
        {
          sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
          FemaleHold = false;
        }
      }
      if (!TenUpHold)
      {
        if ((XCoord <= 60) && (XCoord >= 50) && (YCoord <= 60) && (YCoord >= 0))
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          XCoordLS = XCoord;
          TenUpHold = true;
        }
        else
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_BLACK); // Ten up
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Tens <= 8)
          {
            Tens++;
          }
          else
          {
            Tens = 0;
          }
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          TenUpHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
        }
        else
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          TenUpHold = false;
        }
      }
      if (!TenDwHold)
      {
        if ((XCoord <= 165) && (XCoord >= 120) && (YCoord <= 70) && (YCoord >= 10))
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
          XCoordLS = XCoord;
          TenDwHold = true;
        }
        else
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Tens >= 1)
          {
            Tens--;
          }
          else
          {
            Tens = 9;
          }
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
          TenDwHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
        }
        else
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
          TenDwHold = false;
        }
      }
      if (!OneUpHold)
      {
        if ((XCoord <= 70) && (XCoord >= 40) && (YCoord <= 165) && (YCoord >= 100))
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
          XCoordLS = XCoord;
          OneUpHold = true;
        }
        else
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Ones <= 8)
          {
            Ones++;
          }
          else
          {
            Ones = 0;
          }
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
          OneUpHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
        }
        else
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
          OneUpHold = false;
        }
      }
      if (!OneDwHold)
      {
        if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 165) && (YCoord >= 100))
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
          XCoordLS = XCoord;
          OneDwHold = true;
        }
        else
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Ones >= 1)
          {
            Ones--;
          }
          else
          {
            Ones = 9;
          }
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
          OneDwHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
        }
        else
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
          OneDwHold = false;
        }
      }
      if ((NewAge != 0) && (NewGenderStr != ""))
      {
        sprite.drawString("CONFIRM", 190 + 60, 149, 2);
        if (!NextHold)
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
          {
            XCoordLS = XCoord;
            NextHold = true;
          }
          else
          {
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            ProfileCRmode = 4;
            NextHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
          }
          else
          {
            NextHold = false;
          }
        }
      }
    }
    else if (ProfileCRmode == 4)
    {
      if (!FinaliseInput)
      {
        if (!NextHold) // confirm button
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 250) && (YCoord >= 150))
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
            XCoordLS = XCoord;
            NextHold = true;
          }
          else
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            FinaliseInput = true;
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
            NextHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          }
          else
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
            NextHold = false;
          }
        }
        if (!BackHold) // Back button //reuse vall
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 145) && (YCoord >= 80))
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
            XCoordLS = XCoord;
            BackHold = true;
          }
          else
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            ProfileCRmode--;
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
            BackHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          }
          else
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
            BackHold = false;
          }
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("CONFIRM", 193, 150, 2);
        sprite.drawString("BACK", 120, 150, 2);
      }
    }
  }
  else
  {
    YearMonthDate = 1;
    if (!ProfileSL) // Profile not selected
    {
      sprite.fillRoundRect(265, 27, 50, 40, 2, TFT_DARKGREY);
      sprite.fillRoundRect(265, 71, 50, 50, 2, TFT_DARKGREY);
      sprite.fillRoundRect(265, 126, 50, 40, 2, TFT_DARKGREY);
      if (!ProfileDWHold)
      {
        if ((XCoord <= 70) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileDWHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          ProfileRw--;
          ButtonTouched = true;
          // Serial.println("ProfileRW =" + String(ProfileRw));
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
          ProfileDWHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
          ProfileDWHold = false;
        }
      }
      if (!ProfileUPHold)
      {
        if ((XCoord <= 170) && (XCoord >= 125) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileUPHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          ProfileRw++;
          ButtonTouched = true;
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
          ProfileUPHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
          ProfileUPHold = false;
        }
      }
      if (!ProfileSLHold)
      {
        if ((XCoord <= 120) && (XCoord >= 70) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileSLHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          ProfileSL = true;
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
          ProfileSLHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
          ProfileSLHold = false;
        }
      }
      sprite.fillTriangle(280, 36 + 20, 290, 36, 300, 36 + 20, TFT_CYAN);
      sprite.fillTriangle(280, 155 - 20, 290, 155, 300, 155 - 20, TFT_CYAN);
      sprite.fillCircle(289, 95, 15, TFT_LIGHTGREY);
    }
    else
    {
      //-------------------Profile manage screen----------------------------------
      if (confirmDEL) // delete confirm
      {
        if (!NextHold) // confirm button
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 250) && (YCoord >= 150))
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_ORANGE);
            XCoordLS = XCoord;
            NextHold = true;
          }
          else
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            confirmDEL = false;
            ProfileSL = false;
            sprite.drawString("....DELETING....", 160, 35);
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
            String DELString = "/IC3-Results/Profiles/" + CurName + ".txt";
            const char *DelChar = DELString.c_str();
            removeFile(SD, DelChar);
            ButtonTouched = true;
            delay(250);
            NextHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_ORANGE);
          }
          else
          {
            sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
            NextHold = false;
          }
        }

        if (!CancelDeleteHOLD) // Back button //reuse vall
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 145) && (YCoord >= 80))
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
            XCoordLS = XCoord;
            CancelDeleteHOLD = true;
          }
          else
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            confirmDEL = false;
            ProfileSL = false;
            ButtonTouched = true;
            CancelDeleteHOLD = false;
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          }
          else
          {
            sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
            CancelDeleteHOLD = false;
          }
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("DELETE", 193, 150, 2);
        sprite.drawString("BACK", 122, 150, 2);
      }
      else
      {
        if (!ProfileUPHold) // Back buttom
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 95) && (YCoord >= 0))
          {
            sprite.fillRoundRect(10, 130 + 8, 70, 30, 4, TFT_BLACK);
            XCoordLS = XCoord;
            ProfileUPHold = true;
          }
          else
          {
            sprite.fillRoundRect(10, 130 + 8, 70, 30, 4, TFT_DARKGREY);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            ProfileSL = false;
            sprite.fillRoundRect(10, 130 + 8, 70, 30, 4, TFT_DARKGREY);
            ProfileUPHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(10, 130 + 8, 70, 30, 4, TFT_BLACK);
          }
          else
          {
            sprite.fillRoundRect(10, 130 + 8, 70, 30, 4, TFT_DARKGREY);
            ProfileUPHold = false;
          }
        }
        if (!ProfileDWHold) // delete
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 150) && (YCoord >= 99))
          {
            sprite.fillRoundRect(85, 130 + 8, 70, 30, 4, TFT_ORANGE);
            XCoordLS = XCoord;
            ProfileDWHold = true;
          }
          else
          {
            sprite.fillRoundRect(85, 130 + 8, 70, 30, 4, TFT_RED);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            confirmDEL = true;
            sprite.fillRoundRect(85, 130 + 8, 70, 30, 4, TFT_RED);
            ProfileDWHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(85, 130 + 8, 70, 30, 4, TFT_ORANGE);
          }
          else
          {
            sprite.fillRoundRect(85, 130 + 8, 70, 30, 4, TFT_RED);
            ProfileDWHold = false;
          }
        }
        if (!SaveHOLD) // edit //reuse vall
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 225) && (YCoord >= 160))
          {
            sprite.fillRoundRect(160, 130 + 8, 70, 30, 4, TFT_BLUE);
            XCoordLS = XCoord;
            SaveHOLD = true;
          }
          else
          {
            sprite.fillRoundRect(160, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            ProfileCRmode = 5;
            editPage = 1;
            ProfileSL = false;
            sprite.fillRoundRect(160, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
            SaveHOLD = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(160, 130 + 8, 70, 30, 4, TFT_BLUE);
          }
          else
          {
            sprite.fillRoundRect(160, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
            SaveHOLD = false;
          }
        }
        if (!ProfileSLHold) // select button
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 230))
          {
            sprite.fillRoundRect(235, 130 + 8, 70, 30, 4, TFT_DARKGREEN);
            XCoordLS = XCoord;
            ProfileSLHold = true;
          }
          else
          {
            sprite.fillRoundRect(235, 130 + 8, 70, 30, 4, TFT_GREEN);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            Profileset = false;
            ProfileSL = false;
            ProfileCRmode = 0;
            sprite.fillRoundRect(235, 130 + 8, 70, 30, 4, TFT_GREEN);
            ProfileSLHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(235, 130 + 8, 70, 30, 4, TFT_DARKGREEN);
          }
          else
          {
            sprite.fillRoundRect(235, 130 + 8, 70, 30, 4, TFT_GREEN);
            ProfileSLHold = false;
          }
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("BACK", 10 + 37, 150, 2);
        sprite.drawString("DELETE", 85 + 37, 150, 2);
        sprite.drawString("EDIT", 160 + 37, 150, 2);
        sprite.drawString("SELECT", 235 + 37, 150, 2);
        sprite.setTextSize(1);
      }
      sprite.setTextSize(1);
      sprite.setTextDatum(4);
      sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("Profile Selected", 160, 35);
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("Name : " + CurName, 160, 65);
      sprite.drawString("Sex : " + CurSex, 160, 90);
      sprite.drawString("Age : " + CurAge, 160, 115);
      sprite.drawString(RecYear, 160, 8, 2);
      EditName = CurName;
      EditAge = CurAge.toInt();
      EditTens = (EditAge / 10) % 10;
      EditOnes = EditAge % 10;
      EditGender = CurSex;
      // sprite.fillRoundRect(40, 130 + 8, 100, 30, 4, TFT_DARKGREY);
      // sprite.fillRoundRect(180, 130 + 8, 100, 30, 4, TFT_SKYBLUE);
      sprite.setTextSize(1);
    }
  }
}
void ProfileCreate()
{
  if (PageAlpha == 1)
  { // 1-6 A-X
    FDigit = "A";
    SDigit = "B";
    TDigit = "C";
    FrDigit = "D";
  }
  else if (PageAlpha == 2)
  {
    FDigit = "E";
    SDigit = "F";
    TDigit = "G";
    FrDigit = "H";
  }
  else if (PageAlpha == 3)
  {
    FDigit = "I";
    SDigit = "J";
    TDigit = "K";
    FrDigit = "L";
  }
  else if (PageAlpha == 4)
  {
    FDigit = "M";
    SDigit = "N";
    TDigit = "O";
    FrDigit = "P";
  }
  else if (PageAlpha == 5)
  {
    FDigit = "Q";
    SDigit = "R";
    TDigit = "S";
    FrDigit = "T";
  }
  else if (PageAlpha == 6)
  {
    FDigit = "U";
    SDigit = "V";
    TDigit = "W";
    FrDigit = "X";
  }
  else if (PageAlpha == 7)
  {
    FDigit = "Y";
    SDigit = "Z";
    TDigit = "A";
    FrDigit = "B";
  }
}
void Profilelist(File dir2) // Lists profiles found
{
  // Don't edit this one
  if ((ButtonTouched) || (PFfirstrun))
  {
    PFfirstrun = false;
    Profileamount = 0;
    ButtonTouched = false;
    while (true)
    {
      File entry = dir2.openNextFile();
      if (!entry)
      {
        // no more files
        break;
      }
      OBJname = entry.name();
      // Serial.println(OBJname);
      OBJLenth = OBJname.length();
      String AdrString = "/IC3-Results/Profiles/" + OBJname;
      const char *AdrChar = AdrString.c_str();
      FileData = readFile(SD, AdrChar);
      int str_len = FileData.length() + 1;
      char char_array[str_len]; // input data from txt
      FileData.toCharArray(char_array, str_len);
      char *pch;
      // printf("Splitting string \"%s\" into tokens:\n", char_array);
      pch = strtok(char_array, ",");
      int n = 0;
      String Name, Sex, Age;
      while (pch != NULL)
      {
        // printf("%s\n", pch);
        if (n == 0)
        {
          Name = String(pch);
          n++;
        }
        else if (n == 1)
        {
          Age = String(pch);
          n++;
        }
        else if (n == 2)
        {
          Sex = String(pch);
          n++;
        }
        else if (n == 3)
        {
          DMY = String(pch);

          n = 0;
        }
        pch = strtok(NULL, ",");
      }
      Profileamount++;
      // Serial.print("InternalRW = " + String(InternalRw)); // 1,2,3,4
      if (Profileamount == (1 + InternalRw)) // 1+0 is first data 1+1 is second
      {
        Row2N = Name;
        Row2S = Sex;
        Row2A = Age;
        Row2Y = DMY;
        ONEran = true;
        TWOran = false;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (2 + InternalRw))
      {
        Row3N = Name;
        Row3S = Sex;
        Row3A = Age;
        Row3Y = DMY;
        TWOran = true;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (3 + InternalRw))
      {
        Row4N = Name;
        Row4S = Sex;
        Row4A = Age;
        Row4Y = DMY;
        THREEran = true;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (4 + InternalRw))
      {
        Row5N = Name;
        Row5S = Sex;
        Row5A = Age;
        Row5Y = DMY;
        FOURran = true;
        FIVEran = false;
      }
      else if (Profileamount == (5 + InternalRw))
      {
        Row6N = Name;
        Row6S = Sex;
        Row6A = Age;
        Row6Y = DMY;
        FIVEran = true;
      }
      else
      {
        ONEran = false;
        TWOran = false;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      // print out the name here
      // Serial.println(entry.name());
      entry.close();
    }
  }
  if (Profileamount <= (4 + InternalRw))
  {
    if (!ONEran) // remove empty results
    {
      Row2N = "";
      Row2S = "";
      Row2A = "";
      Row2Y = "";
    }
    if (!TWOran)
    {
      Row3N = "";
      Row3S = "";
      Row3A = "";
      Row3Y = "";
    }
    if (!THREEran)
    {
      Row4N = "";
      Row4S = "";
      Row4A = "";
      Row4Y = "";
    }
    if (!FOURran)
    {
      Row5N = "";
      Row5S = "";
      Row5A = "";
      Row5Y = "";
    }
    if (!FIVEran)
    {
      Row6N = "";
      Row6S = "";
      Row6A = "";
      Row6Y = "";
    }
  }
}

void Wifilist(File dir2) // Lists Wifi names found
{
  if ((ButtonTouched) || (WFfirstrun))
  {
    WFfirstrun = false;
    Profileamount = 0;
    ButtonTouched = false;
    while (true)
    {
      File entry = dir2.openNextFile();
      if (!entry)
      {
        // no more files
        break;
      }
      OBJname = entry.name();
      OBJLenth = OBJname.length();
      String AdrString = "/IC3-Results/Settings/WifiList/" + OBJname;
      const char *AdrChar = AdrString.c_str();
      FileData = readFile(SD, AdrChar);
      int str_len = FileData.length() + 1;
      char char_array[str_len]; // input data from txt
      FileData.toCharArray(char_array, str_len);
      char *pch;
      pch = strtok(char_array, "^");
      int n = 0;
      String Name, PW, State;
      Name = OBJname;
      Name = Name.substring(0, Name.length() - 4);
      // Serial.println("FinalName = " + Name);
      while (pch != NULL)
      {
        if (n == 0)
        {
          PW = String(pch);
          n++;
        }
        else if (n == 1)
        {
          State = String(pch);
          if (State == "YES")
          {
            CurWifi = Name;
            CurPW = PW;
          }
          n = 0;
        }
        pch = strtok(NULL, "^");
      }
      Profileamount++;
      if (Profileamount == (1 + InternalRw)) // 1+0 is first data 1+1 is second
      {
        Row2W = Name;
        Row2P = PW;
        Row2ST = State;
        ONEran = true;
        TWOran = false;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (2 + InternalRw))
      {
        Row3W = Name;
        Row3P = PW;
        Row3ST = State;
        TWOran = true;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (3 + InternalRw))
      {
        Row4W = Name;
        Row4P = PW;
        Row4ST = State;
        THREEran = true;
        FOURran = false;
        FIVEran = false;
      }
      else if (Profileamount == (4 + InternalRw))
      {
        Row5W = Name;
        Row5P = PW;
        Row5ST = State;
        FOURran = true;
        FIVEran = false;
      }
      else if (Profileamount == (5 + InternalRw))
      {
        Row6W = Name;
        Row6P = PW;
        Row6ST = State;
        FIVEran = true;
      }
      else
      {
        ONEran = false;
        TWOran = false;
        THREEran = false;
        FOURran = false;
        FIVEran = false;
      }
      // print out the name here
      // Serial.println(entry.name());
      entry.close();
    }
  }
  if (Profileamount <= (4 + InternalRw))
  {
    if (!ONEran) // remove empty results
    {
      Row2W = "";
      Row2P = "";
      Row2ST = "";
    }
    if (!TWOran)
    {
      Row3W = "";
      Row3P = "";
      Row3ST = "";
    }
    if (!THREEran)
    {
      Row4W = "";
      Row4P = "";
      Row4ST = "";
    }
    if (!FOURran)
    {
      Row5W = "";
      Row5P = "";
      Row5ST = "";
    }
    if (!FIVEran)
    {
      Row6W = "";
      Row6P = "";
      Row6ST = "";
    }
  }
}
void ScreenProfile() // Display the lists and move them around
{
  Topbar();
  Backbutton(); // create a back button; on top left
  if (ProfileCRmode == 0)
  {
    if (ProfileSL)
    {
      if (ProfileRw == 1)
      {
        if (Sdvalid)
        {
          ProfileCRmode = 1;
        }
        else
        {
          ProfileSL = false;
        }
      }
      else if (ProfileRw == 2)
      {
        CurName = Row2N;
        CurSex = Row2S;
        CurAge = Row2A;
        RecYear = Row2Y;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (ProfileRw == 3)
      {
        CurName = Row3N;
        CurSex = Row3S;
        CurAge = Row3A;
        RecYear = Row3Y;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (ProfileRw == 4)
      {
        CurName = Row4N;
        CurSex = Row4S;
        CurAge = Row4A;
        RecYear = Row4Y;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (ProfileRw == 5)
      {
        CurName = Row5N;
        CurSex = Row5S;
        CurAge = Row5A;
        RecYear = Row5Y;
        if (CurName == "")
          ProfileSL = false;
      }
    }
    else
    {
      File PF = SD.open("/IC3-Results/Profiles");
      sprite.fillRect(8, 27, 250, 138, TFT_BLUE);               // BG for data table
      sprite.fillRect(8, 27 + 23, 250, 138 - 23, TFT_DARKCYAN); // BG for data table
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("NAME", 50, 38, 2);
      sprite.drawString("AGE", 128, 38, 2);
      sprite.drawString("SEX", 218, 38, 2);
      sprite.drawString(RecYear, 160, 8, 2);
      if (ProfileRw > 5) // shift data rows down
      {
        ProfileRw = 5;
        InternalRw++;
        // Move data
      }
      if (ProfileRw < 1) // shift data rows up
      {
        ProfileRw = 1;
        if (InternalRw > 0)
        {
          InternalRw--;
        }
      }
      Profilelist(PF);
      if (Profileamount == 0)
      {
        Row2N = "";
        Row2S = "";
        Row2A = "";
      }
      if (ProfileRw == 2)
      {
        sprite.fillRect(8, 27 + 46, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row2N, 50, 38 + 46, 2);
      sprite.drawString(Row2A, 128, 38 + 46, 2);
      sprite.drawString(Row2S, 218, 38 + 46, 2);
      if (ProfileRw == 3)
      {
        sprite.fillRect(8, 27 + 69, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row3N, 50, 38 + 69, 2);
      sprite.drawString(Row3A, 128, 38 + 69, 2);
      sprite.drawString(Row3S, 218, 38 + 69, 2);
      if (ProfileRw == 4)
      {
        sprite.fillRect(8, 27 + 92, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row4N, 50, 38 + 92, 2);
      sprite.drawString(Row4A, 128, 38 + 92, 2);
      sprite.drawString(Row4S, 218, 38 + 92, 2);
      if (ProfileRw == 5)
      {
        sprite.fillRect(8, 27 + 115, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row5N, 50, 38 + 115, 2);
      sprite.drawString(Row5A, 128, 38 + 115, 2);
      sprite.drawString(Row5S, 218, 38 + 115, 2);
      // display the pf amount here
      if (1)
      {                                                        // draw borders
        sprite.drawLine(8, 27 + 138, 258, 165, TFT_WHITE);     // X axis bottom?
        sprite.drawLine(8, 27, 258, 27, TFT_WHITE);            // X axis top?
        sprite.drawLine(8 + 80, 27, 8 + 80, 165, TFT_WHITE);   // Y axis line 1
        sprite.drawLine(8 + 160, 27, 8 + 160, 165, TFT_WHITE); // Y axis line 2
      }
      if (ProfileRw == 1)
      {
        sprite.setTextColor(TFT_BLACK);
        sprite.fillRect(8, 27 + 23, 250, 23, TFT_CYAN);
      }
      else
      {
        sprite.fillRect(8, 27 + 23, 250, 23, TFT_DARKCYAN);
      }

      if (Sdvalid)
      {
        sprite.drawString("CREATE NEW PROFILE", 120, 64, 2);
      }
      else
      {
        sprite.drawString("NO SD-CARD DETECTED", 120, 64, 2);
      }
      for (int i = 0; i < 6; i++)
      {
        sprite.drawLine(8, 27 + (i * 23), 258, 27 + (i * 23), TFT_WHITE); // graph X line
      }
      sprite.drawLine(8, 27, 8, 165, TFT_WHITE);             // X axis left most
      sprite.drawLine(8 + 250, 27, 8 + 250, 165, TFT_WHITE); // X axis right most
      sprite.fillSmoothRoundRect(5 + 60, 0, 100, 16, 4, TFT_DARKGREY, TFT_BLACK);
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString(String(Profileamount) + " : Profiles", 60 + 60, 8, 2);
      // Serial.println("Profileamount = " + String(Profileamount));
    }
  }
  else if (ProfileCRmode == 1) // name
  {
  }
  else if (ProfileCRmode == 2) // BD
  {
    if (YearMonthDate != 3)
    {
      sprite.fillRoundRect(190, 30 + 20, 120, 98 - 20, 4, TFT_DARKGREY);
      sprite.fillRoundRect(190, 135, 120, 25, 4, TFT_DARKCYAN); // CF button
      sprite.fillRoundRect(20 - 10, 30, 80, 130, 4, TFT_DARKGREY);
      sprite.fillRoundRect(110 - 10, 30, 80, 130, 4, TFT_DARKGREY);
      sprite.fillRoundRect(27 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
      sprite.fillRoundRect(117 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
      sprite.drawWideLine(35 - 10, 70, 92 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(117 + 8 - 10, 70, 182 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(35 - 10, 120, 92 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(117 + 8 - 10, 120, 182 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.setTextSize(2);
      sprite.drawString(String(Tens), 60 - 11, 89);
      sprite.drawString(String(Ones), 150 - 11, 89);
      sprite.setTextSize(1);
      if (YearMonthDate == 1) // Day
      {
        NewDate = (Tens * 10) + Ones;
        sprite.fillRoundRect(200, 62, 100, 25, 8, TFT_DARKCYAN);
      }
      else if (YearMonthDate == 2) // Month
      {
        NewMonth = (Tens * 10) + Ones;
        sprite.fillRoundRect(200, 105 - 13, 100, 25, 8, TFT_DARKCYAN);
      }
      sprite.drawString("BIRTH-DATE", 252, 75, 2);
      sprite.drawString("BIRTH-MONTH", 252, 105, 2);
    }
    else if (YearMonthDate == 3)
    {
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("INPUT BIRTH YEAR", 160, 8, 2);
      sprite.setTextColor(TFT_BLACK);
    }
  }
  else if (ProfileCRmode == 3) // Age and Sex
  {
    sprite.fillRoundRect(190, 30, 120, 98, 4, TFT_DARKGREY);
    sprite.fillRoundRect(190, 135, 120, 25, 4, TFT_DARKCYAN); // CF button
    sprite.fillRoundRect(20 - 10, 30, 80, 130, 4, TFT_DARKGREY);
    sprite.fillRoundRect(110 - 10, 30, 80, 130, 4, TFT_DARKGREY);
    sprite.fillRoundRect(27 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
    sprite.fillRoundRect(117 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
    sprite.drawWideLine(35 - 10, 70, 92 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(117 + 8 - 10, 70, 182 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(35 - 10, 120, 92 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(117 + 8 - 10, 120, 182 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.setTextSize(2);
    sprite.drawString(String(Tens), 60 - 11, 89);
    sprite.drawString(String(Ones), 150 - 11, 89);
    sprite.setTextSize(1);

    sprite.drawString("Select Gender", 254, 42, 2);
    sprite.drawString("MALE", 225, 115, 2);
    sprite.drawString("FEMALE", 275, 115, 2);
    NewAge = (Tens * 10) + Ones;
    if (NewAge == 0)
    {
      sprite.drawString("INVALID AGE", 190 + 60, 149, 2);
    }
    else if (NewGenderStr == "")
    {
      sprite.drawString("INPUT GENDER", 190 + 60, 149, 2);
    }
  }
  else if (ProfileCRmode == 4) // Last profile Page
  {
    sprite.setTextSize(1);
    sprite.setTextDatum(4);
    sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
    sprite.setTextColor(TFT_BLACK);
    if (!FinaliseInput) // create new PF
    {
      sprite.drawString("Profile Selected", 160, 35);
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("Name : " + NewNameStr, 160, 65);
      sprite.drawString("Sex : " + NewGenderStr, 160, 90);
      sprite.drawString("Age : " + String(NewAge), 160, 115);
      sprite.drawString(NewDMY, 160, 8, 2);
    }
    else
    {
      taskVibrate.enable();
      sprite.drawString("....SAVING....", 160, 35);
      delay(250);
      PFdump = SD.open("/IC3-Results/Profiles/" + NewNameStr + ".txt", FILE_WRITE);
      PFdump.print(NewNameStr + "," + String(NewAge) + "," + NewGenderStr + "," + NewDMY);
      // Serial.println(NewNameStr + "," + String(NewAge) + "," + NewGenderStr);
      PFdump.close();
      ProfileCRmode = 0;
      Tens = 0;
      Ones = 0;
      YearMonthDate = 1;
      FinaliseInput = false;
      ProfileSL = false;
      CurName = NewNameStr;
      CurSex = NewGenderStr;
      CurAge = String(NewAge);
      FolderCreate();
      NewNameStr = "";
      NewGenderStr = "";
      NewAge = 0;
      ButtonTouched = true;
      // end and reset valls
    }
  }
  else if (ProfileCRmode == 5) //  Edit
  {
    DataEdit();
  }
}

void ScreenRecords()
{
  Topbar();
  Backbutton(); // create a back button; on top left
  if (RecordsPage == 0)
  {
    if (ProfileSL)
    {
      if (RecRow == 1)
      {
        RecName = Row2N;
        RecSex = Row2S;
        RecAge = Row2A;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (RecRow == 2)
      {
        RecName = Row3N;
        RecSex = Row3S;
        RecAge = Row3A;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (RecRow == 3)
      {
        RecName = Row4N;
        RecSex = Row4S;
        RecAge = Row4A;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (RecRow == 4)
      {
        RecName = Row5N;
        RecSex = Row5S;
        RecAge = Row5A;
        if (CurName == "")
          ProfileSL = false;
      }
      else if (RecRow == 5)
      {
        RecName = Row6N;
        RecSex = Row6S;
        RecAge = Row6A;
        if (CurName == "")
          ProfileSL = false;
      }
      if (RecName != "")
      { // if data not empty go to next page
        RecordsPage++;
        ProfileSL = false;
        ModeRw = 0;
      }
    }
    else
    {
      File PF = SD.open("/IC3-Results/Profiles");
      sprite.fillRect(8, 27, 250, 138, TFT_BLUE);               // BG for data table
      sprite.fillRect(8, 27 + 23, 250, 138 - 23, TFT_DARKCYAN); // BG for data table
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("NAME", 50, 38, 2);
      sprite.drawString("AGE", 128, 38, 2);
      sprite.drawString("SEX", 218, 38, 2);
      if (RecRow > 5) // shift data rows down
      {
        RecRow = 5;
        InternalRw++;
        // Move data
      }
      if (RecRow < 1) // shift data rows up
      {
        RecRow = 1;
        if (InternalRw > 0)
        {
          InternalRw--;
        }
      }
      Profilelist(PF);
      PF.close();
      if (Profileamount == 0)
      {
        Row2N = "";
        Row2S = "";
        Row2A = "";
      }
      sprite.setTextColor(TFT_BLACK);
    }
    if (RecRow == 1)
    {
      sprite.fillRect(8, 27 + 23, 250, 23, TFT_CYAN);
    }
    sprite.drawString(Row2N, 50, 64, 2);
    sprite.drawString(Row2A, 128, 64, 2);
    sprite.drawString(Row2S, 218, 64, 2);
    if (RecRow == 2)
    {
      sprite.fillRect(8, 27 + 46, 250, 23, TFT_CYAN);
    }
    sprite.drawString(Row3N, 50, 38 + 46, 2);
    sprite.drawString(Row3A, 128, 38 + 46, 2);
    sprite.drawString(Row3S, 218, 38 + 46, 2);
    if (RecRow == 3)
    {
      sprite.fillRect(8, 27 + 69, 250, 23, TFT_CYAN);
    }
    sprite.drawString(Row4N, 50, 38 + 69, 2);
    sprite.drawString(Row4A, 128, 38 + 69, 2);
    sprite.drawString(Row4S, 218, 38 + 69, 2);
    if (RecRow == 4)
    {
      sprite.fillRect(8, 27 + 92, 250, 23, TFT_CYAN);
    }
    sprite.drawString(Row5N, 50, 38 + 92, 2);
    sprite.drawString(Row5A, 128, 38 + 92, 2);
    sprite.drawString(Row5S, 218, 38 + 92, 2);
    if (RecRow == 5)
    {
      sprite.fillRect(8, 27 + 115, 250, 23, TFT_CYAN);
    }
    sprite.drawString(Row6N, 50, 38 + 115, 2);
    sprite.drawString(Row6A, 128, 38 + 115, 2);
    sprite.drawString(Row6S, 218, 38 + 115, 2);
    // display the pf amount here
    if (1)
    {                                                        // draw borders
      sprite.drawLine(8, 27 + 138, 258, 165, TFT_WHITE);     // X axis bottom?
      sprite.drawLine(8, 27, 258, 27, TFT_WHITE);            // X axis top?
      sprite.drawLine(8 + 80, 27, 8 + 80, 165, TFT_WHITE);   // Y axis line 1
      sprite.drawLine(8 + 160, 27, 8 + 160, 165, TFT_WHITE); // Y axis line 2
    }
    // sprite.drawString("Create New Profile", 120, 64, 2);
    for (int i = 0; i < 6; i++)
    {
      sprite.drawLine(8, 27 + (i * 23), 258, 27 + (i * 23), TFT_WHITE); // graph X line
    }
    sprite.drawLine(8, 27, 8, 165, TFT_WHITE);             // X axis left most
    sprite.drawLine(8 + 250, 27, 8 + 250, 165, TFT_WHITE); // X axis right most
    sprite.fillSmoothRoundRect(5 + 60, 0, 100, 16, 4, TFT_DARKGREY, TFT_BLACK);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString(String(Profileamount) + " : Profiles", 60 + 60, 8, 2);
  }
  else if (RecordsPage == 1)
  {
    sprite.fillRect(8, 27, 250, 138 - 46, TFT_BLUE);               // BG for data table
    sprite.fillRect(8, 27 + 23, 250, 138 - 23 - 46, TFT_DARKCYAN); // BG for data table
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("Select Record Mode", 120, 38, 2);
    if (ModeRw > 3) // shift data rows down
    {
      ModeRw = 3;
      // Move data
    }
    if (ModeRw < 1) // shift data rows up
    {
      ModeRw = 1;
    }
    sprite.setTextColor(TFT_BLACK);
    if (ModeRw == 1)
    {
      sprite.fillRect(8, 27 + 23, 250, 23, TFT_CYAN);
    }
    sprite.drawString("SELECT MODE ECG", 120 - 51, 64, 2);
    if (ModeRw == 2)
    {
      sprite.fillRect(8, 27 + 46, 250, 23, TFT_CYAN);
    }
    sprite.drawString("SELECT MODE SPO2", 120 - 46, 38 + 47, 2);
    if (ModeRw == 3)
    {
      sprite.fillRect(8, 27 + 69, 250, 23, TFT_CYAN);
    }
    sprite.drawString("SELECT MODE MULTI", 120 - 44, 38 + 70, 2);
    // display the pf amount here
    if (1)
    {                                                        // draw borders
      sprite.drawLine(8, 27 + 92, 258, 165 - 46, TFT_WHITE); // X axis bottom?
      sprite.drawLine(8, 27, 258, 27, TFT_WHITE);            // X axis top?
    }
    // sprite.drawString("Create New Profile", 120, 64, 2);
    for (int i = 0; i < 4; i++)
    {
      sprite.drawLine(8, 27 + (i * 23), 258, 27 + (i * 23), TFT_WHITE); // graph X line
    }
    sprite.drawLine(8, 27, 8, 165 - 46, TFT_WHITE);             // X axis left most
    sprite.drawLine(8 + 250, 27, 8 + 250, 165 - 46, TFT_WHITE); // X axis right most
  }
  else if (RecordsPage == 2)
  {
    if (ProfileSL)
    {
      OUTName = ""; // reset Curname
      if (RecRow == 1)
      {
        OUTName = Row2N; // Name
        RecSex = Row2S;  // is size in this case
        if (OUTName == "")
          ProfileSL = false;
      }
      else if (RecRow == 2)
      {
        OUTName = Row3N; // Name
        RecSex = Row3S;  // is size in this case
        if (OUTName == "")
          ProfileSL = false;
      }
      else if (RecRow == 3)
      {
        OUTName = Row4N; // Name
        RecSex = Row4S;  // is size in this case
        if (OUTName == "")
          ProfileSL = false;
      }
      else if (RecRow == 4)
      {
        OUTName = Row5N; // Name
        RecSex = Row5S;  // is size in this case
        if (OUTName == "")
          ProfileSL = false;
      }
      else if (RecRow == 5)
      {
        OUTName = Row6N; // Name
        RecSex = Row6S;  // is size in this case
        if (OUTName == "")
          ProfileSL = false;
      }
      if (OUTName != "")
      { 
        RecordsPage++;
      }
      Serial.println("CURNAME =" + OUTName);
    }
    else
    {
      String Path;
      if (FileAmount == 0)
      {
        Row2N = "";
        Row2S = "";
        Row2A = "";
        Row2Y = "";
      }
      if (ModeRw == 1)
      {
        Path = "/IC3-Results/" + RecName + "/Mode1ECG";
      }
      else if (ModeRw == 2)
      {
        Path = "/IC3-Results/" + RecName + "/Mode2HR";
      }
      else if (ModeRw == 3)
      {
        Path = "/IC3-Results/" + RecName + "/Mode3Multi";
      }
      sprite.fillRect(8, 27, 250, 138, TFT_BLUE);               // BG for data table
      sprite.fillRect(8, 27 + 23, 250, 138 - 23, TFT_DARKCYAN); // BG for data table
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("FILE NAME", 85, 38, 2);
      sprite.drawString("TIME (Mins)", 210, 38, 2);
      if (RecRow > 5) // shift data rows down
      {
        RecRow = 5;
        InternalRRw++; // Move data
      }
      if (RecRow < 1) // shift data rows up
      {
        RecRow = 1;
        if (InternalRRw > 0)
        {
          InternalRRw--;
        }
      }
      // Serial.println("Path = " + Path);
      File PF = SD.open(Path.c_str());
      printDirectory(PF, Path);
      PF.close();
      if (RecRow == 1)
      {
        sprite.fillRect(8, 27 + 23, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row2N, 145 - (Row2A.toInt() * 5), 64, 2);
      sprite.drawString(Row2T, 218, 64, 2);
      if (RecRow == 2)
      {
        sprite.fillRect(8, 27 + 46, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row3N, 145 - (Row3A.toInt() * 5), 38 + 46, 2);
      sprite.drawString(Row3T, 218, 38 + 46, 2);
      if (RecRow == 3)
      {
        sprite.fillRect(8, 27 + 69, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row4N, 145 - (Row4A.toInt() * 5), 38 + 69, 2);
      sprite.drawString(Row4T, 218, 38 + 69, 2);
      if (RecRow == 4)
      {
        sprite.fillRect(8, 27 + 92, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row5N, 145 - (Row5A.toInt() * 5), 38 + 92, 2);
      sprite.drawString(Row5T, 218, 38 + 92, 2);
      if (RecRow == 5)
      {
        sprite.fillRect(8, 27 + 115, 250, 23, TFT_CYAN);
      }
      sprite.drawString(Row6N, 145 - (Row6A.toInt() * 5), 38 + 115, 2);
      sprite.drawString(Row6T, 218, 38 + 115, 2);
      if (1)
      {                                                    // draw borders
        sprite.drawLine(8, 27 + 138, 258, 165, TFT_WHITE); // X axis bottom?
        sprite.drawLine(8, 27, 258, 27, TFT_WHITE);        // X axis top?
        // sprite.drawLine(8 + 80, 27, 8 + 80, 165, TFT_WHITE);   // Y axis line 1
        sprite.drawLine(8 + 160 - 10, 27, 8 + 160 - 10, 165, TFT_WHITE); // Y axis line 2
      }
      // sprite.drawString("Create New Profile", 120, 64, 2);
      for (int i = 0; i < 6; i++)
      {
        sprite.drawLine(8, 27 + (i * 23), 258, 27 + (i * 23), TFT_WHITE); // graph X line
      }
      sprite.drawLine(8, 27, 8, 165, TFT_WHITE);             // X axis left most
      sprite.drawLine(8 + 250, 27, 8 + 250, 165, TFT_WHITE); // X axis right most
    }
  }
  else if (RecordsPage == 3)
  {
    if (!RecServer)
    {
      if (WifiEmpty)
      {
      }
      else
      {
        sprite.setTextSize(1);
        sprite.setTextDatum(4);
        sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("FILE SELECTED", 160, 35);
        sprite.setTextColor(TFT_WHITE);
        sprite.drawString(OUTName, 160, 65 + 10);
        sprite.drawString("Size : " + RecSex, 160, 90 + 10);
        if (CurWifi != "")
        {
          sprite.drawString("WIFI : " + CurWifi, 160, 8, 2);
        }
        else
        {
          sprite.drawString("WIFI : NOT SELECTED", 160, 8, 2);
        }
        if (RecDelConfirm)
        {
          String FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + OUTName;
          const char *DelChar = FilePath.c_str();
          removeFile(SD, DelChar);
          RecordsPage = 2;
          ButtonTouched = true;
          RecDelConfirm = false;
        }
      }
    }
    else if (RecServer)
    {
      sprite.setTextSize(1);
      sprite.setTextDatum(4);
      sprite.fillRoundRect(160 - 155, 24 + 45, 310, 30, 4, TFT_LIGHTGREY);
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("WIFI : " + CurWifi, 160, 10, 2);
      sprite.setTextColor(TFT_BLACK);
      String FilePath = "/IC3-Results/" + RecName + "/Mode1ECG/" + OUTName;
      if (CurWifi == "")
      {
        WifiEmpty = true;
      }
      else
      {
        if (SizeChk(FilePath))
        {
          sprite.drawString("SENDING TO SERVER", 160, 35 + 45);
          sprite.pushSprite(0, 0);
          sprite.setTextColor(TFT_WHITE);
          sprite.drawString("WIFI : " + CurWifi, 160, 10, 2);
          delay(100);
          SendtoServer(FilePath);
          LatestTouch = millis();
        }
        else
        {
          sprite.drawString("FILE IS TOO SHORT", 160, 35 + 45);
          sprite.setTextColor(TFT_RED);
          sprite.drawString("MORE THAN 30 SECONDS IS NEEDED", 160, 35 + 90, 2);
          sprite.pushSprite(0, 0);
          delay(1000);
        }
      }
      sprite.setTextColor(TFT_BLACK);
      RecServer = false;
      // Serial.println(FilePath);
    }
  }
}
void RecordTouch()
{
  if ((RecordsPage == 0) || (RecordsPage == 1)) // On records page 0,1
  {
    sprite.fillRoundRect(265, 27, 50, 40, 2, TFT_DARKGREY);
    sprite.fillRoundRect(265, 71, 50, 50, 2, TFT_DARKGREY);
    sprite.fillRoundRect(265, 126, 50, 40, 2, TFT_DARKGREY);
    if (!ProfileDWHold)
    {
      if ((XCoord <= 70) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileDWHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (RecordsPage == 0)
        {
          RecRow--;
        }
        else if (RecordsPage == 1)
        {
          ModeRw--;
        }
        ButtonTouched = true;
        // Serial.println("ProfileRW =" + String(ProfileRw));
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        ProfileDWHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        ProfileDWHold = false;
      }
    }
    if (!ProfileUPHold)
    {
      if ((XCoord <= 170) && (XCoord >= 125) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileUPHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (RecordsPage == 0)
        {
          RecRow++;
        }
        else if (RecordsPage == 1)
        {
          ModeRw++;
        }
        ButtonTouched = true;
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        ProfileUPHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        ProfileUPHold = false;
      }
    }
    if (!ProfileSLHold) // Select
    {
      if ((XCoord <= 120) && (XCoord >= 70) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileSLHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (RecordsPage == 0)
        {
          ProfileSL = true;
        }
        else if (RecordsPage == 1)
        {
          RecordsPage++;
          PFfirstrun = true;
        }
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        ProfileSLHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        ProfileSLHold = false;
      }
    }
    sprite.fillTriangle(280, 36 + 20, 290, 36, 300, 36 + 20, TFT_CYAN);
    sprite.fillTriangle(280, 155 - 20, 290, 155, 300, 155 - 20, TFT_CYAN);
    sprite.fillCircle(289, 95, 15, TFT_LIGHTGREY);
  }
  else if (RecordsPage == 2)
  {
    sprite.fillRoundRect(265, 27, 50, 40, 2, TFT_DARKGREY);
    sprite.fillRoundRect(265, 71, 50, 50, 2, TFT_DARKGREY);
    sprite.fillRoundRect(265, 126, 50, 40, 2, TFT_DARKGREY);
    if (!ProfileDWHold)
    {
      if ((XCoord <= 70) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileDWHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        RecRow--;
        ButtonTouched = true;
        // Serial.println("ProfileRW =" + String(ProfileRw));
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        ProfileDWHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        ProfileDWHold = false;
      }
    }
    if (!ProfileUPHold)
    {
      if ((XCoord <= 170) && (XCoord >= 125) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileUPHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        RecRow++;
        ButtonTouched = true;
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        ProfileUPHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        ProfileUPHold = false;
      }
    }
    if (!ProfileSLHold) // Select
    {
      if ((XCoord <= 120) && (XCoord >= 70) && (YCoord <= 320) && (YCoord >= 250))
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileSLHold = true;
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        ProfileSL = true;
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        ProfileSLHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        ProfileSLHold = false;
      }
    }
    sprite.fillTriangle(280, 36 + 20, 290, 36, 300, 36 + 20, TFT_CYAN);
    sprite.fillTriangle(280, 155 - 20, 290, 155, 300, 155 - 20, TFT_CYAN);
    sprite.fillCircle(289, 95, 15, TFT_LIGHTGREY);
  }
  else if (RecordsPage == 3)
  {
    if (!RecDel)
    {
      if (WifiEmpty)
      {
        sprite.fillRoundRect(32, 40, 255, 40, 4, TFT_LIGHTGREY);
        sprite.fillRoundRect(38, 85, 245, 40, 4, TFT_LIGHTGREY);
        sprite.setTextColor(TFT_BLACK);
        sprite.drawString("PLEASE SELECT", 160, 57);
        sprite.drawString("VALID WIFI", 158, 102);
        if (!Noprofilehold) // continue button
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 200) && (YCoord >= 100))
          {
            XCoordRS = XCoord;
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_DARKCYAN);
            Noprofilehold = true;
          }
          else
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            WifiSettingFR = false;
            WFfirstrun = true;
            Wifisettings = true;
            SettingsPage = 2;
            WifiEmpty = false;
            RecordsPage = 0;
            Profileset = true;
            StartRec = false;
            WifiPage = 0;
            Noprofilehold = false;
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
          }
          else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_DARKCYAN);
          }
          else
          {
            sprite.fillRoundRect(120, 140, 80, 25, 4, TFT_CYAN);
            Noprofilehold = false;
          }
        }
        sprite.drawString("CONTINUE", 160, 150, 2);
      }
      else
      {
        if (!ProfileUPHold) // delete
        {
          if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 130) && (YCoord >= 40))
          {
            sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_ORANGE);
            XCoordLS = XCoord;
            ProfileUPHold = true;
          }
          else
          {
            sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_RED);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            RecDel = true;
            sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_RED);
            ProfileUPHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_ORANGE);
          }
          else
          {
            sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_RED);
            ProfileUPHold = false;
          }
        }
        if (!ProfileSLHold) // Server
        {
          if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 300) && (YCoord >= 160))
          {
            sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_GREEN);
            XCoordLS = XCoord;
            ProfileSLHold = true;
          }
          else
          {
            sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_DARKGREEN);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            RecServer = true;
            sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_DARKGREEN);
            ProfileSLHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_GREEN);
          }
          else
          {
            sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_DARKGREEN);
            ProfileSLHold = false;
          }
        }
        sprite.drawString("DELETE", 90, 140 + 5, 2);
        sprite.drawString("SEND SERVER", 230, 140 + 5, 2);
      }
    }
    else if (RecDel)
    {
      if (!ProfileUPHold) // delete
      {
        if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 130) && (YCoord >= 40))
        {
          sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_GREEN);
          XCoordLS = XCoord;
          ProfileUPHold = true;
        }
        else
        {
          sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_DARKGREEN);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          RecDel = false;
          sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_DARKGREEN);
          ProfileUPHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_GREEN);
        }
        else
        {
          sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_DARKGREEN);
          ProfileUPHold = false;
        }
      }
      if (!ProfileSLHold) // Server
      {
        if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 300) && (YCoord >= 160))
        {
          sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_ORANGE);
          XCoordLS = XCoord;
          ProfileSLHold = true;
        }
        else
        {
          sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_RED);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          RecDelConfirm = true;
          sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_RED);
          ProfileSLHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_ORANGE);
        }
        else
        {
          sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_RED);
          ProfileSLHold = false;
        }
      }
      sprite.drawString("CANCEL", 90, 140 + 5, 2);
      sprite.drawString("CONFIRM", 230, 140 + 5, 2);
    }
  }
}
void ScreenSaver() // dim or shut down screen after sometime
{
  if ((millis() - LatestTouch) > ScreenOff * 1000)
  { // sc off
    ledcWrite(0, 0);
    SCSaverOn = true;
  }
  else if ((5000 + millis() - LatestTouch) > ScreenOff * 1000)
  { // dim
    ledcWrite(0, 50);
  }
  else
  {
    ledcWrite(0, SCBright);
    delay(10);
    SCSaverOn = false;
  }
}
void DataEdit() // edit SD
{
  if (editPage == 1) // If setting name
  {
    // For Left Arrow
    if (!SCRHoldL)
    {
      if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 60) && (YCoord >= 0))
      {
        sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
        XCoordLS = XCoord;
        SCRHoldL = true;
      }
      else
      {
        sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_DARKGREY);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
        if (PageAlpha > 1) // Loop 1-4 temp untill touch buttons
        {
          PageAlpha--;
        }
        else
        {
          PageAlpha = 7;
        }
        SCRHoldL = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
      }
      else
      {
        sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_WHITE);
        SCRHoldL = false;
      }
    }
    // For Right Arrow
    if (!SCRHoldR)
    {
      if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 270))
      {
        XCoordRS = XCoord;
        sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
        SCRHoldR = true;
      }
      else
      {
        sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
        if (PageAlpha <= 6) // Loop 1-4 temp untill touch buttons
        {
          PageAlpha++;
        }
        else
        {
          PageAlpha = 1;
        }
        SCRHoldR = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
      }
      else
      {
        sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
        sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
        SCRHoldR = false;
      }
    }
    sprite.fillRoundRect(40 - 17, 100, 65, 38, 4, TFT_DARKGREY);
    sprite.fillRoundRect(110 - 17, 100, 65, 38, 4, TFT_DARKGREY);
    sprite.fillRoundRect(180 - 17, 100, 65, 38, 4, TFT_DARKGREY);
    sprite.fillRoundRect(250 - 17, 100, 65, 38, 4, TFT_DARKGREY);
    if (!FAlphaHold) // first alphabet
    {
      if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 70) && (YCoord >= 0))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
        FAlphaHold = true;
      }
      else
      {
        sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {

        if (EditNameLenth <= 9)
          EditName += FDigit;
        sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
        FAlphaHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
      }
      else
      {
        sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1

        FAlphaHold = false;
      }
    }
    if (!SAlphaHold) // Second alphabet
    {
      if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 160) && (YCoord >= 80))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
        SAlphaHold = true;
      }
      else
      {
        sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditNameLenth <= 9)
          EditName += SDigit;
        sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
        SAlphaHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
      }
      else
      {
        sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
        SAlphaHold = false;
      }
    }
    if (!TAlphaHold) // Third alphabet
    {
      if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 220) && (YCoord >= 170))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
        TAlphaHold = true;
      }
      else
      {
        sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditNameLenth <= 9)
          EditName += TDigit;
        sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
        TAlphaHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
      }
      else
      {
        sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3

        TAlphaHold = false;
      }
    }
    if (!FRAlphaHold) // Forth alphabet
    {
      if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 290) && (YCoord >= 230))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
        FRAlphaHold = true;
      }
      else
      {
        sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {

        if (EditNameLenth <= 9)
          EditName += FrDigit;
        sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
        FRAlphaHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
      }
      else
      {
        sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4

        FRAlphaHold = false;
      }
    }
    if (!BackSpcHold) // BackSpace
    {
      if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 140) && (YCoord >= 20))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
        BackSpcHold = true;
      }
      else
      {
        sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        EditNameLenth = EditName.length();
        String EditNameMinus;
        for (i = 1; i < EditNameLenth; i++)
        {
          EditNameMinus += EditName[i - 1];
        }
        EditName = EditNameMinus;
        sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
        BackSpcHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
      }
      else
      {
        sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
        BackSpcHold = false;
      }
    }
    if (!NextHold) // Next button
    {
      if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 300) && (YCoord >= 160))
      {
        XCoordRS = XCoord;
        sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
        NextHold = true;
      }
      else
      {
        sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        EditNameLenth = EditName.length();
        String AdrString = "/IC3-Results/Profiles/" + EditName + ".txt";
        const char *AdrChar = AdrString.c_str();
        String SomeData = readFile(SD, AdrChar);
        if (EditName == CurName)
        { // Same name is ok
          editPage = 2;
        }
        else if (EditNameLenth == 0)
        {
          for (int time = 0; time < 20; time++)
          {
            sprite.setTextColor(TFT_RED);
            sprite.drawString("Not a valid name", 160, 35, 2);
            delay(15);
          }
        }
        else if (FileFound) // duplicate name with other file
        {
          for (int time = 0; time < 20; time++)
          {
            sprite.setTextColor(TFT_RED);
            sprite.drawString("Duplicate name", 160, 35, 2);
            delay(15);
          }
        }
        else
        {
          editPage = 2;
        }
        // change to page 2
        sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
        NextHold = false;
      }
      else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
      }
      else
      {
        sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
        NextHold = false;
      }
    }
    sprite.setTextColor(TFT_BLACK);
    sprite.fillRoundRect(45, 42, 230, 50, 8, TFT_WHITE); // BG NAME BOX
    EditNameLenth = EditName.length();
    sprite.drawString(EditName, 60 + (EditNameLenth * 10), 60);
    sprite.drawString(String(FDigit), 30 + 24, 107 + 8);
    sprite.drawString(String(SDigit), 100 + 24, 107 + 8);
    sprite.drawString(String(TDigit), 170 + 24, 107 + 8);
    sprite.drawString(String(FrDigit), 240 + 24, 107 + 8);
    sprite.drawString("DELETE", 90, 150);
    sprite.drawString("CONFIRM", 230, 155, 2);
    ProfileCreate();
  }
  else if (editPage == 2)
  {
    if (YearMonthDate != 3)
    {
      sprite.fillRoundRect(190, 30 + 20, 120, 98 - 20, 4, TFT_DARKGREY);
      sprite.fillRoundRect(190, 135, 120, 25, 4, TFT_DARKCYAN); // CF button
      sprite.fillRoundRect(20 - 10, 30, 80, 130, 4, TFT_DARKGREY);
      sprite.fillRoundRect(110 - 10, 30, 80, 130, 4, TFT_DARKGREY);
      sprite.fillRoundRect(27 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
      sprite.fillRoundRect(117 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
      sprite.drawWideLine(35 - 10, 70, 92 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(117 + 8 - 10, 70, 182 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(35 - 10, 120, 92 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.drawWideLine(117 + 8 - 10, 120, 182 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
      sprite.setTextSize(2);
      sprite.drawString(String(Tens), 60 - 11, 89);
      sprite.drawString(String(Ones), 150 - 11, 89);
      sprite.setTextSize(1);
      if (YearMonthDate == 1) // Day
      {
        NewDate = (Tens * 10) + Ones;
        sprite.fillRoundRect(200, 62, 100, 25, 8, TFT_DARKCYAN);
      }
      else if (YearMonthDate == 2) // Month
      {
        NewMonth = (Tens * 10) + Ones;
        sprite.fillRoundRect(200, 105 - 13, 100, 25, 8, TFT_DARKCYAN);
      }
      sprite.drawString("BIRTH-DATE", 252, 75, 2);
      sprite.drawString("BIRTH-MONTH", 252, 105, 2);
    }
    else if (YearMonthDate == 3)
    {
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("INPUT BIRTH YEAR", 160, 8, 2);
      sprite.setTextColor(TFT_BLACK);
    }
    if (YearMonthDate != 3)
    {
      if (!TenUpHold)
      {
        if ((XCoord <= 60) && (XCoord >= 50) && (YCoord <= 60) && (YCoord >= 0))
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          XCoordLS = XCoord;
          TenUpHold = true;
        }
        else
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_BLACK); // Ten up
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Tens <= 8)
          {
            Tens++;
          }
          else
          {
            Tens = 0;
          }
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          TenUpHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
        }
        else
        {
          sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
          TenUpHold = false;
        }
      }
      if (!TenDwHold)
      {
        if ((XCoord <= 165) && (XCoord >= 120) && (YCoord <= 70) && (YCoord >= 10))
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
          XCoordLS = XCoord;
          TenDwHold = true;
        }
        else
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Tens >= 1)
          {
            Tens--;
          }
          else
          {
            Tens = 9;
          }
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
          TenDwHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
        }
        else
        {
          sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
          TenDwHold = false;
        }
      }
      if (!OneUpHold)
      {
        if ((XCoord <= 70) && (XCoord >= 40) && (YCoord <= 165) && (YCoord >= 100))
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
          XCoordLS = XCoord;
          OneUpHold = true;
        }
        else
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Ones <= 8)
          {
            Ones++;
          }
          else
          {
            Ones = 0;
          }
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
          OneUpHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
        }
        else
        {
          sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
          OneUpHold = false;
        }
      }
      if (!OneDwHold)
      {
        if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 165) && (YCoord >= 100))
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
          XCoordLS = XCoord;
          OneDwHold = true;
        }
        else
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (Ones >= 1)
          {
            Ones--;
          }
          else
          {
            Ones = 9;
          }
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
          OneDwHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
        }
        else
        {
          sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
          OneDwHold = false;
        }
      }
      if (((NewDate <= 31) && (NewDate != 0)) && (YearMonthDate == 1))
      {
        sprite.drawString("NEXT", 190 + 60, 149, 2);
        if (!NextHold)
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
          {
            XCoordLS = XCoord;
            NextHold = true;
          }
          else
          {
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            YearMonthDate = 2;
            Ones = 0;
            Tens = 0;
            // Serial.println("Day =" + String(NewDate));
            NextHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
          }
          else
          {
            NextHold = false;
          }
        }
      }
      else if (YearMonthDate == 1)
      {
        sprite.drawString("INVALID DATE", 190 + 60, 149, 2);
      }
      if ((YearMonthDate == 2) && ((NewMonth <= 12) && (NewMonth != 0)))
      {
        sprite.drawString("NEXT", 190 + 60, 149, 2);
        if (!NextHold)
        {
          if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
          {
            XCoordLS = XCoord;
            NextHold = true;
          }
          else
          {
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            Ones = 0;
            Tens = 0;
            Serial.println("Month =" + String(NewMonth));
            NewYear = "";
            YearMonthDate = 3;
            // ProfileCRmode = 3;
            NextHold = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
          }
          else
          {
            NextHold = false;
          }
        }
      }
      else if (YearMonthDate == 2)
      {
        sprite.drawString("INVALID MONTH", 190 + 60, 149, 2);
      }
    }
    else // Set name
    {
      // For Left Arrow
      if (!SCRHoldL)
      {
        if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 60) && (YCoord >= 0))
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
          XCoordLS = XCoord;
          SCRHoldL = true;
        }
        else
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_DARKGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
          if (YearPg > 1)
          {
            YearPg--;
          }
          else
          {
            YearPg = 3;
          }
          SCRHoldL = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_BLACK);
        }
        else
        {
          sprite.fillTriangle(15, 85 - 20, 35, 65 - 20, 35, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(20, 85 - 20, 31, 74 - 20, 31, 96 - 20, TFT_WHITE);
          SCRHoldL = false;
        }
      }
      // For Right Arrow
      if (!SCRHoldR)
      {
        if ((XCoord <= 95) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 270))
        {
          XCoordRS = XCoord;
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
          SCRHoldR = true;
        }
        else
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
          if (YearPg <= 2)
          {
            YearPg++;
          }
          else
          {
            YearPg = 1;
          }
          SCRHoldR = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_BLACK);
        }
        else
        {
          sprite.fillTriangle(305, 85 - 20, 285, 65 - 20, 285, 105 - 20, TFT_LIGHTGREY);
          sprite.fillTriangle(300, 85 - 20, 289, 74 - 20, 289, 96 - 20, TFT_DARKGREY);
          SCRHoldR = false;
        }
      }
      sprite.fillRoundRect(40 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(110 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(180 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      sprite.fillRoundRect(250 - 17, 100, 65, 38, 4, TFT_DARKGREY);
      if (!FAlphaHold) // first alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 70) && (YCoord >= 0))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
          FAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {

          if (NameLenth <= 9)
            NewYear += String(Yearint);
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
          FAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b1
        }
        else
        {
          sprite.fillRoundRect(40 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b1
          FAlphaHold = false;
        }
      }
      if (!SAlphaHold) // Second alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 160) && (YCoord >= 80))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
          SAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if (NameLenth <= 9)
            NewYear += String(Yearint + 1);
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
          SAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b2
        }
        else
        {
          sprite.fillRoundRect(110 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b2
          SAlphaHold = false;
        }
      }
      if (!TAlphaHold) // Third alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 220) && (YCoord >= 170))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
          TAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {

          if (NameLenth <= 9)
          {
            if (YearPg != 3)
            {
              NewYear += String(Yearint + 2);
            }
            else
            {
              NewYear += String(0);
            }
          }
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3
          TAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b3
        }
        else
        {
          sprite.fillRoundRect(180 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b3

          TAlphaHold = false;
        }
      }
      if (!FRAlphaHold) // Forth alphabet
      {
        if ((XCoord <= 140) && (XCoord >= 100) && (YCoord <= 290) && (YCoord >= 230))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
          FRAlphaHold = true;
        }
        else
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {

          if (NameLenth <= 9)
          {
            if (YearPg != 3)
            {
              NewYear += String(Yearint + 3);
            }
            else
            {
              NewYear += String(1);
            }
          }
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4
          FRAlphaHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_BLACK); // b4
        }
        else
        {
          sprite.fillRoundRect(250 - 10, 100 + 7, 50, 25, 4, TFT_LIGHTGREY); // b4

          FRAlphaHold = false;
        }
      }
      if (!BackSpcHold) // BackSpace
      {
        if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 140) && (YCoord >= 20))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
          BackSpcHold = true;
        }
        else
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          NameLenth = NewYear.length();
          String NewYearMinus;
          for (i = 1; i < NameLenth; i++)
          {
            NewYearMinus += NewYear[i - 1];
          }
          NewYear = NewYearMinus;
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
          BackSpcHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_BLUE); // Delete button
        }
        else
        {
          sprite.fillRoundRect(40 - 16, 142, 134, 25, 8, TFT_DARKCYAN); // Delete button
          BackSpcHold = false;
        }
      }
      if (!NextHold) // Next button
      {
        if ((XCoord <= 170) && (XCoord >= 160) && (YCoord <= 300) && (YCoord >= 160))
        {
          XCoordRS = XCoord;
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
          NextHold = true;
        }
        else
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          if ((NewYear == "0") || (NewYear.toInt() > 2050))
          {
            sprite.setTextColor(TFT_RED);
            sprite.drawString("NOT A VALID YEAR", 160, 35, 2);
            delay(500);
          }
          else
          {
            NewDMY = String(NewDate) + "." + String(NewMonth) + "." + NewYear;
            YearPg = 1;
            NewYear = "";
            editPage = 3;
          }
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
          NextHold = false;
        }
        else if ((XCoord <= XCoordRS + 15) && (XCoord >= XCoordRS - 15)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_BLUE); // Confirm button
        }
        else
        {
          sprite.fillRoundRect(163, 142, 134, 25, 8, TFT_DARKCYAN); // Confirm button
          NextHold = false;
        }
      }
      sprite.setTextColor(TFT_BLACK);
      sprite.fillRoundRect(45, 42, 230, 50, 8, TFT_WHITE); // BG NAME BOX
      NameLenth = NewYear.length();
      sprite.drawString(String(NewYear), 60 + (NameLenth * 10), 60);
      if (YearPg == 1) // 0123
      {
        Yearint = 0;
      }
      else if (YearPg == 2) // 4567
      {
        Yearint = 4;
      }
      else if (YearPg == 3) // 8901
      {
        Yearint = 8;
      }
      sprite.drawString(String(Yearint), 30 + 24, 107 + 8);
      sprite.drawString(String(Yearint + 1), 100 + 24, 107 + 8);
      if (YearPg != 3)
      {
        sprite.drawString(String(Yearint + 2), 170 + 24, 107 + 8);
        sprite.drawString(String(Yearint + 3), 240 + 24, 107 + 8);
      }
      else
      {
        sprite.drawString(String("0"), 170 + 24, 107 + 8);
        sprite.drawString(String("1"), 240 + 24, 107 + 8);
      }
      sprite.drawString("DELETE", 90, 150);
      sprite.drawString("CONFIRM", 230, 155, 2);
      // ProfileCreate();
    }
  }
  else if (editPage == 3)
  {
    sprite.fillRoundRect(190, 30, 120, 98, 4, TFT_DARKGREY);
    sprite.fillRoundRect(190, 135, 120, 25, 4, TFT_DARKCYAN); // CF button
    sprite.fillRoundRect(20 - 10, 30, 80, 130, 4, TFT_DARKGREY);
    sprite.fillRoundRect(110 - 10, 30, 80, 130, 4, TFT_DARKGREY);
    sprite.fillRoundRect(27 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
    sprite.fillRoundRect(117 - 10, 38, 65, 115, 4, TFT_LIGHTGREY);
    sprite.drawWideLine(35 - 10, 70, 92 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(117 + 8 - 10, 70, 182 - 8 - 10, 70, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(35 - 10, 120, 92 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.drawWideLine(117 + 8 - 10, 120, 182 - 8 - 10, 120, 4, TFT_BLACK, TFT_LIGHTGREY);
    sprite.setTextSize(2);
    sprite.drawString(String(EditTens), 60 - 11, 89);
    sprite.drawString(String(EditOnes), 150 - 11, 89);
    sprite.setTextSize(1);

    sprite.drawString("Select Gender", 254, 42, 2);
    sprite.drawString("MALE", 225, 115, 2);
    sprite.drawString("FEMALE", 275, 115, 2);
    EditAge = (EditTens * 10) + EditOnes;
    if (EditAge == 0)
    {
      sprite.drawString("INVALID AGE", 190 + 60, 149, 2);
    }
    else if (EditGender == "")
    {
      sprite.drawString("INPUT GENDER", 190 + 60, 149, 2);
    }
    if (!MaleHold)
    {
      if ((XCoord <= 100) && (XCoord >= 60) && (YCoord <= 240) && (YCoord >= 180))
      {
        sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
        XCoordLS = XCoord;
        MaleHold = true;
      }
      else
      {
        if (EditGender != "Male")
        {
          sprite.drawRoundRect(200 + 4, 60, 40, 40, 4, TFT_BLACK); // Male
        }
        else
        {
          sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
        }
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        EditGender = "Male";
        sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
        MaleHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
      }
      else
      {
        sprite.fillRoundRect(200 + 4, 60, 40, 40, 4, TFT_ORANGE); // Male
        MaleHold = false;
      }
    }
    if (!FemaleHold)
    {
      if ((XCoord <= 100) && (XCoord >= 60) && (YCoord <= 300) && (YCoord >= 260))
      {
        sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
        XCoordLS = XCoord;
        FemaleHold = true;
      }
      else
      {
        if (EditGender != "Female")
        {
          sprite.drawRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLACK); // Female
        }
        else
        {
          sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
        }
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        EditGender = "Female";
        sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
        FemaleHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
      }
      else
      {
        sprite.fillRoundRect(250 + 4, 60, 40, 40, 4, TFT_BLUE); // Female
        FemaleHold = false;
      }
    }
    if (!TenUpHold)
    {
      if ((XCoord <= 60) && (XCoord >= 50) && (YCoord <= 60) && (YCoord >= 0))
      {
        sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
        XCoordLS = XCoord;
        TenUpHold = true;
      }
      else
      {
        sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_BLACK); // Ten up
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditTens <= 8)
        {
          EditTens++;
        }
        else
        {
          EditTens = 0;
        }
        sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
        TenUpHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
      }
      else
      {
        sprite.fillTriangle(40, 36 + 25, 50, 36 + 10, 60, 36 + 25, TFT_DARKGREY); // Ten up
        TenUpHold = false;
      }
    }
    if (!TenDwHold)
    {
      if ((XCoord <= 165) && (XCoord >= 120) && (YCoord <= 70) && (YCoord >= 10))
      {
        sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
        XCoordLS = XCoord;
        TenDwHold = true;
      }
      else
      {
        sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditTens >= 1)
        {
          EditTens--;
        }
        else
        {
          EditTens = 9;
        }
        sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
        TenDwHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_DARKGREY); // Ten down
      }
      else
      {
        sprite.fillTriangle(40, 155 - 25, 50, 155 - 10, 60, 155 - 25, TFT_BLACK); // Ten down
        TenDwHold = false;
      }
    }
    if (!OneUpHold)
    {
      if ((XCoord <= 70) && (XCoord >= 40) && (YCoord <= 165) && (YCoord >= 100))
      {
        sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
        XCoordLS = XCoord;
        OneUpHold = true;
      }
      else
      {
        sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditOnes <= 8)
        {
          EditOnes++;
        }
        else
        {
          EditOnes = 0;
        }
        sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_BLACK); // One up
        OneUpHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
      }
      else
      {
        sprite.fillTriangle(40 + 90, 36 + 25, 50 + 90, 36 + 10, 60 + 90, 36 + 25, TFT_DARKGREY); // One up
        OneUpHold = false;
      }
    }
    if (!OneDwHold)
    {
      if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 165) && (YCoord >= 100))
      {
        sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
        XCoordLS = XCoord;
        OneDwHold = true;
      }
      else
      {
        sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        if (EditOnes >= 1)
        {
          EditOnes--;
        }
        else
        {
          EditOnes = 9;
        }
        sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
        OneDwHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_DARKGREY); // One down
      }
      else
      {
        sprite.fillTriangle(40 + 90, 155 - 25, 50 + 90, 155 - 10, 60 + 90, 155 - 25, TFT_BLACK); // One down
        OneDwHold = false;
      }
    }
    if ((editPage != 0) && (EditGender != ""))
    {
      sprite.drawString("CONFIRM", 190 + 60, 149, 2);
      if (!NextHold)
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320) && (YCoord >= 200))
        {
          XCoordLS = XCoord;
          NextHold = true;
        }
        else
        {
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          editPage = 4;
          NextHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
        }
        else
        {
          NextHold = false;
        }
      }
    }
  }
  else if (editPage == 4)
  {
    sprite.setTextSize(1);
    sprite.setTextDatum(4);
    sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
    sprite.setTextColor(TFT_BLACK);
    // end and reset valls
    if (!FinaliseInput)
    {
      sprite.drawString("Profile EDIT", 160, 35);
      sprite.setTextColor(TFT_WHITE);
      sprite.drawString("Name : " + EditName, 160, 65);
      sprite.drawString("Sex : " + EditGender, 160, 90);
      sprite.drawString("Age : " + String(EditAge), 160, 115);
      if (!NextHold) // confirm button
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 250) && (YCoord >= 150))
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          XCoordLS = XCoord;
          NextHold = true;
        }
        else
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          FinaliseInput = true;
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          NextHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
        }
        else
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          NextHold = false;
        }
      }
      if (!EditBackHold) // Back button
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 145) && (YCoord >= 80))
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          XCoordLS = XCoord;
          EditBackHold = true;
        }
        else
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          editPage--;
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          EditBackHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
        }
        else
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          EditBackHold = false;
        }
      }
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("CONFIRM", 193, 150, 2);
      sprite.drawString("BACK", 120, 150, 2);
    }
    else
    {
      taskVibrate.enable();
      sprite.drawString("....EDITING....", 160, 35);
      delay(250);
      String OLDString = "/IC3-Results/Profiles/" + CurName + ".txt";
      const char *OLDChar = OLDString.c_str();
      String NEWString = "/IC3-Results/Profiles/" + EditName + ".txt";
      const char *NEWChar = NEWString.c_str();
      renameFile(SD, OLDChar, NEWChar);
      PFdump = SD.open("/IC3-Results/Profiles/" + EditName + ".txt", FILE_WRITE);
      PFdump.print(EditName + "," + String(EditAge) + "," + EditGender + "," + NewDMY);
      OLDString = "/IC3-Results/" + CurName;
      const char *OLDCharL = OLDString.c_str();
      NEWString = "/IC3-Results/" + EditName;
      const char *NEWCharL = NEWString.c_str();
      renameFile(SD, OLDCharL, NEWCharL);
      PFdump.close();
      ProfileCRmode = 0;
      YearMonthDate = 1;
      editPage = 0;
      Tens = 0;
      Ones = 0;
      FinaliseInput = false;
      ProfileSL = false;
      CurName = EditName;
      CurSex = EditGender;
      CurAge = String(EditAge);
      EditName = "";
      EditGender = "";
      EditAge = 0;
      ButtonTouched = true;
      PFfirstrun = false;
    }
  }
}

void update_ecg_buffer(int CURMilli) // For outputting correct Y axis and
{
  static int prev_bp = 0;
  int16_t *ecg_buf = get_ecg_buf();
  int bp = get_ecg_buf_pos();
  int blen = get_ecg_buf_len();
  while (prev_bp != bp)
  {
    //-->here add ecg_buf[prev_bp] into output string
    ECG_Y = ecg_buf[prev_bp];
    Uv = ECG_Y * 0.7522;
    FinalDATA = FinalDATA + String(get_BPM()) + "," + CURMilli + "," + String(ECG_Y) + "," + String(get_fall_detected()) + "\n";
    prev_bp++;
    if (prev_bp >= blen)
      prev_bp = 0;
  }
}
void VibrateCD()
{
  if (VibeOn == false)
  {
    if (Vibrate)
      digitalWrite(Vibepin, HIGH);
    VibeOn = true;
  }
  else
  {
    digitalWrite(Vibepin, LOW);
    VibeOn = false;
    taskVibrate.disable();
  }
}
void ButtonCK()
{
  if (ScreenNum == 4) // Home page Start and profile become records and settings
  {
    if (!StartRec && !Profileset) // Removes home buttons when Measuring
    {
      if (BOneP) // top
      {
        ScreenNum = 1;
        BOneP = false;
      }
      else if (BTwoP) // bottom
      {
        BTwoP = false;
      }
      else if (BThreeP) // left
      {
        ScreenNum = 3;
        BThreeP = false;
      }
    }
    if (Profileset) // press the settings
    {
      if (BOneP) // top bt
      {
        if (SettingsPage >= 1)
        {
          SettingsPage--;
        }
        else
        {
          SettingsPage = 2;
        }
        BOneP = false;
      }
      else if (BTwoP) // bottom bt
      {
        Profileset = false;
        BTwoP = false;
      }
      else if (BThreeP) // left
      {
        if (SettingsPage < 2) // Loop 1-4 temp untill touch buttons
        {
          SettingsPage++;
        }
        else
        {
          SettingsPage = 0;
        }
        BThreeP = false;
      }
      else if (BFourP) // right
      {
        File file = SD.open("/IC3-Results/Settings/Config.txt", FILE_WRITE); // recreate settings file
        String SettingsMsg = "Screen Brightness : " + String(SCBright) + ",\nScreen Timeout : " + String(ScreenOff) + ",\nVibrate : " + String(Vibrate) + ",\nILFforever,\nDefWifiPWD : 19283746\n";
        file.print(SettingsMsg);
        file.close();
        BFourP = false;
        Profileset = false;
      }
    }
    else if (StartRec) // RECORD Pressed
    {
      if ((RecordsPage == 0) || (RecordsPage == 1))
      {
        if (BOneP) // top
        {
          if (RecordsPage == 0)
          {
            RecRow--;
          }
          else if (RecordsPage == 1)
          {
            ModeRw--;
          }
          BOneP = false;
        }
        else if (BTwoP) // bottom
        {
          if (RecordsPage == 0)
          {
            StartRec = false;
          }
          else if (RecordsPage == 1)
          {
            PFfirstrun = true;
            RecordsPage--;
          }
          BTwoP = false;
        }
        else if (BThreeP) // left
        {
          if (RecordsPage == 0)
          {
            RecRow++;
          }
          else if (RecordsPage == 1)
          {
            ModeRw++;
          }
          BThreeP = false;
        }
        else if (BFourP) // right
        {
          if (RecordsPage == 0)
          {
            ProfileSL = true;
          }
          else if (RecordsPage == 1)
          {
            RecordsPage++;
            PFfirstrun = true;
          }
          BFourP = false;
        }
      }
      else if (RecordsPage == 2)
      {
        if (BOneP) // top
        {
          RecRow--;
          ButtonTouched = true;
          BOneP = false;
        }
        else if (BTwoP) // bottom
        {
          PFfirstrun = true;
          RecordsPage--; // take code from back button
          BTwoP = false;
        }
        else if (BThreeP) // left
        {
          RecRow++;
          ButtonTouched = true;
          BThreeP = false;
        }
        else if (BFourP) // right
        {
          ProfileSL = true;
          BFourP = false;
        }
      }
      else if (RecordsPage == 3)
      {
        if (BOneP) // top
        {
          BOneP = false;
        }
        else if (BTwoP) // bottom
        {
          if (RecDel)
          {
            RecDel = false;
          }
          else
          {
            RecordsPage--;
            ProfileSL = false;
            PFfirstrun = true;
          }
          BTwoP = false;
        }
        else if (BThreeP) // left
        {
          if (!RecDel)
          {
            RecDel = true;
          }
          else
          {
            RecDel = false;
          }
          BThreeP = false;
        }
        else if (BFourP) // right
        {
          if (RecDel)
          {
            RecDelConfirm = true;
          }
          else
          {
            RecServer = true;
          }
          BFourP = false;
        }
      }
    }
  }
  else
  {
    if (!StartRec && !Profileset) // Removes home buttons when Measuring
    {
      if (BOneP) // top
      {
        if (ScreenNum < 4)
        {
          ScreenNum++;
        }
        else
        {
          ScreenNum = 1;
        }
        BOneP = false;
      }
      else if (BTwoP) // bottom
      {
        BTwoP = false;
      }
      else if (BThreeP) // left
      {
        if (ScreenNum > 1)
        {
          ScreenNum--;
        }
        else
        {
          ScreenNum = 4;
        }
        BThreeP = false;
      }
      else if (BFourP) // right
      {
        BFourP = false;
      }
    }
    else if (Profileset) // press the profile button
    {
      if (BOneP) // top
      {
        BOneP = false;
      }
      else if (BTwoP) // bottom
      {
        BTwoP = false;
      }
      else if (BThreeP) // left
      {
        BThreeP = false;
      }
      else if (BFourP) // right
      {
        BFourP = false;
      }
    }
  }
}
void ConCheck()
{                               // Check for additional modules on Homescreen ONLY
  if (!StartRec && !Profileset) // Removes home buttons when Measuring
  {
    // Serial.println(analogRead(SensorDetecPin));
    if (analogRead(SensorDetecPin) >= 1000)
    {
      if (SerialDebug)
        if (MaxConnect != false)
        {
          if (!particleSensor.begin(Wire1))
          {
            MaxConnect = false;
          }
          else
          {
            byte ledBrightness = 60;                                                                       // Options: 0=Off to 255=50mA
            byte sampleAverage = 4;                                                                        // Options: 1, 2, 4, 8, 16, 32
            byte ledMode = 2;                                                                              // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
            byte sampleRate = 100;                                                                         // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
            int pulseWidth = 411;                                                                          // Options: 69, 118, 215, 411
            int adcRange = 4096;                                                                           // Options: 2048, 4096, 8192, 16384                                                                     // Options: 2048, 4096, 8192, 16384                                                                   // Options: 2048, 4096, 8192, 16384
            particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
            MaxConnect = true;
          }
        }
    }
    else
    {
      MaxConnect = false;
    }
  }
}

bool SizeChk(String Path) // this code relies on comma position please don't change format of data too much :(
{
  BatchDone = false;
  while (!BatchDone)
  {
    File myFile = SD.open(Path);
    int Lost = myFile.size() - Posi;
    myFile.seek(myFile.size() - BUFSIZE);
    memset(package, '\0', BUFSIZE);
    while (myFile.available())
    {
      Posi = myFile.position();
      myFile.readBytes(package, BUFSIZE);
    }
    BatchDone = true;
    myFile.close();
  }
  int shift = 0;
  String i;
  int CommaCnt = 0;
  while (CommaCnt < 2)
  {
    i = String(package[BUFSIZE - shift]);
    if (i == ",")
    {
      CommaCnt++;
    }
    shift++;
  }
  String Out, FinalTime;
  while (CommaCnt < 3)
  {
    i = String(package[BUFSIZE - shift]);
    if (i == ",")
    {
      CommaCnt++;
    }
    else
    {
      Out = Out + i;
    }
    shift++;
  }
  for (int l = Out.length(); l > 0; l--)
  {
    FinalTime += Out[l - 1];
  }
  Inttime = FinalTime.toFloat();
  if (Inttime <= 30000)
  {
    // Serial.println("Fail");
    return false;
  }
  else
  {
    // Serial.println("Pass");
    return true;
  }
}

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

void WifiScr()
{
  Topbar();
  Backbutton(); // create a back button; on top left
  if (WifiPage == 0)
  {
    if (WifiSL)
    {
      if (Wifirow == 1)
      {
        SLWifi = Row2W;
        SLPW = Row2P;
        SLST = Row2ST;
        if (SLWifi == "")
          WifiSL = false;
      }
      else if (Wifirow == 2)
      {
        SLWifi = Row3W;
        SLPW = Row3P;
        SLST = Row3ST;
        if (SLWifi == "")
          ProfileSL = false;
      }
      else if (Wifirow == 3)
      {
        SLWifi = Row4W;
        SLPW = Row4P;
        SLST = Row4ST;
        if (SLWifi == "")
          ProfileSL = false;
      }
      else if (Wifirow == 4)
      {
        SLWifi = Row5W;
        SLPW = Row5P;
        SLST = Row5ST;
        if (SLWifi == "")
          WifiSL = false;
      }
      else if (Wifirow == 5)
      {
        SLWifi = Row6W;
        SLPW = Row6P;
        SLST = Row6ST;
        if (SLWifi == "")
          WifiSL = false;
      }
      if (SLWifi != "") // if data not empty go to next page
      {
        WifiPage++;
        WifiSL = false;
      }
    }
    else
    {
      File PF = SD.open("/IC3-Results/Settings/WifiList");
      sprite.fillRect(8, 27, 250, 138, TFT_BLUE);               // BG for data table
      sprite.fillRect(8, 27 + 23, 250, 138 - 23, TFT_DARKCYAN); // BG for data table
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("WIFI-NAME", 100, 38, 2);
      sprite.drawString("SELECTED", 218, 38, 2);
      if (Wifirow > 5) // shift data rows down
      {
        Wifirow = 5;
        InternalWifiRw++;
        // Move data
      }
      if (Wifirow < 1) // shift data rows up
      {
        Wifirow = 1;
        if (InternalWifiRw > 0)
        {
          InternalWifiRw--;
        }
      }
      Wifilist(PF);
      // Serial.println("CurWifi = " + CurWifi);
      PF.close();
      if (Profileamount == 0)
      {
        Row2W = "";
        Row2P = "";
        Row2ST = "";
      }
      sprite.setTextColor(TFT_BLACK);
    }
    if (Wifirow == 1)
    {
      sprite.fillRect(8, 27 + 23, 171, 23, TFT_CYAN);
    }
    sprite.drawString(Row2W, 50 + (Row2W.length() * 1), 64, 2);
    if (Row2ST == "YES")
    {
      sprite.fillRect(179, 27 + 23, 80, 23, TFT_GREEN);
    }
    else if (Row2ST == "NO")
    {
      sprite.fillRect(179, 27 + 23, 80, 23, TFT_RED);
    }
    sprite.drawString(Row2ST, 218, 64, 2);
    if (Wifirow == 2)
    {
      sprite.fillRect(8, 27 + 46, 171, 23, TFT_CYAN);
    }
    if (Row3ST == "YES")
    {
      sprite.fillRect(179, 27 + 46, 80, 23, TFT_GREEN);
    }
    else if (Row3ST == "NO")
    {
      sprite.fillRect(179, 27 + 46, 80, 23, TFT_RED);
    }
    sprite.drawString(Row3W, 50 + (Row2W.length() * 1), 38 + 46, 2);
    sprite.drawString(Row3ST, 218, 38 + 46, 2);
    if (Wifirow == 3)
    {
      sprite.fillRect(8, 27 + 69, 171, 23, TFT_CYAN);
    }
    if (Row4ST == "YES")
    {
      sprite.fillRect(179, 27 + 69, 80, 23, TFT_GREEN);
    }
    else if (Row4ST == "NO")
    {
      sprite.fillRect(179, 27 + 69, 80, 23, TFT_RED);
    }
    sprite.drawString(Row4W, 50 + (Row2W.length() * 1), 38 + 69, 2);
    sprite.drawString(Row4ST, 218, 38 + 69, 2);
    if (Wifirow == 4)
    {
      sprite.fillRect(8, 27 + 92, 171, 23, TFT_CYAN);
    }
    if (Row5ST == "YES")
    {
      sprite.fillRect(179, 27 + 92, 80, 23, TFT_GREEN);
    }
    else if (Row5ST == "NO")
    {
      sprite.fillRect(179, 27 + 92, 80, 23, TFT_RED);
    }
    sprite.drawString(Row5W, 50 + (Row2W.length() * 1), 38 + 92, 2);
    sprite.drawString(Row5ST, 218, 38 + 92, 2);
    if (Wifirow == 5)
    {
      sprite.fillRect(8, 27 + 115, 171, 23, TFT_CYAN);
    }
    if (Row6ST == "YES")
    {
      sprite.fillRect(179, 27 + 115, 80, 23, TFT_GREEN);
    }
    else if (Row6ST == "NO")
    {
      sprite.fillRect(179, 27 + 115, 80, 23, TFT_RED);
    }
    sprite.drawString(Row6W, 50, 38 + 115, 2);
    sprite.drawString(Row6ST, 218, 38 + 115, 2);
    // display the pf amount here
    if (1)
    {                                                                  // draw borders
      sprite.drawLine(8, 27 + 138, 258, 165, TFT_WHITE);               // X axis bottom?
      sprite.drawLine(8, 27, 258, 27, TFT_WHITE);                      // X axis top?
      sprite.drawLine(8 + 160 + 10, 27, 8 + 160 + 10, 165, TFT_WHITE); // Y axis line 2
    }
    // sprite.drawString("Create New Profile", 120, 64, 2);
    for (int i = 0; i < 6; i++)
    {
      sprite.drawLine(8, 27 + (i * 23), 258, 27 + (i * 23), TFT_WHITE); // graph X line
    }
    sprite.drawLine(8, 27, 8, 165, TFT_WHITE);             // X axis left most
    sprite.drawLine(8 + 250, 27, 8 + 250, 165, TFT_WHITE); // X axis right most
    sprite.fillSmoothRoundRect(5 + 60, 0, 70, 16, 4, TFT_DARKGREY, TFT_BLACK);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString(String(Profileamount) + " : WIFIs", 60 + 40, 8, 2);
  }
  else if (WifiPage == 1)
  {
    sprite.setTextSize(1);
    sprite.setTextDatum(4);
    sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("WIFI SELECTED", 160, 35);
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("SSID : " + SLWifi, 160, 70);
    sprite.drawString("PW : " + SLPW, 160, 95);
  }
  else if (WifiPage == 2)
  { // New wifi
    if (!WifiSettingFR)
    {
      WebHostinit();
      WifiSettingFR = true;
    }
    sprite.pushImage(40, 35, 100, 100, QRWIFI);
    sprite.fillRoundRect(50, 150 - 5, 80, 20, 4, TFT_LIGHTGREY);
    sprite.drawString("IC-3 TCP", 90, 160 - 5, 2);
    sprite.fillRoundRect(190, 40, 60, 20, 4, TFT_LIGHTGREY);
    sprite.drawString("STEP 1", 220, 50, 2);
    sprite.fillRoundRect(160, 70, 120, 45, 4, TFT_LIGHTGREY);
    sprite.drawString("SCAN QR-CODE TO", 220, 80, 2);
    sprite.drawString("CONNECT TO WIFI", 220, 100, 2);
  }
  else if (WifiPage == 3)
  {
    sprite.pushImage(40, 35, 100, 100, WebQR);
    sprite.fillRoundRect(40, 150 - 5, 100, 20, 4, TFT_LIGHTGREY);
    sprite.drawString("192.168.4.1", 90, 160 - 5, 2);
    sprite.fillRoundRect(190, 40, 60, 20, 4, TFT_LIGHTGREY);
    sprite.drawString("STEP 2", 220, 50, 2);
    sprite.fillRoundRect(160, 70, 120, 45, 4, TFT_LIGHTGREY);
    sprite.drawString("INPUT DATA INTO", 220, 80, 2);
    sprite.drawString("EACH DATA FIELD", 220, 100, 2);
    if ((!getSSID) || (!getPW))
    {
      if (!getSSID)
      {
        sprite.fillRoundRect(160, 120, 120, 20, 4, TFT_RED);
        sprite.drawString("MISSING SSID", 220, 130, 2);
      }
      else
      {
        sprite.fillRoundRect(160, 120, 120, 20, 4, TFT_GREEN);
        sprite.drawString("AQUIRED SSID", 220, 130, 2);
      }
      if (!getPW)
      {
        sprite.fillRoundRect(160, 145, 120, 20, 4, TFT_RED);
        sprite.drawString("MISSING PW", 220, 155, 2);
      }
      else
      {
        sprite.fillRoundRect(160, 145, 120, 20, 4, TFT_GREEN);
        sprite.drawString("AQUIRED PW", 220, 155, 2);
      }
    }
  }
  else if (WifiPage == 4)
  {
    sprite.setTextSize(1);
    sprite.setTextDatum(4);
    sprite.fillRoundRect(44, 24, 235, 30, 4, TFT_LIGHTGREY);
    sprite.setTextColor(TFT_BLACK);
    sprite.drawString("WIFI INPUTED", 160, 35);
    sprite.setTextColor(TFT_WHITE);
    sprite.drawString("SSID : " + NewSSID, 160, 70);
    sprite.drawString("PW : " + NewPW, 160, 95);
  }
}

void WifiTouch()
{
  if (WifiPage == 0)
  {
    if (!WifiSL) // Wifi not selected
    {
      sprite.fillRoundRect(265, 27, 50, 40, 2, TFT_DARKGREY);
      sprite.fillRoundRect(265, 71, 50, 50, 2, TFT_DARKGREY);
      sprite.fillRoundRect(265, 126, 50, 40, 2, TFT_DARKGREY);
      if (!ProfileDWHold)
      {
        if ((XCoord <= 70) && (XCoord >= 5) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileDWHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          Wifirow--;
          ButtonTouched = true;
          // Serial.println("ProfileRW =" + String(ProfileRw));
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
          ProfileDWHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 27 + 5, 40, 30, 2, TFT_BLUE);
          ProfileDWHold = false;
        }
      }
      if (!ProfileUPHold)
      {
        if ((XCoord <= 170) && (XCoord >= 125) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileUPHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          Wifirow++;
          ButtonTouched = true;
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
          ProfileUPHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 126 + 5, 40, 30, 2, TFT_BLUE);
          ProfileUPHold = false;
        }
      }
      if (!ProfileSLHold)
      {
        if ((XCoord <= 120) && (XCoord >= 70) && (YCoord <= 320) && (YCoord >= 250))
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
          XCoordLS = XCoord;
          ProfileSLHold = true;
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          WifiSL = true;
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
          ProfileSLHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(265 + 5, 71 + 5, 40, 40, 2, TFT_BLUE);
          ProfileSLHold = false;
        }
      }
      if (Sdvalid)
      {
        if (!CancelDeleteHOLD) // new wifi button
        {
          if ((XCoord <= 5) && (XCoord >= 0) && (YCoord <= 50 + 135) && (YCoord >= 1 + 135))
          {
            sprite.fillSmoothRoundRect(10 + 135, 0, 40, 16, 4, TFT_DARKGREY, TFT_BLACK);
            XCoordLS = XCoord;
            CancelDeleteHOLD = true;
          }
          else
          {
            sprite.fillSmoothRoundRect(10 + 135, 0, 40, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
          }
        }
        else
        {
          if ((XCoord == 0) && (YCoord == 0))
          {
            WifiPage = 2;
            sprite.fillSmoothRoundRect(10 + 135, 0, 40, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
            CancelDeleteHOLD = false;
          }
          else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
          {
            sprite.fillSmoothRoundRect(10 + 135, 0, 40, 16, 4, TFT_DARKGREY, TFT_BLACK);
          }
          else
          {
            sprite.fillSmoothRoundRect(10 + 135, 0, 40, 16, 4, TFT_LIGHTGREY, TFT_BLACK);
            CancelDeleteHOLD = false;
          }
        }
        sprite.drawString("NEW", 32 + 135, 8, 2);
      }
      sprite.fillTriangle(280, 36 + 20, 290, 36, 300, 36 + 20, TFT_CYAN);
      sprite.fillTriangle(280, 155 - 20, 290, 155, 300, 155 - 20, TFT_CYAN);
      sprite.fillCircle(289, 95, 15, TFT_LIGHTGREY);
    }
  }
  else if (WifiPage == 1) // Selected and del
  {
    if (confirmDEL) // delete confirm
    {
      if (!NextHold) // confirm button
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 250) && (YCoord >= 150))
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_ORANGE);
          XCoordLS = XCoord;
          NextHold = true;
        }
        else
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          confirmDEL = false;
          WifiSL = false;
          WifiPage = 0;
          sprite.drawString("....DELETING....", 160, 35);
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
          String DELString = "/IC3-Results/Settings/WifiList/" + SLWifi + ".txt";
          const char *DelChar = DELString.c_str();
          removeFile(SD, DelChar);
          ButtonTouched = true;
          delay(250);
          NextHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_ORANGE);
        }
        else
        {
          sprite.fillRoundRect(235 - 75, 130 + 8, 70, 30, 4, TFT_RED);
          NextHold = false;
        }
      }

      if (!CancelDeleteHOLD) // cancel del button //reuse vall
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 145) && (YCoord >= 80))
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
          XCoordLS = XCoord;
          CancelDeleteHOLD = true;
        }
        else
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          confirmDEL = false;
          ButtonTouched = true;
          CancelDeleteHOLD = false;
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_BLUE);
        }
        else
        {
          sprite.fillRoundRect(160 - 75, 130 + 8, 70, 30, 4, TFT_SKYBLUE);
          CancelDeleteHOLD = false;
        }
      }
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("DELETE", 193, 150, 2);
      sprite.drawString("BACK", 122, 150, 2);
    }
    else
    {
      if (!ProfileUPHold) // Back buttom
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 95 + 35) && (YCoord >= 0 + 35))
        {
          sprite.fillRoundRect(10 + 35, 130 + 8, 70, 30, 4, TFT_BLACK);
          XCoordLS = XCoord;
          ProfileUPHold = true;
        }
        else
        {
          sprite.fillRoundRect(10 + 35, 130 + 8, 70, 30, 4, TFT_DARKGREY);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          WifiSL = false;
          WifiPage = 0;
          sprite.fillRoundRect(10 + 35, 130 + 8, 70, 30, 4, TFT_DARKGREY);
          ProfileUPHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(10 + 35, 130 + 8, 70, 30, 4, TFT_BLACK);
        }
        else
        {
          sprite.fillRoundRect(10 + 35, 130 + 8, 70, 30, 4, TFT_DARKGREY);
          ProfileUPHold = false;
        }
      }
      if (!ProfileDWHold) // delete
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 150 + 35) && (YCoord >= 99 + 35))
        {
          sprite.fillRoundRect(85 + 35, 130 + 8, 70, 30, 4, TFT_ORANGE);
          XCoordLS = XCoord;
          ProfileDWHold = true;
        }
        else
        {
          sprite.fillRoundRect(85 + 35, 130 + 8, 70, 30, 4, TFT_RED);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          confirmDEL = true;
          sprite.fillRoundRect(85 + 35, 130 + 8, 70, 30, 4, TFT_RED);
          ProfileDWHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(85 + 35, 130 + 8, 70, 30, 4, TFT_ORANGE);
        }
        else
        {
          sprite.fillRoundRect(85 + 35, 130 + 8, 70, 30, 4, TFT_RED);
          ProfileDWHold = false;
        }
      }

      if (!ProfileSLHold) // select button
      {
        if ((XCoord <= 170) && (XCoord >= 120) && (YCoord <= 320 - 35) && (YCoord >= 230 - 35))
        {
          sprite.fillRoundRect(235 - 35, 130 + 8, 70, 30, 4, TFT_DARKGREEN);
          XCoordLS = XCoord;
          ProfileSLHold = true;
        }
        else
        {
          sprite.fillRoundRect(235 - 35, 130 + 8, 70, 30, 4, TFT_GREEN);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          // Serial.println("SLWifi = " + SLWifi);
          // Serial.println("SLPW = " + SLPW);
          // Serial.println("CurWifi = " + CurWifi);
          // Serial.println("CurPW = " + CurPW);
          String Path;
          File file;
          if (CurWifi != "")
          {
            Path = "/IC3-Results/Settings/WifiList/" + CurWifi + ".txt";
            const char *PathCr = Path.c_str();
            file = SD.open(PathCr, FILE_WRITE);
            file.print(CurPW + "^NO");
            file.close();
          }
          Path = "/IC3-Results/Settings/WifiList/" + SLWifi + ".txt";
          const char *PathCH = Path.c_str();
          file = SD.open(PathCH, FILE_WRITE);
          file.print(SLPW + "^YES");
          file.close();
          WifiPage = 0;
          WifiSL = false;
          WFfirstrun = true;
          ProfileCRmode = 0;
          sprite.fillRoundRect(235 - 35, 130 + 8, 70, 30, 4, TFT_GREEN);
          ProfileSLHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(235 - 35, 130 + 8, 70, 30, 4, TFT_DARKGREEN);
        }
        else
        {
          sprite.fillRoundRect(235 - 35, 130 + 8, 70, 30, 4, TFT_GREEN);
          ProfileSLHold = false;
        }
      }
      sprite.setTextColor(TFT_BLACK);
      sprite.drawString("BACK", 10 + 37 + 35, 150, 2);
      sprite.drawString("DELETE", 85 + 37 + 35, 150, 2);
      sprite.drawString("SELECT", 235 + 37 - 35, 150, 2);
      sprite.setTextSize(1);
    }
  }
  else if (WifiPage == 2)
  {
    if (!NextHold) // confirm button
    {
      if ((XCoord <= 170) && (XCoord >= 115) && (YCoord <= 260) && (YCoord >= 150))
      {
        sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_NAVY);
        XCoordLS = XCoord;
        NextHold = true;
      }
      else
      {
        sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
        WifiPage = 3;
        NextHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
        NextHold = false;
      }
    }
    sprite.drawString("CONTINUE", 220, 140, 2);
  }
  else if (WifiPage == 3)
  {
    if ((getSSID) && (getPW))
    {
      if (!NextHold) // confirm button
      {
        if ((XCoord <= 170) && (XCoord >= 115) && (YCoord <= 260) && (YCoord >= 150))
        {
          sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_NAVY);
          XCoordLS = XCoord;
          NextHold = true;
        }
        else
        {
          sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
        }
      }
      else
      {
        if ((XCoord == 0) && (YCoord == 0))
        {
          sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
          WifiPage = 4;
          NextHold = false;
        }
        else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
        {
          sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_NAVY);
        }
        else
        {
          sprite.fillRoundRect(160, 130, 120, 20, 4, TFT_BLUE);
          NextHold = false;
        }
      }
      sprite.drawString("CONTINUE", 220, 140, 2);
    }
  }
  else if (WifiPage == 4)
  {
    if (!ProfileUPHold) // back
    {
      if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 130) && (YCoord >= 40))
      {
        sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_NAVY);
        XCoordLS = XCoord;
        ProfileUPHold = true;
      }
      else
      {
        sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_BLUE);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        WifiPage = 3;
        sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_BLUE);
        ProfileUPHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_NAVY);
      }
      else
      {
        sprite.fillRoundRect(40, 130, 100, 30, 5, TFT_BLUE);
        ProfileUPHold = false;
      }
    }
    if (!ProfileSLHold) // confirm save new wifi
    {
      if ((XCoord <= 160) && (XCoord >= 120) && (YCoord <= 300) && (YCoord >= 160))
      {
        sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_DARKGREEN);
        XCoordLS = XCoord;
        ProfileSLHold = true;
      }
      else
      {
        sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_GREEN);
      }
    }
    else
    {
      if ((XCoord == 0) && (YCoord == 0))
      {
        String Path = "/IC3-Results/Settings/WifiList/" + NewSSID + ".txt";
        const char *PathCr = Path.c_str();
        String SettingData = readFile(SD, PathCr);
        int PWlen = NewPW.length();
        Serial.println(String(PWlen));
        sprite.setTextColor(TFT_RED);
        if ((FileFound) && (PWlen < 8))
        {
          sprite.drawString("DUPLICATE WIFI NAME", 160, 118, 2);
          sprite.drawString("PASSWORD TOO SHORT", 160, 100, 2);
          delay(1000);
        }
        else if (FileFound)
        {
          sprite.drawString("DUPLICATE WIFI NAME", 160, 118, 2);
          delay(500);
        }
        else if (PWlen < 8)
        {
          sprite.drawString("PASSWORD TOO SHORT", 160, 118, 2);
          delay(500);
        }
        else
        {
          sprite.setTextColor(TFT_BLACK);
          SD1 = SD.open(PathCr, FILE_WRITE);
          String Msg = NewPW + "^NO";
          SD1.print(Msg);
          SD1.close();
          WifiPage = 0;
          WiFi.mode(WIFI_MODE_NULL); // Turn off wifi
          WifiSettingFR = false;
        }
        sprite.setTextColor(TFT_BLACK);
        sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_GREEN);
        ProfileSLHold = false;
      }
      else if ((XCoord <= XCoordLS + 5) && (XCoord >= XCoordLS - 5)) // Don't need Y X alone works well
      {
        sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_DARKGREEN);
      }
      else
      {
        sprite.fillRoundRect(320 - 140, 130, 100, 30, 5, TFT_GREEN);
        ProfileSLHold = false;
      }
    }
    sprite.drawString("BACK", 90, 140 + 5, 2);
    sprite.drawString("CONFIRM", 230, 140 + 5, 2);
  }
}
void WebHostinit()
{
  getSSID = false;
  getPW = false;
  FinalisedWIFI = false;
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(IP);
  /// server.begin();
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      getSSID = true;
      NewSSID = inputMessage;
    }  // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
      getPW = true;
      NewPW = inputMessage;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
      
    }
    //Serial.println(inputMessage);
    request->send(200, "text/html", "<dialog open>HTTP GET request sent to your ESP on input field (" 
                                     + inputParam + ") with value: " + inputMessage +
                                     "<br><a href=\"/\">Return to Home Page</a></dialog>"); });
  server.onNotFound(notFound);
  server.begin();
}

void ScanWifiST(File dir2)
{
  while (true)
  {
    File entry = dir2.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    OBJname = entry.name();
    OBJLenth = OBJname.length();
    String AdrString = "/IC3-Results/Settings/WifiList/" + OBJname;
    const char *AdrChar = AdrString.c_str();
    FileData = readFile(SD, AdrChar);
    int str_len = FileData.length() + 1;
    char char_array[str_len]; // input data from txt
    FileData.toCharArray(char_array, str_len);
    char *pch;
    pch = strtok(char_array, "^");
    int n = 0;
    String Name, PW, State;
    Name = OBJname;
    Name = Name.substring(0, Name.length() - 4);
    // Serial.println("FinalName = " + Name);
    while (pch != NULL) // Scan untill find the one with Yes in tag
    {
      if (n == 0)
      {
        PW = String(pch);
        n++;
      }
      else if (n == 1)
      {
        State = String(pch);
        if (State == "YES")
        {
          CurWifi = Name;
          CurPW = PW;
        }
        n = 0;
      }
      pch = strtok(NULL, "^");
    }
    // Serial.println("CurWifi = " + CurWifi + "CurPW = " + CurPW);
    //  Serial.println(entry.name());
    entry.close();
  }
}