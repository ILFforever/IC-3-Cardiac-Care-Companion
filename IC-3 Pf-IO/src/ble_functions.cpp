#include "Arduino.h"
#include <Wire.h>
#include "BLEDevice.h"
#include "BLEScan.h"
#include <BLEUtils.h>
#include "SPI.h"
#include "ble_functions.h"
int uECGConnected = 4; // 1 = connected,2 = Scanning,3 = Connecting,4 = off

// Note to self DO NOT update the BLE library
// if update add ESP_BLE_AD_TYPE_NAME_SHORT: to line 297 of BLEAdvertisedDevice.cpp for name finding
enum
{
  pack_ecg_short = 1,
  pack_imu_rr_short,
  pack_hrv_short,
  pack_ecg_imu,
  pack_ecg_rr,
  pack_ecg_hrv
};
int bat_lvl;
int uECG_found = 0;
int need_scan = 1;
int BPM = 0;
int RMSSD = 0;
int cur_RR = 0;
int uecg_batt_mv = 0;
int fall_detected;
uint16_t prev_data_id = 0;
uint32_t real_data_id = 0;

int16_t ecg_buf[256];
int ecg_buf_pos = 0;
int ecg_buf_len = 256;

void parse_ecg_pack(uint8_t *buf, int length)
{
  int dat_id = (buf[0] << 8) | buf[1];
  if (real_data_id == 0)
  {
    real_data_id = dat_id;
    prev_data_id = dat_id;
  }
  int dp = dat_id - prev_data_id;
  // Serial.println(dat_id);
  if (dp < 0)
    dp += 65536;
  real_data_id += dp;
  if (dp > 13)
    dp = 13;
  int16_t ecg_vals[16];
  int pp = 2;
  int scale = buf[pp++];
  if (scale > 100)
    scale = 100 + (scale - 100) * 4;
  int16_t v0 = (buf[pp] << 8) | buf[pp + 1];
  pp += 2;
  int16_t prev_v = v0;
  for (int n = 0; n < 13; n++)
  {
    int dv = buf[pp++] - 128;
    dv *= scale;
    ecg_vals[n] = prev_v + dv;
    prev_v = ecg_vals[n];
    //    Serial.println(prev_v);
  }
  int max_diff = 0;
  for (int d = 0; d < 13 - dp; d++)
  {
    int ecg_comp_pos = ecg_buf_pos - (13 - dp) + d;
    if (ecg_comp_pos < 0)
      ecg_comp_pos += ecg_buf_len;
    int dv = ecg_buf[ecg_comp_pos] - ecg_vals[d];
    if (dv < 0)
      dv = -dv;
    if (dv > max_diff)
      max_diff = dv;
  }
  // Serial.println(max_diff);
  int is_bad = 0;
  if (max_diff > 10)
    is_bad = 1;
  static int conseq_bad = 0;
  if (is_bad)
    conseq_bad++;
  else
    conseq_bad = 0;

  if (conseq_bad > 5)
    is_bad = 0; // stream lost, can't rely on this

  if (!is_bad)
  {
    prev_data_id = dat_id;

    for (int d = 0; d < dp; d++)
    {
      ecg_buf[ecg_buf_pos] = ecg_vals[13 - dp + d];
      ecg_buf_pos++;
      if (ecg_buf_pos >= ecg_buf_len)
        ecg_buf_pos = 0;
    }
  }
}
void parse_imu_rr_pack(uint8_t *buf, int length)
{
  float T = buf[9] + 200;
  T /= 10.0;

  int rr_id = buf[12];
  int rr1 = (buf[13] << 4) | (buf[14] >> 4);
  int rr2 = ((buf[14] & 0xF) << 8) | buf[15];
  BPM = buf[16];
  cur_RR = rr2;
  if (buf[18] & 0b01)
    fall_detected = 1;
  else
    fall_detected = 0;
}
void parse_hrv_pack(uint8_t *buf, int length)
{
  int pp = 15;
  int SDRR = buf[pp++] << 4;
  SDRR += buf[pp] >> 4;
  RMSSD = (buf[pp++] & 0xF) << 8;
  RMSSD += buf[pp++];
  uecg_batt_mv = buf[pp] * 10 + 2000;
   bat_lvl = (uecg_batt_mv - 3300) / 9;
}
void parse_uecg_data(uint8_t *buf, int length)
{
  if (buf[0] == pack_ecg_short)
    parse_ecg_pack(buf + 1, length);
  if (buf[0] == pack_imu_rr_short)
    parse_imu_rr_pack(buf + 1, length);
  if (buf[0] == pack_hrv_short)
    parse_hrv_pack(buf + 1, length);

  uECG_found = 1;
}

// static ClientCallbacks clientCB;
// The remote service we wish to connect to.
static BLEUUID serviceUUID("93375900-F229-8B49-B397-44B5899B8600");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("FC7A850D-C1A5-F61F-0DA7-9995621FBD00");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  parse_uecg_data(pData, length);
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }

  void onDisconnect(BLEClient *pclient)
  {
    doConnect = false;
    doScan = true; // Scan again
    connected = false;
    uECGConnected = 2;
    //Serial.println("onDisconnect");
  }
};

bool connectToServer()
{
  BLEClient *pClient = BLEDevice::createClient();
  // Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
                              // Serial.println(" - Connected to server");
  pClient->setMTU(517);       // set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    // Serial.print("Failed to find our service UUID: ");
    // Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    uECGConnected = 2;
    return false;
  }
  // Serial.println(" - Found our service");
  //  Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    // Serial.print("Failed to find our characteristic UUID: ");
    // Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    uECGConnected = 2;
    return false;
  }
  // Serial.println(" - Found our characteristic");
  //  uECGConnected = 3; // Connecting
  //   Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
  }
  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  uECGConnected = 1;
  connected = true;
  return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // Serial.print("BLE Advertised Device found: ");
    // Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.haveName())
    {
      const char *nm = advertisedDevice.toString().c_str();
      //Serial.println(nm);
      if (nm[6 + 0] == 'u' && nm[6 + 1] == 'E' && nm[6 + 2] == 'C' && nm[6 + 3] == 'G')
      {
        uECGConnected = 3;
        //Serial.println("uECG found");
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = false;
      }
      else
      {
        uECGConnected = 2;
        connected = false;
        doScan = true;
      }
    } // Found our server
  }   // onResult
};    // MyAdvertisedDeviceCallbacks

void ble_functions_init()
{
  Serial.setDebugOutput(0);
  if (btStart())
  {
    // Serial.print("BLE On");
  } // Turn on BLE
  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(16);
  pBLEScan->setWindow(15);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
void BLE_Reset()
{
  doConnect = false;
  connected = false;
  doScan = false;
}
// This is the Arduino main loop function.
void ble_cycle()
{
  if (doConnect == true) // Connect to server
  {
    if (connectToServer())
    {
      //Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
     // Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  if (connected)
  {
    uECGConnected = 1;
  }
  else if (doScan) // If DC and rescan
  {
    uECGConnected = 2;
    BLEDevice::getScan()->start(3); // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  else
  {
    uECGConnected = 2; // not Connected
  }
  delay(10);
} // End of loop

int get_BPM()
{
  return BPM;
}
int get_RR()
{
  return cur_RR;
}
int get_uecg_batt()
{
  return bat_lvl;
}
int16_t *get_ecg_buf()
{
  return ecg_buf;
}
int get_ecg_buf_pos()
{
  return ecg_buf_pos;
}
int get_ecg_buf_len()
{
  return ecg_buf_len;
}
int get_fall_detected()
{
  return fall_detected;
}