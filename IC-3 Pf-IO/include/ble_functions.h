#include "Arduino.h"

void ble_functions_init();
void ble_cycle();
void BLE_Reset();
int get_BPM();
int get_uecg_batt();
int16_t* get_ecg_buf();
int get_ecg_buf_pos();
int get_ecg_buf_len();
int get_fall_detected();
extern int uECGConnected;
