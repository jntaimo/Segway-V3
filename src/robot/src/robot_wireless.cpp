/*
This code receives data from the remote

Data is sent using the ESP-NOW Protocol
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

joystickData joystick;

bool freshWirelessData = false;

esp_now_peer_info_t peerInfo;

//TODO replace this address with the address of the remote 
uint8_t broadcastAddress[] = {0x70, 0x04, 0x1D, 0xAD, 0xD4, 0x94};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystick, incomingData, sizeof(joystick));
  freshWirelessData = true;
}

void wirelessSetup(void){

// Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Tell the microcontroller which functions to call when
  //data is sent or received
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  

  // Register peer to send to
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete

  //default the joystick values
    joystick.x = 512;
    joystick.y = 512;
}

//sends the currently stored odometry data to the remote
bool sendRobotData(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &robot, sizeof(robot));
  return result == ESP_OK;
}