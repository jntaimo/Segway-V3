
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <wireless.h>
#include <UMS3.h>

#define X_POT_PIN A0
#define Y_POT_PIN A1

//replace with the mac address of the robot
uint8_t broadcastAddress[] = {0x70, 0x04, 0x1D, 0xAD, 0xD4, 0x44};

// Create a struct_message called myData
joystickData joystick;
robotData robot;


UMS3 ums3;


//flag to see if we have new data
bool freshRobotData = false;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

    bool success = status == ESP_NOW_SEND_SUCCESS ;
    if (success) {
      //Serial.printf("Sent x:%d y:%d\n", joyData.joyX, joyData.joyY);
    } else {
      //Serial.println("Failed");
    }

}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&robot, incomingData, sizeof(robot));
  freshRobotData = true;
}

bool readJoystick();
void sendJoystick();
void printRobotDataSerial();

//Joystick tracking variables
uint16_t last_x = 0, last_y = 0;

void setup(void){
  Serial.begin(115200);
    //ESP_NOW Setup
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
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete
    analogReadResolution(10);
    //initialize analog pins
    pinMode(X_POT_PIN, INPUT);
    pinMode(Y_POT_PIN, INPUT);

    //start the LED
  // Initialize all board peripherals, call this first
  ums3.begin();
  // Brightness is 0-255. We set it to 1/3 brightness here
  ums3.setPixelBrightness(255 / 2);
}

//minimum time between sending data to the robot
long sendDataDelay = 50; //Millis
long lastSendData = 0;

//minimum time between sending data over serial
long printDataDelay = 20;
long lastPrintData = 0;

void loop(){
    //Send data when the joystick is ready
    if (readJoystick() && millis()-lastSendData > sendDataDelay){
      sendJoystick();    
      lastSendData = millis();
    }

    //only print data if a new reading came in
    if (freshRobotData && millis() - lastPrintData > printDataDelay){
      printRobotDataSerial();
      lastPrintData = millis();
      //wait for the next bit of fresh data
      freshRobotData = false;
    }

    if (millis() - lastPrintData > printDataDelay){
      Serial.printf("x:%d y:%d\n", last_x, last_y);
      lastPrintData = millis();
      //wait for the next bit of fresh data
      freshRobotData = false;
    }

}

//Sends the joystick readings to the broadcast address
void sendJoystick(){
    // Set values to send
    joystick.x = last_x;
    joystick.y = last_y;
    ums3.setPixelColor(last_x/4, 0, last_y/4);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystick, sizeof(joystick));
}

//Reads the current joystick values and updates the tracking variables
//Returns true if the values have changed
bool readJoystick(){
    //Assume no reading has occured
    bool newReading = false;
    //get newest reading from the joystick
    int x = analogRead(X_POT_PIN);
    int y = analogRead(Y_POT_PIN);
    //If it moved, update the position of the joystick
    if ( (abs(x - last_x) > 5)  ||  (abs(y - last_y) > 5)) {
        newReading = true;
        last_x = x;
        last_y = y;
    }    

    return newReading;
}

//Prints odometry data to be read by matlab
void printRobotDataSerial(){
  Serial.printf("%.4f\t%.4f\t%.4f\t%.4f\n", robot.millis/1000.0, robot.angle, robot.distance, robot.controlEffort);
}