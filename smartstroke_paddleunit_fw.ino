//----------------------------------------
//          Includes
//----------------------------------------
#include <esp_now.h>
#include <WiFi.h>

//----------------------------------------
//          Definitions
//----------------------------------------
#define BLUE_LED  16
#define GREEN_LED 9
#define ADC1_PIN  4
#define ADC2_PIN  5

#define BUFFER_MAX        512
#define SERIAL_BAUD_RATE  115200

//----------------------------------------
//          Prototype Functions
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

//----------------------------------------
//          Globals
//----------------------------------------
// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t basestationAddress[] = {0xcc, 0xdb, 0xa7, 0x14, 0x35, 0x38};

// Structure example to send data 
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    int x;
    int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

hw_timer_t* timerSample = NULL;  //timer for sampling 100Hz

//----------------------------------------
//          Helper Functions
//----------------------------------------
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//----------------------------------------
//          Setup
//----------------------------------------
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, basestationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

//----------------------------------------
//          Main Loop
//----------------------------------------
void loop() {
  // Set values to send
  myData.id = 1;
  myData.x = 10;
  myData.y = 20;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(basestationAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {      
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10000);
}

//----------------------------------------
//          Timer Interrupt Function
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0() {

  //toggle blue LED
  blueLEDState = !blueLEDState;
  digitalWrite(BLUE_LED, blueLEDState);
}