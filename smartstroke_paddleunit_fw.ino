#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t basestationAddress[] = {0x8c, 0xaa, 0xb5, 0x8c, 0x4a, 0x38};

// Structure example to send data 
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    long int time;
    int AccX;
    int AccY;
    int AccZ;
    int ADC;
    int GyroX;
    int GyroY;
    int GyroZ;
} struct_message;

// Create a struct_message called myData
struct_message myData;
//time is 32 bit int
// 7 data values 16 bit
// Each data point is 
// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.println("Setup");
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  //Init ESP-NOW
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
  //Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  Serial.println("Loop");
  // Set values to send
  myData.id = 1;
  myData.time = 12.001;
  myData.AccX = 11.234; 
  myData.AccY = -10.566; 
  myData.AccZ = 7.401; 
  myData.ADC = 13.401; 
  myData.GyroX = 34.234; 
  myData.GyroY = 21.566; 
  myData.GyroZ = 9.401; 
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
