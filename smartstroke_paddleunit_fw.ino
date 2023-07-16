//----------------------------------------
//          Includes
//----------------------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "SPI.h"
//----------------------------------------
//          DEVICE SETTINGS
//----------------------------------------
#define BASESTATION_MAC {0xcc,0xdb,0xa7,0x14,0x35,0x38}
#define DEVICE_ID   1

//----------------------------------------
//          Definitions
//----------------------------------------
#define BLUE_LED  0
#define ADC1_PIN  4
#define ADC2_PIN  5

#define BUFFER_MAX        512
#define SERIAL_BAUD_RATE  115200

#define HSPI_MISO   2
#define HSPI_MOSI   7
#define HSPI_SCLK   6
#define HSPI_SS     10
#define SPI_CLK_SPEED 1000000

#define ACCEL_XOUT_H  0x3B
#define ACCEL_YOUT_H  0x3D
#define ACCEL_ZOUT_H  0x3F
#define GYRO_XOUT_H   0x43
#define GYRO_YOUT_H   0x45
#define GYRO_ZOUT_H   0x47
#define IMU_CONFIG    0x1A
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C
//----------------------------------------
//          Prototype Functions
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0();
void ARDUINO_ISR_ATTR TimerIntr1();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
uint8_t spiCommand(SPIClass *spi, byte data);
int16_t spiReadSensor(SPIClass *spi, uint8_t address);


//----------------------------------------
//          Globals
//----------------------------------------
// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t basestationAddress[] = BASESTATION_MAC;
int connected = 0;

// Structure example to send data 
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    long int time;
    int ADC;
} struct_message;

// Create a struct_message called myData
struct_message myData;
//time is 32 bit int
// 7 data values 16 bit
// Each data point is 
// Create peer interface
esp_now_peer_info_t peerInfo;
hw_timer_t* timerSample = NULL;  //timer for sampling 100Hz
hw_timer_t* timerLED = NULL;  //timer for sampling 100Hz
int blueLEDState = LOW;
SPIClass * hspi = NULL;
char buffer[BUFFER_MAX];         //buffer for serial output
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, fsr;
int32_t lastSampleTime;
//----------------------------------------
//        Handshake Globals
//----------------------------------------
typedef struct handshake_signal {
  int id;
}handshake_signal;

handshake_signal Incoming_ID_check;
int ID;


#include <Arduino.h>

class SinGen {
public:
  SinGen() {
  }

private:
  const double kMillisPerSecond = 1000.0;
  const double kMod = 2.0 * (double)M_PI;

  double mPeriodInSeconds = 0.0;
  double mStartTime = 0.0;

  double millisD() {
    return (double)millis();
  }

public:
// Returns value between -1 and 1 based on elapsed time
  void initialize(float periodInSeconds = 3.0f) {
    mPeriodInSeconds = periodInSeconds;
    mStartTime = millis();
  }

  double getValue(double offset = 0.0) {
    return sin(((millisD() + offset) * kMod) / (kMillisPerSecond * mPeriodInSeconds));
  }
};
SinGen generator;
//----------------------------------------
//          Setup
//----------------------------------------
void setup() {
  //set pin modes
  pinMode(BLUE_LED, OUTPUT);
  // pinMode(GREEN_LED, OUTPUT);
  pinMode(ADC1_PIN, INPUT);
  adcAttachPin(ADC1_PIN);

  generator.initialize(2.0);
  // Init Serial Monitor
  Serial.begin(SERIAL_BAUD_RATE);
  
  
  Serial.println("Setup");
  // //init spi for IMU
  // hspi = new SPIClass(0);
  // hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  // pinMode(hspi->pinSS(), OUTPUT); //HSPI SS

  // spiWrite(hspi, IMU_CONFIG, 0x00);  //set IMU config
  // spiWrite(hspi, PWR_MGMT_1, 0x01);  //power config 1 - turn off sleep set clk sel to 1
  // spiWrite(hspi, PWR_MGMT_2, 0x00);  //enable accel and gyro
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
  
  
  esp_now_register_recv_cb(OnDataRecv);
  //setup timer0 for led at 1Hz
  timerLED = timerBegin(0, 80, true);
  timerAttachInterrupt(timerLED, &TimerIntr0, true);
  timerAlarmWrite(timerLED, 1000000, true);
  timerAlarmEnable(timerLED);

  //setup timer1 for sampling at 100Hz
  timerSample = timerBegin(1, 80, true);
  timerAttachInterrupt(timerSample, &TimerIntr1, true);
  timerAlarmWrite(timerSample, 10000, true);
  timerAlarmEnable(timerSample);
}

void loop() {
  Serial.println("Loop");
  // Set values to send
  myData.id = DEVICE_ID;
  myData.time = lastSampleTime;
  myData.ADC = fsr;
  // Send message via ESP-NOW
  
  Serial.println(ID);
  if(ID = myData.id){
    esp_err_t result = esp_now_send(basestationAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {      
      Serial.println("Sent with success");
    }
    else{
      Serial.println("Error sending the data");
    } 
    Serial.println(generator.getValue());
  }
  Serial.println("Not my Turn");

  delay(50);
}
//----------------------------------------
//          Helper Functions
//----------------------------------------

// callback when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: "); 
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&Incoming_ID_check, incomingData, sizeof(Incoming_ID_check));
  Serial.printf("Board ID %u: %u bytes\n", Incoming_ID_check.id, len);
  ID = Incoming_ID_check.id;
  Serial.printf("ID: %d \n", Incoming_ID_check.id);
  
  Serial.println();
}
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if (status == ESP_NOW_SEND_SUCCESS) {
    connected = 1;
  }
  else{
    connected = 0;
  }

}

uint8_t spiCommand(SPIClass *spi, byte data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(data);
  uint8_t output = spi->transfer(0b00000000);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
  return output;
}

void spiWrite(SPIClass *spi, uint8_t address, uint8_t data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(address);
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
  return;
}

int16_t spiReadSensor(SPIClass *spi, uint8_t address) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(address | 0b10000000);
  uint8_t high_byte = spi->transfer(0b00000000);
  spi->transfer((address+1) | 0b10000000);
  uint8_t low_byte = spi->transfer(0b00000000);
  int16_t output = (high_byte << 8) | low_byte;
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
  return output;
}

//----------------------------------------
//          Timer Interrupt Function
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0() {

  //toggle blue LED
  blueLEDState = !blueLEDState;
  if (connected == 1) {
    digitalWrite(BLUE_LED, blueLEDState);
  }
  else
  {
    digitalWrite(BLUE_LED, 1);
  }
}

void ARDUINO_ISR_ATTR TimerIntr1() {
  //store current time
  lastSampleTime = millis();

  //read accelermeter
  // accX = spiReadSensor(hspi, ACCEL_XOUT_H);
  // accY = spiReadSensor(hspi, ACCEL_YOUT_H);
  // accZ = spiReadSensor(hspi, ACCEL_ZOUT_H);

  // //read gyro
  // gyroX = spiReadSensor(hspi, GYRO_XOUT_H);
  // gyroY = spiReadSensor(hspi, GYRO_YOUT_H);
  // gyroZ = spiReadSensor(hspi, GYRO_ZOUT_H);

  //read force sensing resisitor
  fsr = analogRead(ADC1_PIN);
}