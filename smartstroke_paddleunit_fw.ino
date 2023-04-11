//----------------------------------------
//          Includes
//----------------------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "SPI.h"

//----------------------------------------
//          Definitions
//----------------------------------------
#define BLUE_LED  16
#define GREEN_LED 9
#define ADC1_PIN  4
#define ADC2_PIN  5

#define BUFFER_MAX        512
#define SERIAL_BAUD_RATE  115200

#define HSPI_MISO   13
#define HSPI_MOSI   11
#define HSPI_SCLK   12
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
hw_timer_t* timerLED = NULL;  //timer for sampling 100Hz
int blueLEDState = LOW;
SPIClass * hspi = NULL;
char buffer[BUFFER_MAX];         //buffer for serial output
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, fsr, lastSampleTime;

//----------------------------------------
//          Setup
//----------------------------------------
void setup() {
  //set pin modes
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(ADC1_PIN, INPUT);
  adcAttachPin(ADC1_PIN);
  
  // Init Serial Monitor
  Serial.begin(SERIAL_BAUD_RATE);

  //init spi for IMU
  hspi = new SPIClass(HSPI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  pinMode(hspi->pinSS(), OUTPUT); //HSPI SS

  spiWrite(hspi, IMU_CONFIG, 0x00);  //set IMU config
  spiWrite(hspi, PWR_MGMT_1, 0x01);  //power config 1 - turn off sleep set clk sel to 1
  spiWrite(hspi, PWR_MGMT_2, 0x00);  //enable accel and gyro

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

//----------------------------------------
//          Main Loop
//----------------------------------------
void loop() {
  // Set values to send
  myData.id = 1;
  myData.x = 10;
  myData.y = 20;
  
  digitalWrite(GREEN_LED, HIGH);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(basestationAddress, (uint8_t *) &myData, sizeof(myData));
  digitalWrite(GREEN_LED, LOW);

  if (result == ESP_OK) {      
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  //sprintf(buffer, "X:%d Y:%d Z:%d\r\n", accX, accY, accZ);
  // sprintf(buffer, "X:%d Y:%d Z:%d\r\n", int16_t(spiReadSensor(hspi, GYRO_XOUT_H)), int16_t(spiReadSensor(hspi, GYRO_YOUT_H)), int16_t(spiReadSensor(hspi, GYRO_ZOUT_H)));
  //Serial.print(buffer);

  delay(10);
}

//----------------------------------------
//          Helper Functions
//----------------------------------------
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  digitalWrite(BLUE_LED, blueLEDState);
}

void ARDUINO_ISR_ATTR TimerIntr1() {
  //store current time
  lastSampleTime = millis();

  //read accelermeter
  accX = spiReadSensor(hspi, ACCEL_XOUT_H);
  accY = spiReadSensor(hspi, ACCEL_YOUT_H);
  accZ = spiReadSensor(hspi, ACCEL_ZOUT_H);

  //read gyro
  gyroX = spiReadSensor(hspi, GYRO_XOUT_H);
  gyroY = spiReadSensor(hspi, GYRO_YOUT_H);
  gyroZ = spiReadSensor(hspi, GYRO_ZOUT_H);

  //read force sensing resisitor
  fsr = analogRead(ADC1_PIN);
}