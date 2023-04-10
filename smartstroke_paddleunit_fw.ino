//----------------------------------------
//          Definitions
//----------------------------------------
#define BLUE_LED  16
#define GREEN_LED 9
#define ADC1_PIN  4
#define ADC2_PIN  5

#define BUFFER_MAX 512

//----------------------------------------
//          Prototype Functions
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0();
void ARDUINO_ISR_ATTR TimerIntr1();
void flushSerialInput();

//----------------------------------------
//          Globals
//----------------------------------------
int fsr1 = 0;  // holds force sensing resisitor current adc value
int fsr2 = 0;
int time2 = 0;
char buffer[BUFFER_MAX];         //buffer for serial output
hw_timer_t* timerSample = NULL;  //timer for sampling 100Hz
hw_timer_t* timerLED = NULL;     //timer for led 1Hz
int blueLEDState = LOW;
int greenLEDState = LOW;

//----------------------------------------
//          Setup
//----------------------------------------
void setup() {
  //set pin modes
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(ADC1_PIN, INPUT);
  pinMode(ADC2_PIN, INPUT);
  adcAttachPin(ADC1_PIN);
  adcAttachPin(ADC2_PIN);

  //init serial
  Serial.begin(921600);

  //setup timer0 for sampling at 100Hz
  timerSample = timerBegin(0, 80, true);
  timerAttachInterrupt(timerSample, &TimerIntr0, true);
  timerAlarmWrite(timerSample, 10000, true);
  timerAlarmEnable(timerSample);

  //setup timer1 for led at 1Hz
  timerLED = timerBegin(1, 80, true);
  timerAttachInterrupt(timerLED, &TimerIntr1, true);
  timerAlarmWrite(timerLED, 1000000, true);
  timerAlarmEnable(timerLED);
}

//----------------------------------------
//          Main Loop
//----------------------------------------
void loop() {
  //wait for input from matlab
  if (!Serial.available()) {
    return;
  }
  flushSerialInput();

  //send "TIME(ms) FSR1 FSR2\r\n" over serial output
  sprintf(buffer, "%d %d %d\r\n", time2, fsr1, fsr2);
  Serial.print(buffer);
}

//----------------------------------------
//          Helper Functions
//----------------------------------------
void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

//----------------------------------------
//          Timer Interrupt Function
//----------------------------------------
void ARDUINO_ISR_ATTR TimerIntr0() {
  //read analog values every 10ms/100Hz
  fsr1 = analogRead(ADC1_PIN);
  fsr2 = analogRead(ADC2_PIN);
  time2 = (unsigned long)millis();

  //toggle blue LED
  blueLEDState = !blueLEDState;
  digitalWrite(BLUE_LED, blueLEDState);
}

void ARDUINO_ISR_ATTR TimerIntr1() {
  //toggle green LED
  greenLEDState = !greenLEDState;
  digitalWrite(GREEN_LED, greenLEDState);
}
