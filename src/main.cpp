#include <mbed.h>
#include "nRF24L01P.hpp"
#include "TextLCD.h"
#include <string> 

InterruptIn radioInterrupt(D8);
Ticker ticker, screenTicker;
AnalogIn throttlePin(PA_4), rollPin(PA_0), pitchPin(PA_1), yawPin(PA_3), pinBattery(PA_7);
DigitalIn switch1Pin(D3), switch2Pin(D6);
DigitalOut led(LED1);

nRF24L01P radio(D11,D12,D13,D10,D9);

char rxBuffer[10], data[10];
uint8_t status, pos = 0, signalStrengthArray[256], signalStrength, signalStrengthRaw;
uint16_t throttleValue, sum = 0;
int16_t rollValue, pitchValue, yawValue;
bool switch1, switch2, packetReceived;

I2C i2c_lcd(I2C_SDA, I2C_SCL);
TextLCD_I2C lcd(&i2c_lcd, 0x7E, TextLCD::LCD20x4);

// Border ADC values of sticks
const uint16_t throttleMin = 2000;
const uint16_t throttleMax = 56845;
const uint16_t rollMin = 500;
const uint16_t rollMax = 58000;
const uint16_t rollMid = 29100;
const uint16_t pitchMin = 6000;
const uint16_t pitchMax = 63000;
const uint16_t pitchMid = 34100;
const uint16_t yawMin = 2400;
const uint16_t yawMax = 62000;
const uint16_t yawMid = 27350;

// Dead area of minimum and maximum stick values
const uint16_t rangeMarginMin = 1500;
const uint16_t rangeMarginMax = 1500;

int batteryTxCritical = 960; // low battery alert transmitter, 100=1V
bool backlightOn = false;

enum stickType {throttle, roll, pitch, yaw};

Serial pc(USBTX, USBRX); // tx, rx

void startScreen(void)
{
    lcd.setCursor(TextLCD::CurOff_BlkOff);
    lcd.setBacklight(TextLCD::LightOn); backlightOn = true;
    lcd.printf("Battery tx:     V"); //location 12-15 can be used for 4 voltage digits
    lcd.locate ( 0, 1 );         // go to the 2nd line
    lcd.printf("Battery QC:     V"); //location 12-15 can be used for 4 voltage digits
    lcd.locate ( 0, 2 );         // go to the 3rd line
    lcd.printf("T:       R: ");
    lcd.locate ( 0, 3 );         // go to the 4th line
    lcd.printf("P:       Y: ");
}

uint16_t fetchStickValue(stickType stick){

    uint16_t rawValue = 0;
    uint16_t minBorder = 0;
    uint16_t maxBorder = 0;
    uint16_t midValue = 0;

  if (stick==throttle){
    rawValue = throttlePin.read_u16();
    minBorder = throttleMin + rangeMarginMin;
    maxBorder = throttleMax - rangeMarginMax;
    if (rawValue < minBorder) return 65535;  // invert value
    if (rawValue > maxBorder) return 0;     // invert value    
    double slope = 65535.0/((double)(maxBorder-minBorder));
    return 65535-slope*(rawValue-minBorder);
  }
  else if (stick==roll){
    rawValue = rollPin.read_u16();
    midValue = rollMid;
    if (rawValue > (rollMid+rangeMarginMin)){
        minBorder = rollMid + rangeMarginMin;
        maxBorder = rollMax - rangeMarginMax;
    } else if (rawValue < (rollMid-rangeMarginMax)){
        minBorder = rollMin + rangeMarginMin;
        maxBorder = rollMid - rangeMarginMax;
    } else return (65536/2);
  }
  else if (stick==pitch){
    rawValue = pitchPin.read_u16();
    midValue = pitchMid;
    if (rawValue > (pitchMid+rangeMarginMin)){
        minBorder = pitchMid + rangeMarginMin;
        maxBorder = pitchMax - rangeMarginMax;
    } else if (rawValue < (pitchMid-rangeMarginMax)){
        minBorder = pitchMin + rangeMarginMin;
        maxBorder = pitchMid - rangeMarginMax;
    } else return (65536/2);
  }
  else if (stick==yaw){
    rawValue = yawPin.read_u16();
    midValue = yawMid;
    if (rawValue > (yawMid+rangeMarginMin)){
        minBorder = yawMid + rangeMarginMin;
        maxBorder = yawMax - rangeMarginMax;
    } else if (rawValue < (yawMid-rangeMarginMax)){
        minBorder = yawMin + rangeMarginMin;
        maxBorder = yawMid - rangeMarginMax;
    } else return (65536/2);
  } else return 0;
  
  if (rawValue < minBorder) return 65535;  // invert value
  if (rawValue > maxBorder) return 0;     // invert value
  
  double slope = 65536.0/2/((double)(maxBorder-minBorder));
  if (rawValue < midValue) return 65535-slope*(rawValue-minBorder);
  return 65536/2-slope*(rawValue-minBorder);

}

uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}

void interruptHandler(void){
    status = radio.getStatusRegister();
    if (status == 0){ //data not ready?
        while (status == 0 ){
            status = radio.getStatusRegister();
        }
    }

    if (status & 1){ // TX FIFO full
        radio.disable();
        radio.flushTX();
    }
    if (status & 16){ // max TX retransmits
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07,16);
        signalStrengthRaw = 0;
    }
    if (status & 32){ // TX sent (ACK package available if autoAck is enabled)
        radio.disable();
        radio.flushTX();
        radio.setRegister(0x07,32);
        signalStrengthRaw = 100;
    }
    if (status & 64){ // RX received
        radio.read((status & 14) >> 1, &rxBuffer[0],4);
        radio.setRegister(0x07,64);
        packetReceived = true;
    }

    signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), signalStrengthRaw);
    pos++;
}

void mainLoop(void){
    throttleValue = fetchStickValue(throttle);
    rollValue = fetchStickValue(roll)-32768;
    pitchValue = fetchStickValue(pitch)-32768;
    yawValue = fetchStickValue(yaw)-32768;


    // pc.printf("Throttle: %u \t", throttleValue);
    // pc.printf("Roll: %u \t", rollValue);
    // pc.printf("Pitch: %u \t", pitchValue);
    // pc.printf("Yaw: %u \n", yawValue);

    switch1 = switch1Pin.read();
    switch2 = switch2Pin.read();
    // pc.printf("S1: %d \t", switch1);
    // pc.printf("S2: %d \n", switch2);

    data[0] = (throttleValue & 0xFF);
    data[1] = throttleValue >> 8;
    data[2] = (rollValue & 0xFF);
    data[3] = rollValue >> 8;
    data[4] = (pitchValue & 0xFF);
    data[5] = pitchValue >> 8;
    data[6] = (yawValue & 0xFF);
    data[7] = yawValue >> 8;
    data[8] = 0;
    data[9] = 0;
    data[8] = ((switch1 & 1) << 1) | (switch2 & 1);

    radio.write(NRF24L01P_PIPE_P0, &data[0], 10);
}

void screenLoop(void){
  // Display transmitter battery level
  int batteryLevelTx = (int)((pinBattery.read()*3.3f*3.54f*100)*2-920); //in mV
  //pc.printf("Battery: %d.%02d \n", batteryLevelTx/100, batteryLevelTx%100);
  lcd.locate(11,0);
  if (batteryLevelTx<1000) {
    lcd.printf(" %d.%02d", batteryLevelTx/100, batteryLevelTx%100);
  } else {
    lcd.printf("%d.%02d", batteryLevelTx/100, batteryLevelTx%100);
  }

//    // Start blinking screen when battery is low
   if (batteryLevelTx<batteryTxCritical){
    if (backlightOn) {
      lcd.setBacklight(TextLCD::LightOff); backlightOn = false;
      }
    else {
      lcd.setBacklight(TextLCD::LightOn); backlightOn = true;
    } 
  } else if (!backlightOn) {
      lcd.setBacklight(TextLCD::LightOn); backlightOn = true;
  }

// Display drone battery level
  uint16_t batteryLevelQC_uint = (uint16_t)rxBuffer[1] << 8 | rxBuffer[0];
    //pc.printf("rx battery: %u\n", batteryLevelQC_uint);
  int batteryLevelQC = (int) ((float)batteryLevelQC_uint)/65536.0f * 3.3f * 5.854f * 100.0f; //in mV
    //pc.printf("decoded rx battery: %u\n", batteryLevelQC_uint);
    //pc.printf("Battery QC: %d.%02dV \n", batteryLevelQC/100, batteryLevelQC%100);
  lcd.locate(11,1);
  if (batteryLevelQC<1000) {
    lcd.printf(" %d.%02d", batteryLevelQC/100, batteryLevelQC%100);
  } else {
    lcd.printf("%d.%02d", batteryLevelQC/100, batteryLevelQC%100);
  }

  for (int i=0; i<4; i++){
    
    int32_t stickValue = 0;

    //set cursor position
    if (i==0) {
        lcd.locate(2,2); //throttle position
        stickValue = throttleValue;
    }
    else if (i==1) {
        lcd.locate(11,2); //roll position
        stickValue = rollValue;
    }
    else if (i==2){
        lcd.locate(2,3); //pitch position
        stickValue = pitchValue;
    }
    else {
        lcd.locate(11,3); //yaw position
        stickValue = yawValue;
    }

    //set value
    if (stickValue<=-10000){
      lcd.printf("%d", stickValue);
    } else if((stickValue>=10000)||(stickValue<=-1000)) {
      lcd.printf("%d ", stickValue);    
    } else if((stickValue>=1000)||(stickValue<=-100)) {
      lcd.printf("%d  ", stickValue);
    } else if(stickValue>=100) {
      lcd.printf(" %d ", stickValue);
    } else {
      lcd.printf("  %d   ", stickValue);
    }
  }
}

int main() {
    radio.powerUp();
    radio.setRfFrequency(2400 + 101);
    radio.setTransferSize(10);
    radio.setCrcWidth(16);
    radio.setTxAddress(0x007FFFFFFF);
    radio.setRxAddress(0x007FFFFFFF);
    radio.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
    radio.setAirDataRate(NRF24L01P_DATARATE_250_KBPS);
    radio.enableAutoRetransmit(500, 3);
    radio.setTransmitMode();
    radioInterrupt.fall(&interruptHandler);
    radioInterrupt.enable_irq();
    
    startScreen();

    
    Timer screenTimer = Timer();
    Timer mainTimer = Timer();
    
    screenTimer.start();
    mainTimer.start();

    uint32_t period_us_main = 10001;
    uint32_t period_us_screen = 100000;

    while(1) 
    {
        if(screenTimer.read_us()>period_us_screen)
        {
            screenLoop();
            screenTimer.reset();
        }

        if(mainTimer.read_us()>period_us_main)
        {
            mainLoop();
            mainTimer.reset();
        }
    }
}