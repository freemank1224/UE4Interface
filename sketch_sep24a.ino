#include <arduino-timer.h>

#define KNOB A0
#define SW_MASTER A2
#define SW_SLAVE A3 

#define NUM_WT 21
#define NUM_MODE 3


const uint16_t RPM_MAX = 45;
const uint16_t RPM_MIN = 5;
const uint16_t PITCH_MAX = 60;
const uint16_t PITCH_MIN = 0;

const uint8_t PINOUT_rotorMotorDIR = 10;
const uint8_t PINOUT_rotorMotorPUL = 11;
const uint8_t PINOUT_nacelleMotorDIR = 8;
const uint8_t PINOUT_nacelleMotorPUL = 9;
// const uint8_t PININ_SwitchMaster = 16;
// const uint8_t PININ_SwitchSlave = 17;
float NACELLE_yawRate; // Unit degree per second

uint8_t keyNotPressed = 0;
uint8_t keyNotPressed2 = 0;
uint16_t MasterIdx, SlaveIdx;
char propertyCmd;
uint16_t valueCmd, fluctCmd;
uint32_t rotorFrequencySetpoint;
uint32_t nacelleFrequencySetpoint;
uint32_t currentNacelleAngle;
uint32_t nacelleStepSetpoint; // Angles in the unit of micro step
uint32_t angleCounter, angleTarget;
uint32_t tickCounter, tickTarget; // MUST use ticks in timer



String FLAG_DIR;

///////////// Generate Timer ////////////////////
Timer<1, micros> rotor_timer;
Timer<1, micros> nacelle_timer;

//////////////////  Timer Tasks  ///////////////// 
bool toggle_led(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  return true; // keep timer active? true
}

bool move_motor(void *)
{
  digitalWrite(PINOUT_rotorMotorPUL, !digitalRead(PINOUT_rotorMotorPUL));
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // Serial.println(digitalRead(PINOUT_rotorMotorPUL));
  return true;
}

bool move_motor_count(void *)
{
  tickCounter ++;
  digitalWrite(PINOUT_nacelleMotorPUL, !digitalRead(PINOUT_nacelleMotorPUL));

  if(tickCounter < tickTarget){
    // Serial.println("...........");
    // Serial.println(tickCounter);
    // Serial.println("...........");
    // Serial.println(tickTarget);    
    return true;
  }else{
    tickCounter = 0;
    currentNacelleAngle = angleTarget;
    return false;
  }
}


bool R2U(void *)
{
  sendCommand(2, 'r', 100, 2);
  return true;
}
/////////////////////////////////////////////////////////////

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // set LED pin to OUTPUT
  pinMode(PINOUT_rotorMotorDIR, OUTPUT);
  pinMode(PINOUT_rotorMotorPUL, OUTPUT);
  pinMode(PINOUT_nacelleMotorDIR, OUTPUT);
  pinMode(PINOUT_nacelleMotorPUL, OUTPUT);  
  pinMode(SW_MASTER, INPUT_PULLUP);
  pinMode(SW_SLAVE, INPUT_PULLUP);
  pinMode(KNOB, INPUT);

  Serial.begin(115200);
  MasterIdx = 0;
  SlaveIdx = 0;
  propertyCmd = 'R';
  valueCmd = 6;
  if(propertyCmd == 'N') fluctCmd = 0;
  if(propertyCmd == 'R') fluctCmd = 2;
  if(propertyCmd == 'P') fluctCmd = 0;

  rotorFrequencySetpoint = 1000;
  nacelleFrequencySetpoint = 1000;
  currentNacelleAngle = 0;
  NACELLE_yawRate = 0.7;

  // call the toggle_led function every 1000 millis (1 second)
  rotor_timer.every(1000, move_motor, PINOUT_rotorMotorPUL);
  // nacelle_timer.every(1000, move_motor_count);
}

void loop() {
  uint16_t tempValue;

  rotor_timer.tick();
  nacelle_timer.tick();
  
  /********** SLAVE KEY : TOGGLE PROPERTYS *******/
  if(digitalRead(SW_SLAVE) == 0 && keyNotPressed == 1)
  {
    delay(10);
    if(digitalRead(SW_SLAVE) == 0 && keyNotPressed == 1)
    {
      SlaveIdx ++;
      if(SlaveIdx == NUM_MODE) SlaveIdx = 0;
      // Serial.println(SlaveIdx);

      if(SlaveIdx == 0) propertyCmd = 'R';
      if(SlaveIdx == 1) propertyCmd = 'N';
      if(SlaveIdx == 2) propertyCmd = 'P';

      // Serial.println(propertyCmd);

      keyNotPressed = 0;
    }
  }else if(digitalRead(SW_SLAVE) == 1) keyNotPressed = 1;

  /********** MASTER KEY : TOGGLE WTs and Send Messages *******/
  if(digitalRead(SW_MASTER) == 0 && keyNotPressed2 == 1)
  {
    delay(10);
    if(digitalRead(SW_MASTER) == 0 && keyNotPressed2 == 1)
    {
      MasterIdx ++;
      if(MasterIdx > NUM_WT) MasterIdx = 0;

      for(int i=0; i<5; ++i)
      {
        valueCmd += analogRead(KNOB);
      }

      valueCmd /= 5;

      // Serial.println(propertyCmd);

      if(propertyCmd == 'R')
      {
        tempValue = reScale(valueCmd, RPM_MIN, RPM_MAX);
        valueCmd = tempValue;

        // To UNREAL 
        sendCommand(MasterIdx, propertyCmd, valueCmd, fluctCmd);
        // To MOTOR --> ROTOR
        digitalWrite(PINOUT_rotorMotorDIR, HIGH);
        rotorFrequencySetpoint = setFrequencyRotor(valueCmd, 6);
        rotor_timer.every(rotorFrequencySetpoint, move_motor);
        // Serial.println("Rotor TIMER Setted!");
      }

      if(propertyCmd == 'N')
      {
        tempValue = reScale(valueCmd, 0, 360);
        valueCmd = tempValue;

        // To UNREAL 
        sendCommand(MasterIdx, propertyCmd, valueCmd, fluctCmd);
        // To MOTOR --> NACELLE
        angleTarget = valueCmd;
        if(abs((int16_t)angleTarget - (int16_t)currentNacelleAngle) > 180){
          digitalWrite(PINOUT_nacelleMotorDIR, HIGH);
        }else{
          digitalWrite(PINOUT_nacelleMotorDIR, LOW);
        }

        // Calculate equivalent ticks using angle error: target - current
        nacelleStepSetpoint = getPulseTicks(((int16_t)angleTarget - (int16_t)currentNacelleAngle), 6);
        tickTarget = nacelleStepSetpoint;
        // Calculate frequency
        nacelleFrequencySetpoint = setFrequencyNacelle(NACELLE_yawRate, 6);
        // Serial.println("~~~~~~~~~~~~~");
        // Serial.println(nacelleFrequencySetpoint);
        // Serial.println("~~~~~~~~~~~~~");

        nacelle_timer.every(nacelleFrequencySetpoint, move_motor_count);

        // Serial.println("******************");
        // Serial.println(currentNacelleAngle);
        // Serial.println("******************");
      }


      if(propertyCmd == 'P')
      {
        valueCmd = reScale(valueCmd, PITCH_MIN, PITCH_MAX);
        valueCmd = tempValue;

        // To UNREAL 
        sendCommand(MasterIdx, propertyCmd, valueCmd, fluctCmd);
      } 






      keyNotPressed2 = 0;

    }
  }else if(digitalRead(SW_MASTER) == 1) keyNotPressed2 = 1;




}

/*********** HAVE TO PUT IN A TIMER **********/
/******** HERE FREQ IS CONTROLLED ************/
void rotateMotor(uint8_t outputPin)
{
  digitalWrite(outputPin, !digitalRead(outputPin));
}

/***** Calculate timer frequency for ROTOR *****\
******************* S1  S2  S3 ******************
************* 1 *** ON  ON  ON ******************
************* 2 *** ON  ON  OFF *****************
************* 4 *** ON  OFF  ON *****************
************* 8 *** ON  OFF  OFF ****************
*************16 *** OFF  ON  ON *****************
************ 32 *** OFF  ON  OFF ****************
************ 64 *** OFF  OFF  ON ****************
************128 *** OFF  OFF  OFF ***************
****** Use microStepTicks 0 ~ 7 to represent ****/
uint32_t setFrequencyRotor(uint32_t targetRPM, uint16_t microStepTicks)
{
  uint16_t denominator;
  uint32_t frequencySetpoint;
  float temp;

  denominator = 1 << microStepTicks;  
  // the equvalent number of FULL steps per second corresponding to targetRPM 
  temp = (float)targetRPM * 6 / 1.8;  
  // the equvalent micro step number * 2 = number of pulses | TWO ticks makes ONE pulse
  frequencySetpoint = 1000000 / (temp * denominator * 2);

  return frequencySetpoint;
}

/************* Calculate pulse number for NACELLE **********\
********** FIXED rotation rate : 1 degree/second ************
\***********************************************************/
uint16_t getPulseTicks(int16_t deltaAngle, uint16_t microStepTicks)
{
  float temp;
  temp = (float)(abs(deltaAngle)) * (1 << microStepTicks) / 1.8;
  return (uint16_t)temp;
}

/************ Calculate angel changing rate for nacelle **********/
uint32_t setFrequencyNacelle(float angleRate, uint16_t microStepTicks)
{
  uint32_t frequencySetpoint;
  float temp;
  // the equvalent number of micro step/sec corresponding to the given angleRate
  temp = angleRate * (1 << microStepTicks) / 1.8;
  // the equvalent micro step number * 2 = number of pulses | TWO ticks makes ONE pulse
  frequencySetpoint = 1000000 / (temp * 2);

  return frequencySetpoint;
}
/************* END OF NACELLE *****************************/

void sendCommand(uint8_t ID, char Property, uint16_t Value, int16_t Noise)
{
  String setStr = "";
  Value += Noise / 2 + random(Noise);

  if(Property == 'N')
  {
    if(Value < 0) Value = 0;
    if(Value > 360) Value = 360;
  }

  if(Property == 'R')
  {
    if(Value > RPM_MAX) Value = RPM_MAX;
    if(Value < RPM_MIN) Value = RPM_MIN;
  }

  if(Property == 'P')
  {
    if(Value > PITCH_MAX) Value = PITCH_MAX;
    if(Value < PITCH_MIN) Value = PITCH_MIN;
  }

  setStr += ID;
  setStr += ',';
  setStr += Property;
  setStr += ',';
  setStr += Value;

  Serial.println(setStr);

}

uint8_t receiveCommand(void)
{
  String message;
  int16_t commaIdx;

  if(Serial.available())
  {
    while(Serial.available()) message += Serial.read();
    delay(1);
  }

  do{
    commaIdx = message.indexOf(',');
    if(commaIdx != -1)
    {
      FLAG_DIR = message.substring(0, commaIdx);
      FLAG_DIR.toInt();
      message = message.substring(commaIdx+1, message.length());
    }else{
      if(message.length() > 0) Serial.println(message);
    }
  }while(commaIdx >= 0);

  if(message.length() > 0) Serial.println(message);
  message = "";
}

uint16_t reScale(uint16_t ADvalue, uint16_t MIN, uint16_t MAX)
{
  uint32_t outputValue = 0;
  if(MAX > MIN) outputValue = ((uint32_t)ADvalue * (MAX - MIN)) / 1024;
  if(MAX < MIN) outputValue = ((uint32_t)ADvalue * (MIN - MAX)) / 1024;

  // Serial.print(outputValue);
  // Serial.print("||");
  // Serial.println(ADvalue);

  return (uint16_t)outputValue;
}

