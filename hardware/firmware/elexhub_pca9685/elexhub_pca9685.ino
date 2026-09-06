// arm-fw 3.0: PCA9685 output controller, NOT joint-position feedback.
// VCC = logic only. External regulated servo rail + physical cutoff required.
// OE HIGH disables PWM; it DOES NOT disconnect servo power.
#include <Wire.h>
#include <stdlib.h>
#include <math.h>
#include "arm_config.h"

float currentRaw[NSERVOS], targetRaw[NSERVOS];
bool enabled = false;
const char* fault = "none";
int8_t commissioning = -1;
unsigned long lastMotion = 0, lastTick = 0;
char buffer[256]; uint16_t used = 0;
bool discardLine = false;

void disablePWM() {
  digitalWrite(OE_PIN, HIGH);
  enabled = false;
  commissioning = -1;
  for (uint8_t i=0;i<NSERVOS;i++) targetRaw[i]=currentRaw[i];
}
void fail(const char* reason) {
  disablePWM(); fault=reason;
  Serial.print("! "); Serial.println(reason);
}
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDRESS); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool writeChannel(uint8_t channel, uint16_t ticks, bool off=false) {
  Wire.beginTransmission(PCA_ADDRESS); Wire.write(0x06 + 4*channel);
  Wire.write(0); Wire.write(0); Wire.write(ticks & 0xFF);
  Wire.write(off ? 0x10 : ((ticks >> 8) & 0x0F));
  return Wire.endTransmission() == 0;
}
bool configurePCA() {
  if (!writeReg(0x00,0x10) || !writeReg(0xFE,121) || !writeReg(0x01,0x04)) return false;
  if (!writeReg(0x00,0x20)) return false;
  delayMicroseconds(600);
  for(uint8_t c=0;c<16;c++) if(!writeChannel(c,0,true))return false;
  return true;
}
bool output(uint8_t i) {
  if (PULSE_MIN[i] < 400 || PULSE_MAX[i] > 2600 || PULSE_MIN[i] >= PULSE_MAX[i]) return false;
  const float span = MAX_RAW[i] - MIN_RAW[i];
  if(span <= 0 || currentRaw[i] < MIN_RAW[i] || currentRaw[i] > MAX_RAW[i])return false;
  float us = PULSE_MIN[i] + (currentRaw[i]-MIN_RAW[i])/span*(PULSE_MAX[i]-PULSE_MIN[i]);
  return writeChannel(CHANNEL[i],(uint16_t)lround(us*4096.0f*50.0f/1000000.0f));
}
void state() {
  Serial.print('s');
  for(uint8_t i=0;i<NSERVOS;i++){Serial.print(' ');Serial.print(currentRaw[i],3);}
  Serial.print(' ');Serial.print(millis());Serial.print(' ');Serial.print(enabled?1:0);
  Serial.print(' ');Serial.println(fault);
}
bool number(char* token, float& value) {
  if(!token || !*token)return false;
  char* end; double result=strtod(token,&end);
  if(*end || !isfinite(result))return false;
  value=(float)result;return true;
}
bool enableAll() {
  disablePWM();
  if(!configurePCA())return false;
  for(uint8_t i=0;i<NSERVOS;i++)if(!output(i))return false;
  fault="none";enabled=true;lastMotion=millis();digitalWrite(OE_PIN,LOW);return true;
}
void handle(char* line) {
  if(!*line)return;
  char* cmd=strtok(line," ");
  if(!cmd)return;
  if(!strcmp(cmd,"P") && !strtok(NULL," ")) {
    Serial.print("# pong arm-fw 3.0 joints=");Serial.print(NJOINTS);
    Serial.print(" backend=pca9685 profile=");Serial.print(PROFILE_HASH);Serial.println(" runtime=hardware");return;
  }
  if(!strcmp(cmd,"Q") && !strtok(NULL," ")) { state();return; } // read-only: no watchdog refresh
  if(!strcmp(cmd,"X") && !strtok(NULL," ")) { disablePWM();state();return; }
  if(!strcmp(cmd,"E") && !strtok(NULL," ")) {
    if(!ARM_CALIBRATED)fail("not_calibrated");
    else if(!enableAll())fail("i2c_or_pulse_configuration");
    state();return;
  }
  if(!strcmp(cmd,"C")) {
    float index,value;
    bool parsed=number(strtok(NULL," "),index) && number(strtok(NULL," "),value) && !strtok(NULL," ");
    if(!COMMISSIONING_ALLOWED || ARM_CALIBRATED)fail("commissioning_disabled");
    else if(!parsed || index<0 || index>=NSERVOS || index!=floor(index))fail("bad_command");
    else {
      uint8_t i=(uint8_t)index;
      if(value<COMMISSION_MIN[i] || value>COMMISSION_MAX[i] || COMMISSION_MAX[i]-COMMISSION_MIN[i]>4)fail("commission_limit");
      else if(strcmp(fault,"none"))fail("fault_latched");
      else if(commissioning!=i) {
        disablePWM();
        if(!configurePCA() || !output(i))fail("i2c_error");
        else {commissioning=i;enabled=true;targetRaw[i]=value;lastMotion=millis();digitalWrite(OE_PIN,LOW);}
      } else {targetRaw[i]=value;lastMotion=millis();}
    }
    state();return;
  }
  if(!strcmp(cmd,"K") && !strtok(NULL," ")) {
    if(strcmp(fault,"none"))fail("fault_latched");
    else if(enabled) {for(uint8_t i=0;i<NSERVOS;i++)targetRaw[i]=currentRaw[i];lastMotion=millis();}
    state();return;
  }
  if(!strcmp(cmd,"S") || !strcmp(cmd,"H")) {
    float values[NSERVOS];bool parsed=true;
    if(!strcmp(cmd,"H")) {parsed=!strtok(NULL," ");for(uint8_t i=0;i<NSERVOS;i++)values[i]=HOME_RAW[i];}
    else {for(uint8_t i=0;i<NSERVOS;i++)if(!number(strtok(NULL," "),values[i]))parsed=false;if(strtok(NULL," "))parsed=false;}
    if(!ARM_CALIBRATED)fail("not_calibrated");
    else if(!enabled || commissioning!=-1 || strcmp(fault,"none"))fail("not_enabled_or_fault_latched");
    else if(!parsed)fail("bad_command");
    else {
      for(uint8_t i=0;i<NSERVOS;i++)if(values[i]<MIN_RAW[i] || values[i]>MAX_RAW[i])parsed=false;
      if(!parsed)fail("unsafe_command");
      else {for(uint8_t i=0;i<NSERVOS;i++)targetRaw[i]=values[i];lastMotion=millis();}
    }
    state();return;
  }
  fail("bad_command");state();
}
void setup() {
  digitalWrite(OE_PIN,HIGH);pinMode(OE_PIN,OUTPUT);
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000,true);
#endif
  for(uint8_t i=0;i<NSERVOS;i++)currentRaw[i]=targetRaw[i]=HOME_RAW[i];
  if(!configurePCA())fault="i2c_error";
  lastTick=millis();Serial.println("# ready arm-fw 3.0; PWM disabled");
}
void loop() {
  if(enabled && millis()-lastMotion>COMMAND_TIMEOUT_MS) {fail("watchdog");}
  uint8_t consumed=0;
  while(Serial.available() && consumed++<128) {
    char c=Serial.read();
    if(discardLine) {if(c=='\n' || c=='\r')discardLine=false;continue;}
    if(c=='\n' || c=='\r') {if(used){buffer[used]='\0';handle(buffer);used=0;}}
    else if(used<sizeof(buffer)-1)buffer[used++]=c;
    else {used=0;discardLine=true;fail("line_overflow");}
  }
  unsigned long now=millis();
  if(now-lastTick>=TICK_MS) {
    lastTick=now;
    if(enabled)for(uint8_t i=0;i<NSERVOS;i++) {
      if(commissioning!=-1 && commissioning!=i)continue;
      float step=MAX_RATE[i]*TICK_MS/1000.0f;
      currentRaw[i]+=constrain(targetRaw[i]-currentRaw[i],-step,step);
      if(!output(i)){fail("i2c_error");break;}
    }
  }
}
