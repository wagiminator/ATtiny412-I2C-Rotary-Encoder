#include <Wire.h>

#define encoder_addr 0x36

int16_t value, lastvalue;
boolean pressed, lastpressed;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  encoder_set(-50, 50, 1, 0, 0);
}

void loop() {
  value = encoder_getValue();
  if(value != lastvalue) {
    Serial.println(value);
    lastvalue = value;
  }

  pressed = encoder_isPressed();
  if(pressed != lastpressed) {
    if(pressed) Serial.println("Switch was pressed");
    lastpressed = pressed;
  }

  delay(20);
}

// Set encoder wheel parameters
void encoder_set(int16_t rmin, int16_t rmax, int16_t rstep, int16_t rval, uint8_t rloop) {
  Wire.beginTransmission(encoder_addr);
  Wire.write((uint8_t)(rval & 0xff)); Wire.write((uint8_t)(rval >> 8));
  Wire.write(0); Wire.write(rloop);
  Wire.write((uint8_t)(rmin & 0xff)); Wire.write((uint8_t)(rmin >> 8));
  Wire.write((uint8_t)(rmax & 0xff)); Wire.write((uint8_t)(rmax >> 8));
  Wire.write((uint8_t)(rstep & 0xff)); Wire.write((uint8_t)(rstep >> 8));
  Wire.endTransmission();
}

// Set encoder wheel value
void encoder_setValue(int16_t rval) {
  Wire.beginTransmission(encoder_addr);
  Wire.write((uint8_t)(rval & 0xff)); Wire.write((uint8_t)(rval >> 8));
  Wire.endTransmission();
}

// Read encoder wheel value
int16_t encoder_getValue() {
  Wire.requestFrom(encoder_addr, 2);
  return((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8));
}

// Read encoder switch state
boolean encoder_isPressed() {
  Wire.requestFrom(encoder_addr, 3);
  Wire.read(); Wire.read();
  return(Wire.read());
}
