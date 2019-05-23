#include "Adafruit_LiquidCrystal.h"
#include <Wiichuck.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
Wiichuck wii;

void setup() {
  Serial.begin(115200);
  wii.init();
  lcd.begin(16, 2);
  SPI.begin();                            
  SPI.setClockDivider(SPI_CLOCK_DIV8);    
  digitalWrite(SS,HIGH);                  
}

void loop() {
  byte Mastersend,Mastereceive;
  if (wii.poll()) {
    
    int mappedx = map(wii.joyX(), -100, 100, 1, 6);
    Serial.print("X ");
    Serial.print(mappedx);
    int mappedy = map(wii.joyY(), -100, 100, 1, 6);
    Serial.print("  Y ");
    Serial.println(mappedy);
    Mastersend = ((mappedx * 10) + mappedy);
    //Serial.println(Mastersend);
    SPI.transfer(Mastersend);

    lcd.setCursor(13,0);
    lcd.print("C:");
    lcd.print(wii.buttonC());
    lcd.setCursor(13,1);
    lcd.print("Z:");
    lcd.print(wii.buttonZ());

    digitalWrite(SS, LOW);
    
    if(wii.buttonC() == 1){
      digitalWrite(10, HIGH);
      Serial.println("10");
    }
    if(wii.buttonZ() == 1){
      digitalWrite(9, HIGH);
      Serial.println("9");
    }
    if(wii.buttonC() == 0){
      digitalWrite(10, LOW);
    }
    if(wii.buttonZ() == 0){
      digitalWrite(9, LOW);
    }
    
  }
  delay(50);
}
