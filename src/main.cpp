
// Library version:1.1.5
// Example of picking custom I2C pins on ESP32

#include <Arduino.h>
#include "Wire.h"
#include <LiquidCrystal_I2C.h>

#define I2C_SDA 21
#define I2C_SCL 22
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

const int portPin = 35;
// Almacenamiento del valor de puerto (Rango de 0-4095)
int potValor = 0;


void setup() {
  //Serial.begin(115200);
  delay(1000);

	lcd.init(I2C_SDA, I2C_SCL); // initialize the lcd to use user defined I2C pins
	lcd.backlight();
	lcd.setCursor(3,0);
	lcd.print("Hello, world!");
	lcd.setCursor(2,1);
	lcd.print("Time is now");



}
void loop() {
  // Lectura del valor en cada vuelta del bucle
  potValor = analogRead(portPin);
  Serial.println(potValor);  //Envío del valor al puerto serie
  delay(1000);
}
