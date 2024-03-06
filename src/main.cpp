#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
AccelStepper motor1(1, 27, 14);

#define Potenciometro_velocidad 35

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display


int velocidad;
void setup()
{
	lcd.init(); // initialize the lcd to use user defined I2C pins
	lcd.backlight();
	lcd.setCursor(3,0);
	lcd.print("Hello, world!");

		motor1.setMaxSpeed(5000); //Definimos velocidad maxima motor 1
		motor1.setSpeed(-100);  
}


void loop()
{
velocidad = analogRead(Potenciometro_velocidad);
lcd.setCursor(3,1);
lcd.print("    ");
lcd.setCursor(3,1);
lcd.print(velocidad);

   motor1.runSpeed();

delay(100);
}