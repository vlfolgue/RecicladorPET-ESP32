

#include <LiquidCrystal_I2C.h>
#define Potenciometro_velocidad 35

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display


int velocidad;
void setup()
{
	lcd.init(); // initialize the lcd to use user defined I2C pins
	lcd.backlight();
	lcd.setCursor(3,0);
	lcd.print("Hello, world!");
}


void loop()
{
velocidad = analogRead(Potenciometro_velocidad);
lcd.setCursor(3,1);
lcd.print("    ");
lcd.setCursor(3,1);
lcd.print(velocidad);

delay(100);
}