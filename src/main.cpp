//Reciclador PET ESP 32 --> NO freeRTOS

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <QuickPID.h>

//--> POTENCIOMETRO VELOCIDAD
#define Potenciometro_velocidad 35
int velocidad_raw; // Variable para almacenar la elctura analogica del potenciometro de seleccion de velocidad
int velocidad; // Variable para mapear el valor de seleccion de velocidad

//--> POTENCIOMETRO TEMPERATURA
#define Potenciometro_temperatura 32
int setpoint_temperatura_raw; //Variable para almacenar la lectura analogica del potenciometro de ajuste de setpoint de temperatura
int setpoint_mapped; //variable para mapear el valor del setpoint a partir de la elctura del potenciometro de temperatura.

//-->LCD
LiquidCrystal_I2C lcd(0x27, 20, 4); // Definicion de la LCD (direccion, columnas, filas)
int refresco_linea = 0; //Variable para refrescar la LCD por lineas

//-->MOTOR NEMA 17
const int dirPin = 14; //Pin donde cableamos la DIR del dvr2588
const int stepPin = 27; //Pin donde cableamos el STEP del dvr2588
int enPin = 26; //Pin donde cableamos el ENABLE del dvr2588
#define motorInterfaceType 1 //Interface del motor. De accelstepper library. 1 para motor Step driver con Step, Dir. ver: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin); //Parámetros del motor ( Interface, pin de Step, pin de direccion)

//--> TERMISTOR
#define pinTermistor 25   //Pin Analógico al que se conecta el termistor
#define Termistor_resistencia_nominal 100000  //Resistencia del Termistor a temperatura nominal
#define Termistor_temperatura_nominal 25   // Temperatura nominal del termistor
#define NumMuestras 5 // Numero de lecturas que haremos al termistor para hacer una media-
#define Termistor_coeficiente_B 3950 // Coeficiente Beta del termistor. Del fabricante
#define Resistencia_serie_termistor 9980  // Valor en Ohm de la resistencia colocada en serie al termistor.  
int muestras[NumMuestras]; // Variable para almacenar el numero de muestras de lectuira del termistor.
float media_lecturas_temperatura; // variable para almacenar la media de lecturas del termistor para filtrar.
float temperatura; //Variable para almacenar el valor de la temperatura en los calculos en K.
float temperatura_final; //Variable para almacenar el valor de temperatura tras los calculos en C.

//--> MOSFET
#define pinMosfet 33 // Definimos el pin al que va conectada la señal del MOSFET

//--> PID
float Setpoint, Input, Output; // Definimos las variables para uso del PID
float Kp = 0.5, Ki = 0.2, Kd = 0.1; // Definimos las variables para las ganancias del PID
int porcentaje_PWM; // Variable apra convertir PWM en porcentaje
QuickPID myPID(&Input, &Output, &Setpoint); //Asignamos variables a los aprametros del PID

//--> Control de tiempos
unsigned long tiempo_ultimo_refresh_LCD;
unsigned long tiempo_debounce;

unsigned long tiempo_inicio; //timestamp para definir momento de arranque del motor por primer avez en millis
unsigned long tiempo_parada; //Variable para definir el momento en que se para el motor en cada ciclo de parada
unsigned long tiempo_reanudacion = 1; //Variable para definir el momento de reanudacion despuesd e una parada
unsigned long tiempo_parado; //Variable para calcular para cada parada, cuando tiempo paradao ha estado.
unsigned long milis_totales_parado; // Variable para almacenar el tiempo total de parada como suma de los tiempos totales que ha estado parado
float minutos; // variable con coma para definir los minutos de funcionamiento.

//--> PULSADOR
#define pulsadorPin 13 // Definimos el pin de lectura del pulsador
volatile int pulsador_cuenta = 0; //Variable apra contar las veces pulsado.
void IRAM_ATTR contarpulsador()
{
  if (millis() > tiempo_debounce + 200) // Antirebote para el pulsador de 200milis
  {
    pulsador_cuenta++; // Cada vez que se pulse el pulsador suma un valor
    tiempo_debounce = millis();   
  }
    if(pulsador_cuenta >= 5) // Cuando el pulsador llegue o supere 5 que pase a 0 otra vez. Definimos asi 5 estados.
    {
     pulsador_cuenta = 0;
    }
}
// variables para definir el programa a usar en cada momento en funcion del pulsador
bool motor = false; // Variable para activar/desacrivar el motor
bool calentador = false; // variable apra activar/desactivar el calentador
int direccion = 1; //variable para definir direccion de giro del motor
String estado; //Variable para definir estado en la LCD

void setup()
{

//-->LCD    
lcd.init(); //Iniciamos la LCD segun los pines definidos antes
lcd.backlight(); // Damos luz a la LCD
lcd.setCursor(0,0); //Definimos la posicion del cursos apra imprimir en la LCD (Columna, Fila)
lcd.print("Reciclador PET"); //Imprimimos en la posicion del cursor

    lcd.setCursor(0,1);
    lcd.print("Velo:");
    lcd.setCursor(10,1);
    lcd.print("min:");
    lcd.setCursor(0,2);
    lcd.print("Temp:");
    lcd.setCursor(11,2);
    lcd.print("SP:");
    lcd.setCursor(10,3);
    lcd.print("P:");
    lcd.setCursor(7,3);
    lcd.print("%");
    lcd.setCursor(0,3);
    lcd.print("PWM:");


//-->NEMA 17
pinMode( enPin ,OUTPUT); //Definimos el pin del ENABLE del motor NEMA17 como salida
digitalWrite( enPin , LOW); //Habilitamos salida el funcionamiento del motor NEMA17
myStepper.setMaxSpeed(1000); //Definimos la velocidad maxima que peude alcanzar el motor
myStepper.setAcceleration(100.0); // Definimos la acceleracion del motor


//--> MOSFET
pinMode(pinMosfet, OUTPUT); // Pin de salida del Mosfet del calentador.

//--> PID
myPID.SetTunings(Kp, Ki, Kd); //Applicamos las ganancias a lso valores P, I & D
myPID.SetMode(myPID.Control::automatic); //Iniciamos el PID

//--> PULSADOR
pinMode(pulsadorPin, INPUT); // Defino pulsador como entrada.
attachInterrupt(digitalPinToInterrupt(pulsadorPin), contarpulsador, FALLING); // Interrupcion en el pin del pulsador.

delay (500);
}

/* Segun pulsamos el boton tendremos los estados del motor según los siguientes parámetros
// Valores de : pulsador_cuenta
0. Calentador OFF & Motor OFF
1. Calentador ON & Motor OFF
2. Calentador ON & Motor ON
3. Calentador OFF & Motor OFF
4. Calentador OFF & Motor Reverse
*/

void loop()
{

switch(pulsador_cuenta)
{
case 0://Calentador OFF & Motor OFF
motor = false;
calentador = false;
direccion = 1;
estado = "M:OFF C:OFF";
  break;
case 1://Calentador ON & Motor OFF
motor = false;
calentador = true;
direccion = 1;
estado = "M:OFF C:ON";
  break;
case 2://Calentador ON & Motor ON
motor = true;
calentador = true;
direccion = 1;
estado = "M:ON C:ON";
  break;
case 3://Calentador OFF & Motor OFF
motor = false;
calentador = false;
direccion = 1;
estado = "M:OFF C:OFF";
  break;
case 4://Calentador OFF & Motor Reverse
motor = true;
calentador = false;
direccion = -1;
estado = "M:REV C:OFF";
  break;
}

//--> POTENCIOMETRO VELOCIDAD
velocidad_raw = analogRead(Potenciometro_velocidad); //Leemos la lectura del potenciometro que usamos para selecionar velocidad del motor 
velocidad = map(velocidad_raw,0,4095,0,500); //mapeamos la lectura que vine dada de 0 a 4095 al rango que queremos poder ajustar la velocidad.

//--> POTENCIOMETRO TEMPERATURA
setpoint_temperatura_raw = analogRead(Potenciometro_temperatura); //leemos la lectura del potenciometro de temperatura y almacenamos el valor.
setpoint_mapped = map(setpoint_temperatura_raw,0,4095,0,250); // mapeamos el valor del setpoint entre lso valores que queremos permitir de ajuste

//-->NEMA 17
if (motor == true)
      {
      digitalWrite( enPin , LOW); //Si el programa estar en Motor ON, Habilita el funcionamiento del motor.
      myStepper.setSpeed(velocidad*direccion); //Definimos la velocidad del motor en pasos/segundo según el valor mapeadod el potenciometro
        if (tiempo_inicio == 0) // Si todavia no ha habido un momento en el que se haya puesto por primera vez el motro en ON
        {
        tiempo_inicio = millis(); //Almaceno el timestamp para el inicio en el primer ON del motor.
        tiempo_parada = 0; // Pongo a cero el contador de parada, estamos en estado motor == true del IF.
        }
        if (tiempo_reanudacion == 0 && tiempo_inicio !=  0) //Solo cuando ya haya arrancado antes el motor, y el tiempo de reanudacion se haya puesto a 0 (po una parada)
        {
        tiempo_reanudacion = millis(); // Almaceno el timestamp de reanudacion
        tiempo_parado = tiempo_reanudacion - tiempo_parada; // Calculo el tiempo entre la reanudacion y la parada.
        tiempo_parada = 0; // POngo la parada a 0, estamos en el bucle motor == true del IF.
        }
      }
 else // Si el programa esta en motor OFF , deshabilita el funcionamiento del motor
 {
  if (tiempo_parada == 0 && tiempo_inicio != 0) // Cuando haya habido una parada, tiempo_parada no estará a 0, ademas ya ha arrancado al menos una vez el motor.
  {
    tiempo_parada = millis(); //Almaceno el timestamp de la parada en la variable apra hacer el calculo cuando reanudo el motor del tiempo parado.
    tiempo_reanudacion = 0; //Pongo el tiempo de reanudacion a 0 para limpiar memoria anterior.
  }
  digitalWrite( enPin , HIGH); // Deshabilito el motor.
  myStepper.setSpeed(0); //Definimos la velocidad del motor en pasos/segundo según el valor mapeadod el potenciometro. Lo paro.
 }  

milis_totales_parado = milis_totales_parado + tiempo_parado; // Sumo a la variable de tiempo parado el tiempo parado calculado en caso de que haya habido una parada
tiempo_parado = 0; // Si no ha habido aprada en este ciclo, pongo el contador a 0

if (tiempo_inicio !=0 && tiempo_parada == 0) // Cuando haya arrancado el motor al menso una vez y no este parado en este momento
{
minutos = ((millis()- tiempo_inicio - milis_totales_parado)/60000.00); // Hago el calculo del tiempo en funcionamiento descontando del millis actual el tiempo parado y el momento en que arranco por primera vez.                                                                      
}

 myStepper.runSpeed(); //Ponemos el motor a funcionar segun los aprametros que hemos definido.   
//--> TERMISTOR
//-> Media de lecturas de temperatura del termistor.
int i; // variable para el contador de muestras

        for (i=0; i< NumMuestras; i++) { // Bucle para almacenar las lecturas de la temperatura del termistor
        muestras[i] = analogRead(pinTermistor); //Almacenamos en un array las lecturas realizadas por el sensor
        }
        media_lecturas_temperatura = 0; // Ponemos la lectura anterior a 0
  
        for (i=0; i< NumMuestras; i++) { //Sumamos toras las lecturas anteriores en la variable
        media_lecturas_temperatura += muestras[i];
        }

media_lecturas_temperatura /= NumMuestras; //Dividimos la suma de lecturas por el numero de lecturas para hacer la media.
//-> Convertimos valores medidos a resistencia mediante formula de Steinhart-Hart
/*El gran Luis Llamas: https://www.luisllamas.es/medir-temperatura-con-arduino-y-termistor-mf52/
 *Adafruit: https://learn.adafruit.com/thermistor/using-a-thermistor   */
media_lecturas_temperatura = 4095 / media_lecturas_temperatura - 1;
media_lecturas_temperatura = Resistencia_serie_termistor / media_lecturas_temperatura;
temperatura = media_lecturas_temperatura / Termistor_resistencia_nominal;     // (R/Ro)
temperatura = log(temperatura);                  // ln(R/Ro)
temperatura /= Termistor_coeficiente_B;                   // 1/B * ln(R/Ro)
temperatura += 1.0 / (Termistor_temperatura_nominal + 273.15); // + (1/To)
temperatura = 1.0 / temperatura;                 // Invertimos el valor
temperatura -= 273.15;                         // Convertimos temperatura de Kelvin a C
temperatura_final = temperatura; //En el valor temperatura final tenemos la lectura en C actual.

//--> MOSFET
digitalWrite(pinMosfet, LOW);

//--> PID
Setpoint = setpoint_mapped; //Definimos el setpoint apra el PID como el setpoint mappeado procedente del potenciometro.
Input = temperatura; //El valor de temperatura actual es el input para el PID
if (calentador == true) //Si el calentador no dedbe estar activo la salida es siempre 0%, sino computa el PID
  {
    myPID.Compute(); // Realiza el calculo del parametro del PID en funciond e los parametros.
  }
else
  {
  Output = 0; // Sino el valor PWM es 0
  }

  analogWrite(pinMosfet, Output); // escribimos en el PWM el valor del output 
  porcentaje_PWM = map(Output, 0, 255, 0, 100); //Mapeamos el PWM para expresar en el LCD en funcion %

//-->LCD
if (millis() > (tiempo_ultimo_refresh_LCD + 200 ) ) // Refrescamos al LCD unicamente cada cierto tiempo, en funcion del ultimo refresco.
{

switch(refresco_linea)
{
case 0:
    lcd.setCursor(5,1);
    lcd.print("    ");
        myStepper.runSpeed(); //Continuamos moviendo el motor

    lcd.setCursor(5,1);
    lcd.print(velocidad*direccion);
         myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 1:
    lcd.setCursor(14,1);
    lcd.print(minutos);
        myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 2:
    lcd.setCursor(6,2);
    lcd.print("   ");
        myStepper.runSpeed(); //Continuamos moviendo el motor

    lcd.setCursor(6,2);
    lcd.print(temperatura,0);
        myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 3:
    lcd.setCursor(15,2);
    lcd.print("   ");
        myStepper.runSpeed(); //Continuamos moviendo el motor

    lcd.setCursor(15,2);
    lcd.print(setpoint_mapped,0);
        myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 4:
    lcd.setCursor(4,3);
    lcd.print("   ");
            myStepper.runSpeed(); //Continuamos moviendo el motor

    lcd.setCursor(4,3);
    lcd.print(porcentaje_PWM,0);
            myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 5:
    lcd.setCursor(4,3);
    lcd.print("   ");
        myStepper.runSpeed(); //Continuamos moviendo el motor

    lcd.setCursor(4,3);
    lcd.print(porcentaje_PWM,0);
        myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
 case 6:
    lcd.setCursor(9,3);
    lcd.print("           ");
        myStepper.runSpeed(); //Continuamos moviendo el motor
    lcd.setCursor(9,3);
    lcd.print(estado);
        myStepper.runSpeed(); //Continuamos moviendo el motor
 break;
}
refresco_linea = refresco_linea +1 ;


    tiempo_ultimo_refresh_LCD = millis(); // Almaceno en la variale el tiempo del ultimo refresco para poder calcular el momento de refresco proximo.
}
if (refresco_linea==7) {refresco_linea=0;}

}



