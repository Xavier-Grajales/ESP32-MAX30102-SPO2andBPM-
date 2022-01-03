/*Programador: Xavier Emmanuel Domínguez Grajales
 * En este programa se conjuntan los ejemplos de SPO2 y BPM de la libreria Sparkfun para el sensor
 * Max30102 en un ESP32, este código aún no cuenta con  la conectividad a internet.
 * 
 * ESP32--------------------Max30102
 * GND------------------GND
 * Vin------------------3V3(En caso de no funcionar conectar a 5V)
 * SDA------------------GPIO21
 * SCL------------------GPIO22
*/

//Librerias
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
//definición necesaria para la medición de SPO2
#define MAX_BRIGHTNESS 255

//Variables que permiten el funcionamiento del sensor MAX30102
const byte RATE_SIZE = 4; //Esta variable se puede incrementar para obtener más promedios de la medición
byte rates[RATE_SIZE]; //Arreglo de frecuencia cardiaca
byte rateSpot = 0;
long lastBeat = 0; //Tiempo en el que ocurrió el ultimo latido 
float beatsPerMinute;
int beatAvg;
/*------------------------Sección SPO2 variables-----------------------------------------------*/
//Preparamos todas las variables necesarias para la medición del SPO2
uint32_t irBuffer[100]; //Datos del LED infrarrojo
uint32_t redBuffer[100];  //Daros del LED rojo

int32_t bufferLength; //longitud de datos
int32_t spo2; //SPO2 valor
int8_t validSPO2; //indicador para mostrar si el cálculo de SPO2 es válido
int32_t heartRate; //valor de la frecuencia cardiaca segun el algoritmo
int8_t validHeartRate; //indicador para mostrae si el calculo de la frecuencia cardiaca es valido.

byte pulseLED = 2; //LED integrado en ESP32
byte readLED = 19; //Parpadea con cada lectura de datos.


void setup()
{
  Serial.begin(115200);
  Serial.println("Inicializando..");
  //Definimos los leds que nos serviran como bandera
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Inicialización del sensoe
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Usamos los puertos I2C predeterminados y la velocidad a 400KHz
  {
    Serial.println("MAX30102 no se ha encontrado, verificar la conexión/alimentación. ");
    while (1);
  }
  Serial.println("Por favor, coloque su dedo indice con presión constante.");

  particleSensor.setup(); //Configura el sensor con los ajustes predeterminados
  particleSensor.setPulseAmplitudeRed(0x0A); //Baja el led rojo para indicar que el sensor está funcionando
  particleSensor.setPulseAmplitudeGreen(0); //Apaga el led verde. Posiblemente lo omitiremos

  Serial.println(F("Coloque el sensor en el dedo con una banda elástica. Presione cualquier tecla para iniciar la conversión"));
  while (Serial.available() == 0) ; //espere hasta que el usuario presione una tecla
  Serial.read();
/*---------------------------Definimos la parte del Setup necesaria para la medición del SPO2--------------------------------------------*/
  byte ledBrightness = 60; //Opciones: 0=Apagado a 255=50mA
  byte sampleAverage = 4; //Opciones para promediar la muestras: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //se elige las opciones del led en este caso 2 ṕorque necesitamos infrarojo y rojo: 1 = Rojo , 2 = Rojo + IR, 3 = Rojo + IR + verde
  byte sampleRate = 100; //opciones de la frecuencia de muestreo: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Opciones del ancho de pulso: 69, 118, 215, 411
  int adcRange = 4096; //Opciones: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configuramos al sensor con estos ajustes
}

void loop()
{
  /*------------------------------Loop de la medición de BPM-------------------------------*/
  long irValue = particleSensor.getIR();//toma la lectura del led infrarojo

  if (checkForBeat(irValue) == true)
  {
    //Sensamos un latido
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Almacenamos la lectura en el arreglo
      rateSpot %= RATE_SIZE; //Wrap variable

      //Tomamos el promedio de las lecturas
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  //Medición promedio de los latidos.
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No se ha colocado el dedo");

  Serial.println();
  /*-----------------------------Loop de medición de SPO2----------------------------------------------*/
  bufferLength = 100; //La longitud del búfer de 100 almacena 4 segundos de muestras

  //Lee las primeras 100 muestras y determina el rango de la señal
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //tenemos nuevos datos?
      particleSensor.check(); //verificación del sensor en busca de nuevos datos.

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //Terminamos con esta muestra.

    //Serial.print(F("red="));
    //Serial.print(redBuffer[i], DEC);
    //Serial.print(F(", ir="));
    //Serial.println(irBuffer[i], DEC);
  }

  //calcular la frecuencia cardiaca después de las primeras 100 muestras (primeros 4 segundos de las muestras)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Toma de muestras continua del MAX30102, La frecuencia cardiaca y la SPO2 se calculan cada segundo
  while (1)
  {
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //Tomar 25 conjuntos de muestra antes de calcular la frecuencia cardiaca.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //tenemos nuevos datos?
        particleSensor.check(); //verificación del sensor en busca de nuevos datos

      digitalWrite(readLED, !digitalRead(readLED)); //Parpade el led integrado en el ESP32 por cada lectura

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //Se termina con esta muestra 

      //Serial.print(F("red="));
      //Serial.print(redBuffer[i], DEC);
      //Serial.print(F(", ir="));
      //Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //Después de recopilar 25 muestras vuelva a calcular HR y SPO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
