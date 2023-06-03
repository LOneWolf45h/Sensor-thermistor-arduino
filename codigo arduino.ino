#include <Arduino.h>
#include <math.h>
#include <PID_v1_bc.h>

 
// configurar el pin utilizado para la medicion de voltaje del divisor resistivo del NTC
#define CONFIG_THERMISTOR_ADC_PIN A0
// configurar el valor de la resistencia que va en serie con el termistor NTC en ohms
#define CONFIG_THERMISTOR_RESISTOR 9900l

int32_t thermistor_get_resistance(uint16_t adcval)
{
 
  return (CONFIG_THERMISTOR_RESISTOR * ((1023.0 / adcval) - 1));
}
 

float thermistor_get_temperature(int32_t resistance)
{
  // variable de almacenamiento temporal, evita realizar varias veces el calculo de log
  float temp;
 
  // calculamos logaritmo natural, se almacena en variable para varios calculos
  temp = log(resistance);
 
  // resolvemos la ecuacion de STEINHART-HART
  // http://en.wikipedia.org/wiki/Steinhart–Hart_equation
  temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
 
  // convertir el resultado de kelvin a centigrados y retornar
  return temp - 273.15;
}
 // Constantes del termistor
const float R1 = 6320;  // Resistencia conocida en ohmios
const float THERMISTOR_NOMINAL = 10000;  // Resistencia nominal del termistor en ohmios
const float TEMPERATURE_NOMINAL = 25;  // Temperatura nominal del termistor en grados Celsius
const float B_COEFFICIENT = 3950;  // Coeficiente B del termistor
const float  desired_temperature=40;

// Pines de entrada y salida
const int THERMISTOR_PIN = A0;  // Pin analógico conectado al termistor
const int OUTPUT_PIN = 9;  // Pin de salida conectado al sistema de control (por ejemplo, un relé)

// Variables del PID
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;  // Ajusta estos valores según tus requisitos
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup()
{
  // preparar serial
  Serial.begin(9600);
  while (!Serial)
    ;
 

  pinMode(OUTPUT_PIN, OUTPUT);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);  // Limita la salida entre 0 y 255 (por ejemplo, para controlar un relé)
  Setpoint =  desired_temperature;  // Establece la temperatura deseada

}
 
void loop()
{
  // variable para almacenar la temperatura y resistencia
  float temperatura;
  uint32_t resistencia;
 
  // calcular la resistencia electrica del termistor usando la lectura del ADC
  resistencia = thermistor_get_resistance(analogRead(CONFIG_THERMISTOR_ADC_PIN));
  // luego calcular la temperatura segun dicha resistencia
  temperatura = thermistor_get_temperature(resistencia);
 
  // imprimir resistencia y temperatura al monitor serial
 
  Serial.println(" Temperatura: ");
  Serial.println(temperatura, 1);
 
  // esperar 5 segundos entre las lecturas
  delay(100);


    // Lectura del valor del termistor
  int rawValue = analogRead(THERMISTOR_PIN);

  // Cálculo de la resistencia del termistor
  float resistance = R1 * (1023.0 / rawValue - 1.0);

  // Cálculo de la temperatura en grados Celsius utilizando la fórmula de Steinhart-Hart
  float steinhart;
  steinhart = resistance / R1;  // (R/Ro)
  steinhart = log(steinhart);  // ln(R/Ro)
  steinhart /= B_COEFFICIENT;  // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;  // Invertir
  float temperature = steinhart - 273.15;  // Convertir a Celsius

  // Actualizar la entrada y salida del PID
  Input = temperature;
  pid.Compute();

  // Controlar el sistema de acuerdo a la salida del PID
  analogWrite(OUTPUT_PIN, Output);

}