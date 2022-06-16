#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <Servo.h> 
MPU6050 GY521;                     // Создаем объект, символизирующий модуль датчика
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп
// Переменные для драйвера двигателей движения
int en = 9;
int ina = 5;
int inb = 7;
int pwm_pin = 13;
Servo motor;
Servo StabY;
Servo StabX;
unsigned long timing_sence= millis();
void setup(){
  Wire.begin();                         // Инициализация Wire
  Serial.begin(9600);                  // Инициализация последовательного порта
  Serial.println("Initializing I2C devices..."); // Печать текста
  GY521.initialize();                   // Инициализация MPU
  delay(100);                           // Пауза
 
  pinMode(en, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  
 pinMode(3, OUTPUT);
 pinMode(5, OUTPUT);
 digitalWrite(3, HIGH);
 StabX.attach(10);
 StabY.attach(5, 10, 150);
 motor.attach(11);
 motor.writeMicroseconds(2300);
 delay(2000);
 motor.writeMicroseconds(800);
 delay(6000);
 Wire.begin();

}
void moving(){
  digitalWrite(en, 1);
  digitalWrite(ina, 0);
  digitalWrite(inb, 1 );
  analogWrite(pwm_pin, 250);
}
void loop()
{
GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
 
// вывод значений в монитор
Serial.print("a/g:\t");
Serial.print(ax); Serial.println("\t");
Serial.print(f_ax); Serial.println("\t");
//Serial.print(ay); Serial.print("\t");
//Serial.print(az); Serial.print("\t");
//Serial.print(gx); Serial.println("\t");
//Serial.print(gy); Serial.print("\t");
//Serial.println(gz);
StabY.write(map(f_ax, -7000, 7000, 110, 70));
delay(50); 

moving();
}
