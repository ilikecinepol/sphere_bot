#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev
 
MPU6050 GY521;                       // Создаем объект, символизирующий модуль датчика
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп
 
void setup()
{
Wire.begin();                         // Инициализация Wire
Serial.begin(9600);                  // Инициализация последовательного порта
Serial.println("Initializing I2C devices..."); // Печать текста
GY521.initialize();                   // Инициализация MPU
delay(100);                           // Пауза
 
}
void loop()
{
GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
 
// вывод значений в монитор
Serial.print("a/g:\t");
Serial.print(ax); Serial.println("\t");
Serial.print(ay); Serial.print("\t");
Serial.print(az); Serial.print("\t");
Serial.print(gx); Serial.println("\t");
//Serial.print(gy); Serial.print("\t");
//Serial.println(gz);
delay(50); 
}
