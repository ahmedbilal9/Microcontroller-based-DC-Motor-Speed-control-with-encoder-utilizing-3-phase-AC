#include <Arduino.h>

uint16_t no_of_teeths = 30; 
uint32_t count = 0;
uint32_t count_prev = 0;

void setup()
{
  // Enable external clock
  SystemClock_Config();

  // Enable USB Peripheral
  Serial.begin(115200);
  delay(1000);

  pinMode(PA0, INPUT);

  attachInterrupt(digitalPinToInterrupt(PA0), encoder_input, RISING);

  interrupts();
}

void loop()
{
  while(1)
  {
    uint32_t t1 = micros();
    while ( (micros()-t1) < 1000000 );

    noInterrupts();

    Serial.println(((count - count_prev) * 60) / no_of_teeths);
    Serial.println(count - count_prev);
    count_prev = count;

    interrupts();
  }
}

void encoder_input()
{
  count++;
}
