/*--------------------------------------------------------------
  Program:      ICStation E003A Comparator Module LM393 Adjustable Sensitivity C Demo

  Description:  Reads value on analog input A0, digital input 2 and calculates
                the voltage from analog output (A0) from ICStation E003A Comparator Module LM393 Adjustable Sensitivity C
                You can buy this module from http://www.icstation.com/product_info.php?products_id=3303

  Hardware:     Arduino Duemilanove with ICStation 003A Comparator
                Pin Diagram
                from ICStation E003A Comparator Module LM393 Adjustable Sensitivity C     Arduino Duemilanove
                GND                                                                       GND
                VCC                                                                       5V
                AO                                                                        A0 (ANALOG IN)
                DO                                                                        DO (DIGITAL)

                2 input pins from this module must connected to resistive sensor such as LCR, NTC, etc. You can you use potentiometer for testing purpose

  Software:     Developed using Codeblocks for Arduino 12.11 software for Windows
                Should be compatible with Arduino 1.0 +

  Date:         15 May 2014

  Author:       Mohamad Dani
                Youtube channel : http:///www.youtube.com/danstama
                Twitter : @MD4N1

  This code is based from W.A. Smith, http://startingelectronics.com
--------------------------------------------------------------*/

#include <Arduino.h>
// number of analog samples to take per reading
#define NUM_SAMPLES 10

//Pins Configuration
#define AnalogSensor A0
#define DigitalSensor 2

int sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float voltage = 0.0;            // calculated voltage
unsigned char decision ;

void setup()
{
    pinMode(DigitalSensor,INPUT_PULLUP); //Pin 2 Arduino as input port
    Serial.begin(9600);
    Serial.println("ICStation E003A Comparator Module LM393 Adjustable Sensitivity C Demo");
    Serial.println("=====================================================================");
    Serial.println("Reads value on analog input A0, digital input 2 and calculates");
    Serial.println("Voltage from analog output (A0) from");
    Serial.println("ICStation E003A Comparator Module LM393 Adjustable Sensitivity C");
    Serial.println("You can buy this module from");
    Serial.println("http://www.icstation.com/product_info.php?products_id=3303");
    Serial.println("Author            : Mohamad Dani");
    Serial.println("Youtube channel   : http:///www.youtube.com/danstama");
    Serial.println("Follow my Twitter : @MD4N1");
    Serial.println("Date              : 15 May 2014");
    Serial.println("======================================================================");
    Serial.println();
    delay(5000);
}

void loop()
{
    // take a number of analog samples and add them up
    while (sample_count < NUM_SAMPLES)
    {
        sum += analogRead(AnalogSensor);
        sample_count++;
        delay(10);
    }
    //Reads digital output value from pin 2 Arduino
    decision = digitalRead(DigitalSensor);
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 5.015V is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;

    // send voltage and digital value for display on Serial Monitor
    Serial.print("Analog  Value: ");
    Serial.print(voltage);
    Serial.println ("V");
    Serial.print("Digital Value: ");
    Serial.println(decision);
    sample_count = 0;
    sum = 0;
    delay(100);
}
