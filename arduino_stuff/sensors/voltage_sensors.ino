//Don't include this library if you are not using a Serial LCD
//#include <NewSoftSerial.h>
//NewSoftSerial LCD(2, 3);

#include <Sensors.h>
Sensors s1, s2, s3, s4, s5, s6;

void setup()
{
  s1 = new Sensors(A0, A1);
  s2 = new Sensors(A2, A3);
  s3 = new Sensors(A4, A5);
  s4 = new Sensors(A6, A7);
  s5 = new Sensors(A8, A9);
  s6 = new Sensors(A10, A11);
  Serial.begin(9600);
}

void loop()
{
  s1.READ();
  s2.READ();
  s3.READ();
  s4.READ();
  s5.READ();
  s6.READ();
  
  s1.convert();
  s2.convert();
  s3.convert();
  s4.convert();
  s5.convert();
  s6.convert();
  
  s1.PRINT();
  s2.PRINT();
  s3.PRINT();
  s4.PRINT();
  s5.PRINT();
  s6.PRINT();
}
