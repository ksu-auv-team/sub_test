#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"

class Sensors
{
	public:
		Sensors();
		Sensors(int vPin, int iPin);
		void PRINT();
		void convert();
		void READ();
		Sensors* operator=(Sensors* s);
	private:
		int vPin;
		int iPin;
		int VRaw;
		int IRaw;
		int VFinal;
		int IFinal;
};

#endif
