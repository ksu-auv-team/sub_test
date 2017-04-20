#include "Arduino.h"
#include "Sensors.h"

Sensors::Sensors()
{}

Sensors::Sensors(int vPin, int iPin)
{
	pinMode(vPin, INPUT);
	pinMode(iPin, INPUT);
	this->vPin = vPin;
	this->iPin = iPin;
}

void Sensors::READ()
{
	VRaw = analogRead(vPin);
	IRaw = analogRead(iPin);
}

void Sensors::convert()
{
	// 90 Amp board
	VFinal = VRaw / 12.99;
	IFinal = IRaw / 7.4;
}

void Sensors::PRINT()
{
	//cout << VFinal << "\tVolts" << endl;
	Serial.print(VFinal);
	Serial.println("\tVolts");
	//cout << IFinal << "\tAmps" << endl;
	Serial.print(IFinal);
	Serial.println("\tAmps");
	delay(200);
}

Sensors* Sensors::operator=(Sensors* s)
{
	return s;
}
