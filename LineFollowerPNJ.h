/*==================Library Line Sensor====================
 * 
 *===============InoTech Depok Development=================
 *
 *  Author By Saiful Anton
 *  Electrical Engineering 2017
 *  Psychorobotics PNJ
 *  Politecnic of Jakarta
 *  
 *  Message : Terus Kembangkan Program ini dan gunakan 
 *            dengan Bijak dan Jangan tinggalkan ibadah
 *          
 * ========================================================
*/
#ifndef LineFollowerPNJ_h
#define LineFollowerPNJ_h

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "EEPROM.h"

class lineSensor
{ 
  private:
    byte _s1,_s2,_s3,_adc;
    int  _sensor[7];
    bool  _sensor1[7];
    unsigned long lastCond = 0;
    bool _condition = 0;
    void setSelector(byte n); 
  public:
    lineSensor(byte s1, byte s2, byte s3, byte adc );
    uint8_t readLine();
    void scanLine(unsigned long i);
};

lineSensor::lineSensor(byte s1, byte s2, byte s3, byte adc )
{
  _s1  = s1;
  _s2  = s2;
  _s3  = s3;
  _adc = adc;
  pinMode(_s1,OUTPUT);
  pinMode(_s2,OUTPUT);
  pinMode(_s3,OUTPUT);
  pinMode(_adc,INPUT);  
}

uint8_t lineSensor::readLine()
{
  for(int i = 0; i < 8; i++)
  {
    
    setSelector(i);
    _sensor[i] = analogRead(_adc);
    _sensor[0]<EEPROM.read(0)? _sensor1[0]=0:_sensor1[0]=1;
    _sensor[1]<EEPROM.read(1)? _sensor1[1]=0:_sensor1[1]=1;
    _sensor[2]<EEPROM.read(2)? _sensor1[2]=0:_sensor1[2]=1;
    _sensor[3]<EEPROM.read(3)? _sensor1[3]=0:_sensor1[3]=1;
    _sensor[4]<EEPROM.read(4)? _sensor1[4]=0:_sensor1[4]=1;
    _sensor[5]<EEPROM.read(5)? _sensor1[5]=0:_sensor1[5]=1;
    _sensor[6]<EEPROM.read(6)? _sensor1[6]=0:_sensor1[6]=1;
    _sensor[7]<EEPROM.read(7)? _sensor1[7]=0:_sensor1[7]=1;
	
    
  }  
  uint8_t a = (_sensor1[0]*128 + _sensor1[1]*64 + _sensor1[2]*32 + _sensor1[3]*16 + _sensor1[4]*8 + _sensor1[5]*4 + _sensor1[6]*2 + _sensor1[7]*1  );
  return a;
}

void lineSensor::scanLine(unsigned long i)
{	
	while(_condition==0)
	{
		if(millis()-lastCond < i)
		{
			for(int i = 0; i < 8; i++)
		  	{
		    		setSelector(i);
		    		_sensor[i] = analogRead(_adc);
		    		EEPROM.write(i,_sensor[i]);
				delay(1);
		  	}
			Serial.println("ok");
		}
		else 
		{
			_condition=1;
      			lastCond=millis();
		}
		
		Serial.println("ok1");
	}
}

void lineSensor::setSelector(byte n)
{
  if(n==0)
  {
    digitalWrite(_s1, LOW);
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, LOW);
  }
   
  else if(n==1)
  {
    digitalWrite(_s1, HIGH);
    digitalWrite(_s2, LOW);
    digitalWrite(_s3, LOW);
  }
    
  else if(n==2)
  {
    digitalWrite(_s1, LOW);
    digitalWrite(_s2, HIGH);
    digitalWrite(_s3, LOW);
  }
    
  else if(n==3)
  {
   digitalWrite(_s1, HIGH);
   digitalWrite(_s2, HIGH);
   digitalWrite(_s3, LOW);
  }
  
  else if(n==4)
  {
   digitalWrite(_s1, LOW);
   digitalWrite(_s2, LOW);
   digitalWrite(_s3, HIGH);
  }  

  else if(n==5)
  {
   digitalWrite(_s1, HIGH);
   digitalWrite(_s2, LOW);
   digitalWrite(_s3, HIGH);
  }
  
  else if(n==6)
  {
   digitalWrite(_s1, LOW);
   digitalWrite(_s2, HIGH);
   digitalWrite(_s3, HIGH);
  }
  
  else
  {
   digitalWrite(_s1, HIGH);
   digitalWrite(_s2, HIGH);
   digitalWrite(_s3, HIGH);
  }
}
