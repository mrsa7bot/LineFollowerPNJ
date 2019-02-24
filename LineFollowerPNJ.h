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
    int error;
  public:
    lineSensor(byte s1, byte s2, byte s3, byte adc );
    uint8_t errorLine();
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

uint8_t lineSensor::errorLine()
{
	if     (readLine()==254) error = -7;
	else if(readLine()==248) error = -6;
	else if(readLine()==252) error = -6;
	else if(readLine()==253) error = -5;
	else if(readLine()==241) error = -4;
	else if(readLine()==249) error = -4;
	else if(readLine()==251) error = -3;
	else if(readLine()==227) error = -2;
	else if(readLine()==243) error = -2;
	else if(readLine()==247) error = -1;
	else if(readLine()==231) error =  0;
	else if(readLine()==239) error =  1;
	else if(readLine()==199) error =  2;
	else if(readLine()==207) error =  2;
	else if(readLine()==223) error =  3;
	else if(readLine()==143) error =  4;
	else if(readLine()==159) error =  4;
	else if(readLine()==191) error =  5;
	else if(readLine()==31)  error =  6;
	else if(readLine()==63)  error =  6;
	else if(readLine()==127) error =  7;
	else if(readLine()==255) error =  8;
	return error;
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

class manuver
{
 private :
  byte _pinDir1;
  byte _pinPwm1;
  byte _pinDir2;
  byte _pinPwm2;
  float _kp, _ki, _kd,previousError;  
  byte speedM1 = 100;
  byte speedM2 = 100;
 public:
  float PID(uint8_t error);
  void setConstPID(float kp, float ki, float kd);
  manuver(byte pinDir1, byte pinDir2 , byte pinPwm1, byte pinPwm2);
  String setSpeed(byte s1, byte s2);
  void lurus(byte error);
  void kanan(byte error);
  void kiri(byte error);
  void stop();
};

manuver :: manuver(byte pinDir1, byte pinDir2 , byte pinPwm1, byte pinPwm2)
{
  _pinDir1 = pinDir1;
  _pinDir2 = pinDir2;
  _pinPwm1 = pinPwm1;
  _pinPwm2 = pinPwm2;

  pinMode(_pinDir1, OUTPUT);
  pinMode(_pinDir2, OUTPUT);
  pinMode(_pinPwm1, OUTPUT);
  pinMode(_pinPwm2, OUTPUT);
}

String manuver :: setSpeed(byte s1, byte s2)
{ 
	
	EEPROM.write(11 ,s1);
	EEPROM.write(12 ,s2);
	speedM1  = EEPROM.read(11);
  	speedM2  = EEPROM.read(12); 
	return speedM1 + "" + speedM2;
}

void manuver :: lurus(byte error)
{
	byte a = _pinDir1;
  	byte b = _pinDir2;
	byte c = _pinPwm1;
 	byte d = _pinPwm2;
	digitalWrite(a , HIGH);
	digitalWrite(b , LOW);
	analogWrite(c , speedM1 + PID(error));
	analogWrite(d , speedM2 - PID(error));
}

void manuver :: kanan(byte error)
{
	byte a = _pinDir1;
  	byte b = _pinDir2;
	byte c = _pinPwm1;
 	byte d = _pinPwm2;

	digitalWrite(a , HIGH);
	digitalWrite(b , HIGH);
	analogWrite(c , speedM1 + PID(error));
	analogWrite(d , speedM2 - PID(error));
}

void manuver :: kiri(byte error)
{
	byte a = _pinDir1;
  	byte b = _pinDir2;
	byte c = _pinPwm1;
 	byte d = _pinPwm2;
	digitalWrite(a , LOW);
	digitalWrite(b , LOW);
	analogWrite(c , speedM1 + PID(error));
	analogWrite(d , speedM2 - PID(error));
}

void manuver :: stop()
{
	byte a = _pinDir1;
  	byte b = _pinDir2;
	byte c = _pinPwm1;
 	byte d = _pinPwm2;
	digitalWrite(a , LOW);
	digitalWrite(b , LOW);
	analogWrite(c ,0);
	analogWrite(d ,0);
}

void manuver :: setConstPID(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki; 
	_kd = kd;
	EEPROM.write(8 , _kp);
	EEPROM.write(9 , _ki);
	EEPROM.write(10, _kd);
}

float manuver :: PID(byte error)
{
	_kp = EEPROM.read(8);
	_ki = EEPROM.read(9);
	_kd = EEPROM.read(10);
  	float P = error;
 	float I = I + error;
 	float D = error-previousError;
 	float PIDvalue = (_kp*P) + (_ki*I) + (_kd*D);
  	previousError = error;
	return PIDvalue;
}



class Button
{
	private :
		byte _pin;
		String _mode = "";
		bool flag = 1;
		byte count= 0;
		int buttonState = 0;         // current state of the button
		int lastButtonState = 0;     // previous state of the button
		unsigned long last = 0;
		byte _max = 100;
	public:	

		Button(byte pin)
		{
			_pin  = pin; 
			pinMode(_pin,INPUT);
		}
	
		Button(byte pin, String mode)
		{
			_pin  = pin;
			_mode = mode;
			_mode!=""? pinMode(_pin,INPUT_PULLUP):pinMode(_pin, INPUT);							;
		}
		bool flagButton();
		byte countDwButton();
		byte countUpButton();
		void setMaxCount(byte max);
		bool readState();
		void resetFlag();
};

bool Button :: flagButton()
{	
	byte pin = _pin;
	buttonState = digitalRead(pin);
	buttonState != lastButtonState? ( millis()-last<50?(buttonState==HIGH? flag=0:flag=1 ):last=millis()) : lastButtonState = buttonState;
	return flag;
}

byte Button :: countUpButton()
{
	byte pin = _pin;
	buttonState = digitalRead(pin);
	buttonState != lastButtonState? (millis()-last<50?(buttonState == HIGH? count++:count=count):last=millis()):lastButtonState = buttonState;
	count<=_max? count=_max: count=count;
	return count;
}	

byte Button :: countDwButton()
{
	byte pin = _pin;
	buttonState = digitalRead(pin);
	buttonState != lastButtonState? (millis()-last<50?(buttonState == HIGH? count--:count=count):last=millis()):lastButtonState = buttonState;
	count<=0? count=0: count=count;
	return count;
}

void Button :: setMaxCount(byte max)
{
	_max = max;
}

bool Button :: readState()
{
	byte pin = _pin;
	buttonState = digitalRead(pin);
	return buttonState;
}

void Button :: resetFlag()
{
	return flag=1;
}
