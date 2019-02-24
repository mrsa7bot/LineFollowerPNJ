// include library
#include <LineFollowerPNJ.h>;
#include <LiquidCrystal_I2C.h>

//inisialisasi instance line dengan pin 2 = selector 1, pin 3 = selector 2, pin 3 = selector 3, pin A0 = adc

lineSensor        line(7,8,9,A0); 
manuver           robot(10,11,12,13);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Button            btn1(10,"pullup");


float kp = 4;
float ki = 2;
float kd = 0.1;
byte  speedM1 = 100;
byte  speedM2 = 100;  
unsigned long lastTime = 0;
unsigned long interval = 60000;

void setup() 
{
  //cara membaca sensor garis : instance.readLine; 
  Serial.begin(9600);
  
  robot.setSpeed(speedM1,speedM2);
  robot.setConstPID(kp,ki,kd);
  
  Serial.println("Membaca Garis \n \n");
  line.scanLine(10000); //Scan Sensor Selama 10 detik
  delay(1000);
}

void loop() 
{

  //Start Button to run manuver Robot
  if(btn1.flagButton()==0)
  {
    //follow line selama 60 detik
    if(millis()- lastTime < interval)
    {
      //follow line 
      line.readLine();                //baca garis
      byte error = line.errorLine();  //baca nilai error sensor garis
      Serial.println("Maju Teroos");
      robot.lurus(error);             //menggungakan PID dengan memasukan seperti ini contoh lain robot.kanan(error); atau robot.kanan(error);
    }
  
    else 
    {
     Serial.println("stop");
     robot.stop();
     btn1.resetFlag();
     lastTime=millis();
    }
  } 
}
