// include library
#include <LineFollowerPNJ.h>;

//inisialisasi instance line dengan pin 2 = selector 1, pin 3 = selector 2, pin 3 = selector 3, pin A0 = adc
lineSensor line(7,8,9,A0); 
manuver    robot(10,11,12,13);



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
  //follow line selama 60 detik
  if(millis()- lastTime < interval)
  {
    //follow line 
    line.readLine();                //baca garis
    byte error = line.errorLine();  //baca nilai error sensor garis
    robot.lurus(error);             //manuver menggunakan PID dengan memasukan seperti ini pada contoh lain robot.kanan(error); atau robot.kanan(error);
  }

  else 
  {
   robot.stop();
  } 
}
