// include library

#include <LineFollowerPNJ.h>;

//inisialisasi instance manuver dengan pin 10 = directionM1 , pin 11 = directionM1 , pin 12 = PWM M1 3, pin 13 = PWM M2
manuver    robot(10,11,12,13);

void setup() 
{
  //cara membaca sensor garis : instance.readLine; 
  Serial.begin(9600);
  robot.setSpeed(100,100); //default speed M1 dan M2 = 100 walaupun ini tidak dipanggil : manuver.setSpeed(SpeedM1, SpeedM2);
}

void loop() {
  robot.lurus();
  delay(3000);
  robot.kanan();
  delay(3000);
  robot.kiri();
  delay(3000);
  robot.stop;
  delay(6000);
}
