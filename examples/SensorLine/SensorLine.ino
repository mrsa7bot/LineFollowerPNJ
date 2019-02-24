// include library

#include <LineFollowerPNJ.h>;

//inisialisasi instance line dengan pin 2 = selector 1, pin 3 = selector 2, pin 3 = selector 3, pin A0 = adc
lineSensor line(7,8,9,A0); 

  void setup() 
{
  //cara membaca sensor garis : instance.readLine; 
  Serial.begin(9600);
  Serial.println("=============Menscan Garis=============");
  line.scanLine(6000); //Scan Sensor Selama 6 detik
  delay(1000);
}

void loop() {
  Serial.print("nilai Sensor pada EEPROM = ");
  Serial.println(line.readLine(),BIN);
  Serial.print("nilai Error = ")
  Serial.println(line.errorLine());
  delay(500);
}
