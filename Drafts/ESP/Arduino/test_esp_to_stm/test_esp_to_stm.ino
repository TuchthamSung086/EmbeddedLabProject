#include <SoftwareSerial.h>

SoftwareSerial NodeSerial(D2, D3); // RX | TX

void setup() {
  // put your setup code here, to run once:
  
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  Serial.begin(115200);
  NodeSerial.begin(115200);
  
  Serial.println("Done setup!");
}

void loop() {
  // put your main code here, to run repeatedly:
  String x = "Hello";
  sendUART(NodeSerial,x,5);
  delay(1000);
}


void sendUART(SoftwareSerial &NodeSerial, String &str, int len) {
   char p[str.length()+1];
 
    int i;
    for (i = 0; i < str.length()+1; i++) {
        if ((int) str[i] < 127)
        p[i] = str[i];
    }
    Serial.println(p);
  NodeSerial.write(p);
}
