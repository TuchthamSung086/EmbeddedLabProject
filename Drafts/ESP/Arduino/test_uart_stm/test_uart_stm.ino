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
  readUART(NodeSerial);
  
  delay(100);
}

void readUART(SoftwareSerial &NodeSerial) {
  bool found = false;
  char str[5];
  strcpy(str,"");
  while (NodeSerial.available() > 0) {
    found = true;
    //Serial.print(NodeSerial.available());
    //Serial.print(" bytes waiting");
    int x = NodeSerial.read();
    char xx = x;
    if (x==0 || x>127) break;
    strcat(str,&xx);
    //Serial.print(xx);
  }
  if (found && strlen(str)>0) {
    Serial.print("Data from NodeSerial is ");
    Serial.println(str);
  }
}
