//
// Copyright 2015 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// FirebaseDemo_ESP8266 is a sample that demo the different functions
// of the FirebaseArduino API.


// ***********************************************************************************
// FROM TOUCHY, READ ME FIRST 
// You need to change
// 1. WIFI_SSID and WIFI_PASSWORD to your own WIFI.(Note: Use 2G, not 5G because esp is dumb)
// 2. If necessary, change FIREBASE_HOST and FIREBASE_AUTH (Note: But I'm in charge of this so keep it unchanged)
// ***********************************************************************************


#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <SoftwareSerial.h>

// Set these to run example.
#define FIREBASE_HOST "embedded-lab-project-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "rmzdZbUALOfixTPyV7Xk1F1YVGqm3mNVSgh3IoyO"
#define WIFI_SSID "TrueGigatexFiber_2.4GX77" // "Spamma 5G" / "Touchy's" / "TrueGigatexFiber_2.4GX77"
#define WIFI_PASSWORD "TouchPloy38" // "0917191691" / "sleepisfortheweak" / "TouchPloy38"

// Set my own variables
SoftwareSerial NodeSerial(D2, D3); // RX | TX

void setupESP() {
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  NodeSerial.begin(115200);
  
  Serial.println("Done ESP setup!");
}

void setup() {
  Serial.begin(115200);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  setupESP();
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void loop() {
//  String rq = "h";
//  sendUART(NodeSerial, rq);
//  readUART(NodeSerial); 
  ldrRq();
  delay(1000);
  dhtRq();
  delay(1000);
  dustRq();
  delay(1000);
}

void ldrRq() {
  Serial.println("Requesting LDR...");
  String rq = "l";
  String data = "";
  sendUART(NodeSerial, rq);
  data = readUART(NodeSerial);
  //firebaseSetString("ldr",data);
}

void dhtRq() {
  Serial.println("Requesting DHT...");
  String rq = "d";
  String data = "";
  sendUART(NodeSerial, rq);
  data = readUART(NodeSerial);
  //firebaseSetString("ldr",data);
}

void dustRq() {
  Serial.println("Requesting DUST...");
  String rq = "D";
  String data = "";
  sendUART(NodeSerial, rq);
  data = readUART(NodeSerial);
  //firebaseSetString("ldr",data);
}

void sendUART(SoftwareSerial &NodeSerial, String &str) {
  char p[str.length()+1];
  int i;
  for (i = 0; i < str.length()+1; i++) {
  if ((int) str[i] < 127) p[i] = str[i];
  }
  Serial.print("sendUART as ");
  Serial.println(p);
  NodeSerial.write(p);
//  while (!NodeSerial.available()) {
//    
//  }
}

String readUART(SoftwareSerial &NodeSerial) {
  char result[15]="";
  int idx = 0;
  // Read all bytes
  while (NodeSerial.available() > 0) {
    int x = NodeSerial.read();
    if (x<32 || x>126) {
      continue;
    }
    char xx = x;
    result[idx] = x;
    idx++;
  }

  result[idx]='\0';
  Serial.print("readUART returns ");
  Serial.println(result);
  return result;
}

void firebaseSetString(String &key, String &str) {
  // set string value
  Firebase.setString(key, str);
  // handle error
  if (Firebase.failed()) {
    Serial.print("setting /sensor1 failed:");
    Serial.println(Firebase.error());  
    return;
  }
  delay(100);
}