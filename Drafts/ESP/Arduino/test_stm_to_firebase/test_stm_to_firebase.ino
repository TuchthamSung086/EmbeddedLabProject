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

int n = 0;

void loop() {
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
    // set string value
    Firebase.setString("sensor1", str);
    // handle error
    if (Firebase.failed()) {
      Serial.print("setting /sensor1 failed:");
      Serial.println(Firebase.error());  
      return;
    }
    delay(100);
  }

}

void setupESP() {
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  Serial.begin(115200);
  NodeSerial.begin(115200);
  
  Serial.println("Done setup!");
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
    if (x==0) break;
    strcat(str,&xx);
    //Serial.print(xx);
  }
  if (found && strlen(str)>0) {
    Serial.print("Data from NodeSerial is ");
    Serial.println(str);
  }
}
