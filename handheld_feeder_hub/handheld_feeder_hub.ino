// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 40
#define CSN_PIN 53

const byte thisSlaveAddress[10] = {'F','e','e','d','e','r','H','u','b'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[15]; // this must match dataToSend in the TX (15 bytes)
bool newData = false;
bool loggingReady = false;

//===========

void setup() {
    Serial.begin(115200);
    Serial.println("9,HUBSTART,,"+String(millis()));
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.setChannel(100);
    radio.setPayloadSize(sizeof(dataReceived));
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
    
}

//=============

void loop() {
    getData();
    showData();
}

//==============

void getData() {
    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        //Serial.print("Data received ");
        Serial.println(dataReceived+String(",")+String(millis()));
        newData = false;
    }
}
