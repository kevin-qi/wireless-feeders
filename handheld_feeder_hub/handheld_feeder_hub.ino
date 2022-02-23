// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 40
#define CSN_PIN 53
#define TTL_PIN 3 // Interrupt

const byte feeder_hub_rx_0[10] = {'F','e','e','d','e','r','H','u','b'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[15]; // this must match dataToSend in the TX (15 bytes)
bool newData = false;
long newDataTimestamp;
bool loggingReady = false;

volatile long ttl_timestamp;
volatile long ttl_available;

//===========

void setup() {
    Serial.begin(115200);
    Serial.println("9,HUB,,"+String(millis()));

    attachInterrupt(digitalPinToInterrupt(TTL_PIN), ttlEvent, RISING);
   
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.openReadingPipe(1, feeder_hub_rx_0);
    radio.setChannel(100);
    radio.setPayloadSize(sizeof(dataReceived));
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();

    
}

//=============

void loop() {
    getData();
    showData();
    if(ttl_available){
      char ttlChar[8];
      sprintf(ttlChar, "%08ld",ttl_timestamp);
      Serial.println("9,TTL,,"+String(ttlChar));
      ttl_available = false;
    }
}

//==============

void getData() {
    if ( radio.available() ) {
        newDataTimestamp = millis();
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        //Serial.print("Data received ");
        char timestampChar[8];
        sprintf(timestampChar, "%08ld",newDataTimestamp);
        Serial.println(dataReceived+String(",")+String(timestampChar));
        newData = false;
    }
}

void ttlEvent() {
  ttl_available = true;
  ttl_timestamp = millis();
}
