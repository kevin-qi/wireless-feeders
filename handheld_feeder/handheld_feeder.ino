#include <Fsm.h>
#include <Bounce2.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define DEBUG true  //set to true for debug output, false for no debug output
#define DSerial if(DEBUG)Serial

#define isNotConnected_PIN 13 // HIGH if not connected, LOW if connected
#define CONTROL_INPUT_PIN 3
#define RETRACT_INPUT_PIN 5

#define CE 9
#define MISO 12
#define MOSI 11
#define SCK 13
#define CSN 10

//#define BB_READY_PIN A0
#define BB_STIM_PIN A0

#define EVENT_RESET_READY 10
#define EVENT_READY_REW 20
#define EVENT_REW_READY 30
#define EVENT_READY_RETRACT 40
#define EVENT_RETRACT_RESET 50

#define IN1_PIN A5
#define IN2_PIN A4

int val_control_input;
String err = "";

long init_time; // Timestamp enter ready state (immediately after making radio contact with hub)
long prev_ttl_time = 0;
long cur_ttl_time = 0;
bool ttl_available = false;

RF24 radio(CE, CSN); // CE, CSN
const int rf_channel = 100;
/*  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Change ID below before uploading for the appropriate feeder.
*/
const int FEEDER_ID = 0;
//const int FEEDER_ID = 1;
//Setting the two addresses. One for transmitting and one for receiving

/* 
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
const byte rx_addresses[][12] = {"HandFeeder0", "HandFeeder1"};
const byte tx_address[10] = {'F','e','e','d','e','r','H','u','b'}; 

boolean button_stateA = 0;
boolean button_stateB = 0;
long timestamp = 12321;
char payload[15];

bool isConnected = 0;

// renamed
Bounce bounce_control_pin = Bounce();
Bounce bounce_retract_pin = Bounce();
//Bounce bounce_BB_STIM = Bounce();

State state_reset(&on_reset_enter, NULL, &on_reset_exit);
State state_ready(&on_ready_enter, NULL, &on_ready_exit);
State state_rew(&on_rew_enter, NULL, &on_rew_exit);
State state_retract(&on_retract_enter, NULL, &on_retract_exit);

Fsm fsm(&state_reset);

void on_ready_enter() {
 DSerial.println("READY_ENTER:"+String(millis())+"|");
 err = "";
}

void on_ready_exit() {
  DSerial.println("READY_EXIT:"+String(millis())+"|");
}

void on_retract_enter() {
 DSerial.println("RETRACT_ENTER:"+String(millis())+"|");
 digitalWrite(IN1_PIN, LOW);
 digitalWrite(IN2_PIN, HIGH);
}

void on_retract_exit() {
  DSerial.println("RETRACT_EXIT:"+String(millis())+"|");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void on_rew_enter() {
  DSerial.println("REW_ENTER:"+String(millis())+"|");
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  radioWrite("REW", millis()); // Timestamp of reward
}

void on_rew_exit() {
  DSerial.print("REW_EXIT:"+String(millis())+"|");
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void on_reset_enter() {
  DSerial.print("RESET:"+String(millis())+"|"+"ERR:"+err+"\n");
}

void on_reset_exit() {
  init_time = millis();
  radioWrite("BEG", init_time); // Timestamp of beginning of local 3 sec ttls
}

void on_rew_to_ready_timed(){
  
}


void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);

 //pinMode(RESET_STATE_PIN, OUTPUT);
 pinMode(IN1_PIN, OUTPUT);
 pinMode(IN2_PIN, OUTPUT);
 pinMode(isNotConnected_PIN, OUTPUT);

 digitalWrite(isNotConnected_PIN, HIGH); // Turn LED ON to indicate not connected to radio

 radio.begin();                            //Starting the radio communication
 radio.setChannel(rf_channel);
 DSerial.println(String((char *)tx_address));
 radio.openWritingPipe(tx_address);      //Setting the address at which we will send the data
 //radio.openReadingPipe(1, rx_addresses[FEEDER_ID]);   //Setting the address at which we will receive the data
 radio.setPALevel(RF24_PA_LOW);
 radio.setDataRate( RF24_250KBPS );
 radio.setPayloadSize(sizeof(payload));
 radio.setRetries(3,5); // delay, count
 radio.stopListening(); // Set as TX;
 //radio.printDetails();
 
 //pinMode(BB_READY_PIN, INPUT);
 //bounce_BB_STIM.attach(BB_STIM_PIN, INPUT_PULLUP);
 //bounce.attach(BB_SNIFF_PIN, INPUT);
 //bounce.attach(BB_REW_PIN, INPUT);
 //bounce_BB_STIM.interval(500);

 bounce_control_pin.attach(CONTROL_INPUT_PIN, INPUT_PULLUP);
 bounce_control_pin.interval(75);

 bounce_retract_pin.attach(CONTROL_INPUT_PIN, INPUT_PULLUP);
 bounce_retract_pin.interval(3000);
 
 fsm.add_transition(&state_reset, &state_ready,
                    EVENT_RESET_READY,
                    NULL);
 fsm.add_transition(&state_ready, &state_retract,
                    EVENT_READY_RETRACT,
                    NULL);
 fsm.add_transition(&state_retract, &state_reset,
                    EVENT_RETRACT_RESET,
                    NULL);
 fsm.add_transition(&state_ready, &state_rew,
                    EVENT_READY_REW,
                    NULL);   
            

 fsm.add_timed_transition(&state_rew, &state_ready,
                          100,
                          &on_rew_to_ready_timed);
 
 while(!isConnected){
  fsm.run_machine();
  unsigned long start_timer = micros();                    // start the timer
  isConnected = radioWrite("RDY", millis());
  unsigned long end_timer = micros();                      // end the timer
  if(isConnected){
    fsm.trigger(EVENT_RESET_READY);
    DSerial.print(F("Transmission successful! "));          // payload was delivered
    DSerial.print(F("Time to transmit = "));
    DSerial.print(end_timer - start_timer);                 // print the timer result
    DSerial.print(F(" us. Sent: "));
    DSerial.println(payload);                               // print payload sent

    // Following code block sets up 3 second TTL outputs
    cli();//stop interrupts
    //set timer1 interrupt at 0.33333Hz (1/3 Hz)
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1/3hz increments
    OCR1A = 46874;// = (16*10^6) / ((1/3)*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    digitalWrite(isNotConnected_PIN, LOW); // Turn LED OFF to indicate connected to radio
    
  } else {
    DSerial.println(F("Transmission failed or timed out")); // payload was not delivered
    delay(1000);
  }
 }
}

void loop() {
 if(isConnected){ // Only run FSM if radio has been established.
   fsm.run_machine();
   if(ttl_available){ // Time to send TTL
    radioWrite("TTL", cur_ttl_time);
    ttl_available = false;
   }
   
   bounce_control_pin.update();
   bounce_retract_pin.update();
   
   if(bounce_control_pin.changed()){
     val_control_input = digitalRead(CONTROL_INPUT_PIN);
     
     if(val_control_input == LOW){ // Button is pressed
      fsm.trigger(EVENT_READY_REW);
     }
   }
  
   if(bounce_retract_pin.changed()){
     int val_retract_input = digitalRead(CONTROL_INPUT_PIN);
     DSerial.println(val_retract_input);
     if(val_retract_input == LOW){ // Button is pressed
      fsm.trigger(EVENT_READY_RETRACT);
     }
   }
   if(digitalRead(CONTROL_INPUT_PIN) == HIGH){
    fsm.trigger(EVENT_RETRACT_RESET);
   }
 }
 
/*
 if(Serial.available()){
  char inByte = Serial.read();
  fsm.trigger(EVENT_RESET_READY);  
 }*/
}

ISR(TIMER1_COMPA_vect){ //timer1 interrupt 0.3333Hz toggles pin 13 (LED) (3 second TTL)
  // Set cur ttl time during interupt. This is the most accurate 3000 ms according to arduino clock
  cur_ttl_time = millis();
  // Set ttl_available flag to true so that arduino can
  // send cur_ttl_time via radio on next loop (could potentially
  // be many milliseconds late, but the ttl timestamp has already been
  // saved during this interupt.
  ttl_available = true;
}

bool radioWrite(String three_char_token, long ms){
  bool res = 0;
  if(three_char_token.length() == 3){
    sprintf(payload,"%d:%s:%08ld",FEEDER_ID,three_char_token,ms);
    res = radio.write(&payload, sizeof(payload));
  }
  return res;
}
