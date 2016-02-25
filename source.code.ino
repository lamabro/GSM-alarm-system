







#include <Wire.h>

#include "LiquidCrystal.h"  // library for liquid crystal
LiquidCrystal lcd(0);// Connect via i2c, default address #0 (A0-A2 not jumpered)




#include <SoftwareSerial.h> // Library for connecting serial connection of GSM SHIELD
#include <String.h>
SoftwareSerial mySerial(2,3); // RX PIN, TX PIN


char  *keypressed = 0;
int keyboardPin = 0;    // Analog input pin that the keypad is attached to
int keyboardValue = 0;   // value read from the keyboard


int k;
//int inputPin1=4;
//int inputPin2=7;
#define LEDFLASHER 4 //Pin for LED
     
 // int inputPin1 = 5 ; 
//intinputPin2= 6 ; 
int pir1=12; //Pin for PIR SENSOR

int pinSpeaker=10; //PIN for Buzzer
int Alertfunction; // Function  when movement is detected

  int pirState = LOW;    // we start, assuming no motion detected
  int val = 0;           // variable for reading the pin status of PIR sensor
  
   int val1 = 1;            // variable for reading the pin status of pir 2
  


boolean status; // variable to stor the status of alarm;true for "ALARM ON" and False for "ALARM oFF"


char incoming_char; // Variable to store incoming message from the owner
String received;

//char pp[4];   
char pp[4]; // variable to store passwords


// digital password for alarm

char codeone[4];
char code[4];


void passwordcheckfunction(); // Function to enable for inputting the password and confirming the password and calling the functions respective to the passwords entered when alarm is off.

void redled();  // Function to blink led and make a small buzzer sound when needed.
void controlalramfromsms();  // Function to switch on or off the alarm from sms of owner.



void Alarmon();  // Function to implement when alarm is on. i.e to activate PIR sensor.
void Alarmoff();// Function when alarm is off . i.e to Deactivate the Pir sensor
void passwordcheckfunction1();// Function to enable for inputting the password and confirming the password and calling the functions respective to the passwords entered when alarm is on.
char *keypadreading(); // function to return the values from keypad

void smsowner(); // to send message to owner.

void calltheowner();// to call to owner
void alertfunction();// Function to be executed when movement is detected.
void playTone(); // Play the buzzer






void setup()
{

pp[0]='1'; // password element
pp[1]='2'; // password element
pp[2]='3'; // password element
pp[3]='4'; // password element
pp[4]='0'; // password element
Serial.begin(19200); // set the baud rate for serial communication for monitor
  mySerial.begin(19200); 
  // set the baud rate for serial communication
delay(1000);
 pinMode(LEDFLASHER, OUTPUT);      // declare LED as output
    //pinMode(inputPin1, INPUT); 
  //  pinMode(inputPin2, INPUT); // declare sensor as input
     pinMode(pir1, INPUT);      // declare Pir sensor as input
    pinMode(pir2, INPUT);     // declare Pir2 sensor as input
   pinMode(pinSpeaker, OUTPUT);   //declare buzzer as output


 lcd.begin(16, 2);
 
 Serial.print("alarm is off");
lcd.print("alarm is off");
delay(400);
lcd.clear();

delay(100);
 
  Alarmoff();


              // the GPRS baud rate   
 //Serial.begin(19200);   // Serial1,, Serial 2
            // the GPRS baud rate   

mySerial.print("AT+CMGF=1\r");                                                        // AT command to send SMS message
  delay(2000);

mySerial.print("ATE0\r");  // AT command to send SMS message
delay(5000);
mySerial.print("AT\r");  // AT command to send SMS message
delay(3000);  // AT command to send SMS message
mySerial.print("AT+CNMI=1,2,0,0,0\r");  // AT command to send SMS message
delay(3000);
lcd.begin(16,2);


  
  Alarmoff();

}
 

 

void redled()
{
digitalWrite(LEDFLASHER, HIGH);

 for (int m=0; m<70; m++);
 
        {
           int m;
            digitalWrite(pinSpeaker , m);
            delay(50);
        }
digitalWrite(LEDFLASHER, LOW);



}

void Alarmoff ()
{
 
status=false;
lcd.setCursor(0,2);
lcd.print("alarm is off");
Serial.print("alarm is off");
delay(1000);
lcd.clear();
digitalWrite(LEDFLASHER, LOW);//turn off the buzzer and led.
passwordcheckfunction();
}




  
   
void Alarmon()
{
  lcd.print("alram on");
status=true;
delay(2000);
loop();

Serial.print("Alarm activated");

passwordcheckfunction1();





}




void calltheowner()
{
delay(500);
 // dial US (212) 8675309
  
  mySerial.println("AT+COPS");  // AT COMMANDS TO CALL THE OWNER
  delay(1000);
  mySerial.println("ATD0403594439;"); // AT COMMANDS TO CALL THE OWNER of NUmber 0403594439
  delay(7000);
 mySerial.println();// AT COMMANDS TO CALL THE OWNER
  delay(4000);            // wait for 30 seconds...
  mySerial.println("ATH");   // hang up
  //mySerial.println("called");// set the data rate for the SoftwareSerial port
  lcd.clear();
  lcd.setCursor(0,0);
  delay(500);
  lcd.print("calling");
}

void smsowner()
{


  mySerial.print("AT+CMGF=1\r");                                                        // AT command to send SMS message
  delay(1000);
  mySerial.println("AT + CMGS = \"+358403594439\"");                                     // recipient's mobile number, in international format
  delay(500);

  mySerial.println("Intruder detected");        // message to send
  delay(500);
  mySerial.println((char)26);                       // End AT command with a ^Z, ASCII code 26
  delay(500); 
 
  mySerial.println();
  
  delay(5000);                                     // give module time to send SMS
                                     // turn off module
}





void playTone(long duration, int freq) {
  for(int i=0;i<5;i++){
    duration *= 100;                        // Play the tone for buzzer when movement is detected.
    int period = (1.0 / freq) * 1000000;
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        digitalWrite(pinSpeaker,HIGH);
        digitalWrite(LEDFLASHER, HIGH);
        
        delayMicroseconds(period / 2);
        digitalWrite(pinSpeaker, LOW);
        digitalWrite(LEDFLASHER, LOW);
        delayMicroseconds(period / 2);
        elapsed_time += (period);
        delay(100);
    }


    }
}














void alertfunction()
{
  int i,k;

lcd.clear();
lcd.setCursor(0,1);
delay(100);
lcd.print("Intruder");
lcd.setCursor(0,2);
lcd.print("Detected");
smsowner();
calltheowner();
delay(3000);
 // turn LED ON
    playTone(300, 160);
    delay(700);

delay(1000);

calltheowner();
delay(4000);

Alarmoff();


}





void passwordcheckfunction()    // Function to implement when status of alarm is off,

{
  char *p;
 

char password[4];
lcd.clear();
delay(100);
lcd.setCursor(0,0);
lcd.print("Alaram  off");
delay(1000);
Serial.print("Alarm off");
delay(2000);

int attempt;
k=keypad();
redled();
delay(500);
if (k>1)    // To setup the keypad for entering the password when any key is pressed on the keypad at first.

{


//keypad();
lcd.setCursor(0,0);
delay(1000);
lcd.clear();
lcd.print("Enter password");
Serial.print("Enter password");

delay(1000);
lcd.setCursor(0,1);


p=keypadreading(); // Return the password with single element from keypadreading funtion.



  if(*(p+0)==pp[0]&&*(p + 1)==pp[1]&&*(p + 2)==pp[2]&&*(p + 3)==pp[3])  // checking the password entered with the stored passwords
                {
                //space for four values values.

Alarmon();                     // Call alarmon function when password is correct otherwise it will go back to same original function passwordcheckfunction
delay(4000);
lcd.clear();
delay(1000);
lcd.print("Alarm is on;");

delay(100);
                }
else
{

   lcd.clear();

lcd.print("incorrect password");
     delay(1000);
      lcd.setCursor(0,1);


passwordcheckfunction();

}
}
}
/*
attempt=attempt++;

if(attempt ==3){

lcd.print("access denied");
redled();

}

}


}
}
*/


void passwordcheckfunction1()

{
  char *p;

char password[4];
lcd.clear();
delay(100);
lcd.setCursor(0,0);
lcd.print("still on.");
delay(1000);
Serial.print("still on");
delay(2000);

int attempt;
k=keypad();// Return the password with single element from keypadreading funtion.
redled();
delay(2000);
if (k>1)  // To setup the keypad for entering the password when any key is pressed on the keypad at first.

{


//keypad();
lcd.setCursor(0,0);
delay(1000);
lcd.clear();
lcd.print("Enter the password");
Serial.print("Enter the password");

delay(1000);
lcd.setCursor(0,1);


p=keypadreading();



  if(*(p+0)==pp[0]&&*(p + 1)==pp[1]&&*(p + 2)==pp[2]&&*(p + 3)==pp[3])   // checking the password entered with the stored passwords
                {
                //space for four values values.

Alarmoff();
lcd.clear();
delay(1000);
lcd.print("Alarm is off;");

delay(100);
                }
else
{

   lcd.clear();

lcd.print("incorrect password");
     delay(1000);
      lcd.setCursor(0,1);


passwordcheckfunction();

}
}
}




char keypad( )

{

    keyboardValue = analogRead((A0)); // read the value (0-1023)
/*Serial.print("Pin value:");
Serial.println( keyboardValue);
*/
while (keyboardValue < 220){  
   //do nothing until a key is pressed
   keyboardValue = analogRead(A0); 
   delay(50);
 }



if ((keyboardValue >864) && (keyboardValue < 889)){keypressed = "C"; delay(50);}
else if
  ((keyboardValue >760) && (keyboardValue < 780)){keypressed = "B";delay(50);}
  else if
    ((keyboardValue >666) && (keyboardValue < 688)){keypressed = "0";delay(50);}
    else if
   ((keyboardValue >551) && (keyboardValue < 625)){keypressed = "A";delay(50);}
   else if
  ((keyboardValue >520) && (keyboardValue < 542)){keypressed = "D";delay(50);}
  else if
   ((keyboardValue >478) && (keyboardValue < 500)){keypressed = "9";delay(50);}
    else if ((keyboardValue >440) && (keyboardValue < 460)){keypressed = "8";delay(50);}
   else if ((keyboardValue >400) && (keyboardValue < 435)){keypressed = "7";delay(50);}
   else if ((keyboardValue >352) && (keyboardValue < 389)){keypressed = "E";delay(50);}
   else if ((keyboardValue >332) && (keyboardValue < 347)){keypressed = "6";delay(50);}
   else  if ((keyboardValue >317) && (keyboardValue < 326)){keypressed = "5";delay(50);}
   else if ((keyboardValue >299) && (keyboardValue < 308)){keypressed = "4";delay(50);}
   else  if ((keyboardValue >274) && (keyboardValue < 283)){keypressed = "F";delay(50);}
    else if ((keyboardValue >262) && (keyboardValue < 269)){keypressed = "3";delay(50);}
   else if ((keyboardValue >247) && (keyboardValue < 256)){keypressed = "2";delay(50);}
    else if ((keyboardValue >235) && (keyboardValue < 248)){keypressed = "1";delay(50);}
    else {keypressed="10";}
//NOTE: the values used above are all halfway between the value obtained with each keypress in previous test sketch

Serial.println(keypressed); // print the value back to the Serial view window on your PC
delay(1000); 
return *keypressed;

    
  }


























char *keypadreading( )
{
char password[4];
for(int i=0;i<=3;i++)
{
  
//keyboardValue=analogRead(A0);


//redled(); with small sound

password[i]=keypad();
redled();

delay(100); 
lcd.print("*");
delay(120);
Serial.print(password[i]);
delay(300);

}
return password;
delay(300);
}



  void loop(){



while(status==true)    //check the status of alarm and act accordingly
{
val = digitalRead(pir1); // variable for defining the detection distance.
//val1 = analogRead(pir2);    // read input value, variable1 for defining the detection distance.
    if (val == HIGH){            // check if the input is HIGH i.e movement is detected.
  alertfunction();                  // call function alertfunction if movement is detected.

  
}

}
    keyboardValue = analogRead(A0); // read the keyboard value (0 - 1023)
while (keyboardValue < 220){
//do nothing until a key is pressed
keyboardValue = analogRead(A0);
delay(50);
}

     

//lcd light is on for 20 seconds;if any key is pressed LCD screen brightness is on for 15 seconds.


if(mySerial.available() >0)
  {
    
  
    incoming_char=mySerial.read(); //Get the character from the cellular serial port from message.
        received+=incoming_char;
    Serial.print(incoming_char); //Print the incoming character to the terminal.





  if (received.compareTo("on") == 0)    // compare the received message with on and turn on the device
{
Alarmon;  }

  if (received.compareTo("off") == 0)   // compare the received message with off and turn off the device
  
{Alarmoff;
}




    
    
    
  }








 
 }



