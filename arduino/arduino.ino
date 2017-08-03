

#include "PheenoV2Basic.h"
#include <Adafruit_NeoPixel.h>

// Instantiate Pheeno class
PheenoV2Basic my_robot = PheenoV2Basic(1);

// Constant Variables:
#define NUMPIXELS      8
#define PIN            13
//Motor vaiables

float Linear_vel=40;  // in cm/sec
float right_angle = -1.14;     // in rad/sec
float left_angle = 1.14;
float r = 1.6;        // Wheel radius in cm
float b = 13.37;      // axel length in cm


float range_to_avoid = 25.0;
float compare_time;
float attach_dist = 17.0;  // 7.0;
float attach_dist_close = 10.0;
float detach_dist = 11.0;
float attach_side_dist = 12.0;
float move_delay = 3500;
float turn_delay = 2000;
float turn_away_delay = 1000;

// Dynamic Variables:


/* Encoder Matching PID Vairables */
//PID gains for encoder matching
float Kp=0.19;
float Ki=0.00;//35;//0.0000001;
float Kd=0.18;//1.45;//3.80;

// PID variables
float previous_error=0;
float error=0;
float integ_error=0;
float diff_error=0;


float PIDMotorsTimeStart = 0;

float timeStep=100;
int Total_error = 0;
int LeftDesVel = 0;
int RightDesVel = 0;

String in_data;
String mess_from_serial;
String mess_error;
String prev_signal;
bool is_color_shown = false;
bool is_attached = false;
bool serial_input_complete = false;

int forward_speed = 100;


// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  Serial.begin(9600);
  my_robot.SetupBasic();
  compare_time = millis();
  pixels.begin(); // This initializes the NeoPixel library.

// For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

for(int i=0;i<NUMPIXELS;i++)
{

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(100); // Delay for a period of time (in milliseconds).
  }

  for(int i=7;i>=-1;i--)
{

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(100); // Delay for a period of time (in milliseconds).
  }
      

  
  

}


void loop() {
  // Random turn left or right.

  SerialEvent();

  }



// Turns the Pheeno left.
// Currently the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnLeft(int speed) {
  my_robot.reverseLR(speed);
  my_robot.forwardRL(speed);
//  for(int i=0;i<NUMPIXELS;i++){
//
//      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//      pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(1, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(2, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(3, pixels.Color(255,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(4, pixels.Color(255,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(5, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(6, pixels.Color(0,100,0)); // Moderately bright green color.
//      pixels.setPixelColor(7, pixels.Color(0,150,0)); // Moderately bright green color.
//      pixels.show(); // This sends the updated pixel color to the hardware.
//      delay(100); // Delay for a period of time (in milliseconds).
//    }  
//   
}


// Turns the Pheeno Right.
// Currently, the appropriate use is just by providing a speed between 0-255,
// without any error handling. Be careful!
void PheenoTurnRight(int speed) {
  my_robot.reverseRL(speed);
  my_robot.forwardLR(speed);

//  for(int i=0;i<NUMPIXELS;i++){
//
//      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//      pixels.setPixelColor(0, pixels.Color(0,100,0)); // Moderately bright green color.
//      pixels.setPixelColor(1, pixels.Color(0,150,0)); // Moderately bright green color.
//      pixels.setPixelColor(2, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(3, pixels.Color(255,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(4, pixels.Color(255,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(5, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(6, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.setPixelColor(7, pixels.Color(0,0,0)); // Moderately bright green color.
//      pixels.show(); // This sends the updated pixel color to the hardware.
//      delay(100); // Delay for a period of time (in milliseconds).
//    }  
//   

}


// Moves the Pheeno Forward.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveForward(int speed,String error) {

  //if (millis() - PIDMotorsTimeStart >= timeStep){
  float PIDTimeStep = (millis() - PIDMotorsTimeStart)/1000.0;//Time step for controller to work on (s).
  float  a = 0.29;  
  my_robot.encoderPositionUpdate(timeStep);
  int  error1 = error.toInt();

  error1 = error1*(a)+previous_error*(1-a);

  
  
  integ_error +=  error1 * PIDTimeStep;
  
  diff_error = (error1-previous_error)/ PIDTimeStep;
  
  previous_error = error1;
  
  Total_error = (error1*Kp)+(integ_error*Ki)+(diff_error*Kd);
  
  Total_error = (Total_error >=150 ? Total_error = 150 :Total_error <=-150 ? Total_error= -150:Total_error=Total_error); 
  
  LeftDesVel = speed+Total_error;
  
  RightDesVel = speed-Total_error;
  
  Serial.print("  totak error:  ");
  Serial.print(Total_error);
  Serial.print("  Pos L:  ");
  Serial.print(LeftDesVel);
  Serial.print("  Pos R:  ");
  Serial.println(RightDesVel);
  


  my_robot.forwardLR(LeftDesVel);
  my_robot.forwardRL(RightDesVel);


  PIDMotorsTimeStart = millis();
    //}
  for(int i=0;i<NUMPIXELS;i++){

      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(0, pixels.Color(0,25,0)); // Moderately bright green color.
      pixels.setPixelColor(1, pixels.Color(0,25,0)); // Moderately bright green color.
      pixels.setPixelColor(2, pixels.Color(0,0,0)); // Moderately bright green color.
      pixels.setPixelColor(3, pixels.Color(255,0,0)); // Moderately bright green color.
      pixels.setPixelColor(4, pixels.Color(255,0,0)); // Moderately bright green color.
      pixels.setPixelColor(5, pixels.Color(0,0,0)); // Moderately bright green color.
      pixels.setPixelColor(6, pixels.Color(0,20,0)); // Moderately bright green color.
      pixels.setPixelColor(7, pixels.Color(0,20,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      delay(100); // Delay for a period of time (in milliseconds).
    }  
  

}


// Moves the Pheeno Reverse.
// Applying a specific speed value (0-255) and both motors will apply the speed
// without error handling. Be careful!
void PheenoMoveReverse(int speed) {
  my_robot.reverseLR(speed);
  my_robot.reverseRL(speed);

  for(int i=0;i<NUMPIXELS;i++){

      // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(0, pixels.Color(100,0,0)); // Moderately bright green color.
      pixels.setPixelColor(1, pixels.Color(100,0,0)); // Moderately bright green color.
      pixels.setPixelColor(2, pixels.Color(0,2,0)); // Moderately bright green color.
      pixels.setPixelColor(3, pixels.Color(255,0,0)); // Moderately bright green color.
      pixels.setPixelColor(4, pixels.Color(255,0,0)); // Moderately bright green color.
      pixels.setPixelColor(5, pixels.Color(0,0,0)); // Moderately bright green color.
      pixels.setPixelColor(6, pixels.Color(100,0,0)); // Moderately bright green color.
      pixels.setPixelColor(7, pixels.Color(100,0,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      delay(100); // Delay for a period of time (in milliseconds).
    }  
  

}


// Reads serial communication information, if present.
// ADD MORE INFORMATION HERE.
void SerialEvent() {
  while (Serial.available()) {
    char in_char = (char)Serial.read();  // Get the new byte.
    in_data += in_char;                  // Add it to the input string.

    // If the incoming character is a newline, set a flag so the main loop can
    // do something about it.
    if (in_char == ':') {
      // serial_input_complete = true;
      // Serial.println(in_data);
      ParseData();
      //Serial.println(in_data);
      in_data = "";
     

    }

  }
  in_data="";
  my_robot.brakeAll();  
  //Serial.println(in_data);
}


// Parses the incoming data specifically for incoming Picam information.
// ADD MORE INFORMATION HERE.
// Parses the incoming data specifically for incoming Picam information.
// ADD MORE INFORMATION HERE.
void ParseData() 
{
 // Serial.println(in_data);
  if(in_data!=""){
    
    if (in_data.indexOf(':') >= 0) 
    {
      mess_from_serial = in_data.substring(0, in_data.indexOf(";"));
      mess_error = in_data.substring((in_data.indexOf(";")+1), in_data.indexOf(":"));
  
    my_robot.readIR();
   int distance = my_robot.CDistance;
   //Serial.println(distance);
    if (distance > range_to_avoid)
    {
      
//      if (mess_from_serial == "GREEN") 
//        { 
//            Serial.println (" GREEN detected");
//           int turn_state = random(1,4);
//            if (turn_state == 1)
//            {
//               //Need to turn the Pheeno Left.
//                
//                Serial.println("turnleft");
//                delay(1000);
//                PheenoMoveForward(forward_speed,0);
//                delay(4600);
//                PheenoTurnLeft(120);
//                delay(1800);
//                Serial.println("turnleft done");
//                //delay(2000);            
//                my_robot.brakeAll();
//                delay(1000);
//                PheenoMoveForward(forward_speed,mess_error);
//                prev_signal=="GREEN";
//   
//            } 
//            if(turn_state == 2)
//            {
//                
//                Serial.println("turnRight");
//                delay(1000);
//                PheenoMoveForward(forward_speed,0);
//                delay(3600);
//                PheenoTurnRight(120);
//                delay(1800);
//                Serial.println("turnRight done");
//                //delay(2000);            
//                my_robot.brakeAll();
//                delay(1000);
//                PheenoMoveForward(forward_speed,mess_error);
//                prev_signal=="GREEN";
//            }
//            if(turn_state == 3)
//            {
//              // Need to turn the Pheeno Right.
//              Serial.println("LaneFollow");
//              PheenoMoveForward(forward_speed,mess_error);                 
//              prev_signal=="GREEN";
//                
//            }
//        }
//       else 
//      if (mess_from_serial == "RED")
//      {
//        Serial.println (" RED detected");
//        my_robot.brakeAll();   
//        prev_signal="RED"; 
//        Serial.println (prev_signal);
//         
//      }
//        else 
//      if (mess_from_serial == "REVERSE")
//      {
//          Serial.println("REVERSE No Signal Detected");
//         my_robot.reverseLR(80);
//         my_robot.reverseRL(80);
//         delay(500);
//      }
//      else
      if(mess_from_serial != "PHEENO" && mess_error != "None")
      {
        Serial.println("nocolor detected");
        PheenoMoveForward(forward_speed,mess_error);
        
      }
//      else
//      if(mess_from_serial != "PHEENO" && mess_error =="None")
//      {
//        Serial.println("nothing detected");       
//        my_robot.brakeAll();
//        for(int i=0;i<NUMPIXELS;i++){
//
//            // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//            pixels.setPixelColor(i, pixels.Color(0,20,0)); // Moderately bright green color.
//            pixels.show(); // This sends the updated pixel color to the hardware.
//            delay(1); // Delay for a period of time (in milliseconds).
//           }    
      }
    } 
    
//      else 
//      if (mess_from_serial == "PHEENO")
//      {
//         Serial.println("Pheeno detected");       
//         my_robot.brakeAll();
//      }
//    else 
//    if(mess_from_serial == "PHEENO" && distance<30 && distance >22)
//    {
//         Serial.print(distance);
//         Serial.println("REVERSE Pheeno");
//         my_robot.reverseLR(100);
//         my_robot.reverseRL(100);
//         delay(100);
//    }
//    else
//    if (my_robot.CDistance < 25)
//    {
//      int distance = my_robot.CDistance;
//      Serial.println ("Too Close to wall");
//      Serial.println(distance);
//    //erial.println(my_robot.CDistance);
////    PheenoTurnLeft(100);
////    delay(2350);    
////    my_robot.forwardLR(150);
////    my_robot.forwardRL(150);
////    delay(1250);    
////    PheenoTurnLeft(100);
////    delay(2320);
//    my_robot.brakeAll();
//    //delay(1000);  
//    }
   }
  else
  {
        Serial.println (" Nothing");
        my_robot.brakeAll();  
  }
  
}



















