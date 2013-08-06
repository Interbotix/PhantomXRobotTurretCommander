 /* 
   PhantomX Robot Turret Code - ArbotiX Commander Compatible
  
   This code is designed for the PhantomX Robot Turret(AX-12A and AX-18A compaitble) and enables them to communicate
   with an ArbotiX Commander over an XBee link.
   XBees must be paired and set to 38400 baud rate.     
   https://www.trossenrobotics.com/p/phantomX-robot-turret.aspx
   http://www.trossenrobotics.com/p/arbotix-commander-gamepad-v2.aspx 
   
   Thise sketch can also be used with the Virtual Commander software
   https://github.com/trossenrobotics/Virtual_Commander
   
   *ARBOTIX COMMANDER CONTROLS **********
   *Left Joystick
   **Control the pan and tilt servos at a higher speed. Pressing the left/right top buttons allows for speed control.
   **Left-Right = Pan     
   **Up-Down = Tilt
   *
   *Right Joystick
   **Control the pan and tilt servos at a slower, finer speed.
   **Left-Right = Pan       
   **Up-Down = Tilt
   *
   *Left Pushbuttons (L4-L6)
   **Set the tilt servo to 3 predetermined positions, the lower limit, defualt position, and upper limit
   *
   *Right Pushbuttons(R1-R3)
   **Set the pan servo to 3 predetermined positions, the lower limit, defualt position, and upper limit
   *
   *Left Top Button
   **Modify the tilt speed of the left joystick
   **Each button press will slow the speed, eventually wrapping around and restoring full speed.
   *  
   *Right Top Button   
   **Modify the pan speed of the left joystick
   **Each button press will slow the speed, eventually wrapping around and restoring full speed.
   ********************* 
  ArbotiX Firmware - Commander Extended Instruction Set Example
  Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 
 
 
//define pan and tilt servo IDs
#define PAN    1
#define TILT   2

// the H101 'C' bracket attached to the tilt servo creates a physical limitation to how far
// we can move the tilt servo. This software limit will ensure that we don't jam the bracket into the servo.
#define TILT_UPPER_LIMIT 768 
#define TILT_LOWER_LIMIT 256

//Upper/Lower limits for the pan servo - by defualt they are the normal 0-4095 (0-360) positions for the servo
#define PAN_UPPER_LIMIT 1023 
#define PAN_LOWER_LIMIT 0

//Default/Home position. These positions are used for both the startup position as well as the 
//position the servos will go to when they lose contact with the commander
#define DEFAULT_PAN 512
#define DEFAULT_TILT 512

#define WALKDEADBAND 20 //the 'walk' joystick (left) has a range from -128 to 127. Define a deadband(-20/+20) to reduce drift/stutter.

#define HOME_ON_DISCONNECT false //set to 'true' and the turret will return to the defualt position when it loses contact with the commander. Set to 'false' and the turret will remain in the position it was last in 
#define ARBOTIX_TIMEOUT  1000      // time(ms) after the commander stops sending messages for the turret to return to a defult position.

 
//Include necessary Libraries to drive the DYNAMIXEL servos and recieve commander instructions  
#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>


/* Hardware Constructs */
BioloidController bioloid = BioloidController(1000000);  //create a bioloid object at a baud of 1MBps
Commander command = Commander();

int pan;    //current position of the pan servo
int tilt;   //current position of the tilt servo  
int panMod =1;  //modifier for pan speed on left joystick - increase this to reduce the turret's speed
int tiltMod =1;  //modifier for tilt speed on left joystick - increase this to reduce the turret's speed



boolean turretActive = false;   // Is the turret on/receieivng commands - used to set the turret to a defualt position if it loses contact with the commander
unsigned long   lastMsgTime;    // Keep track of when the last message arrived to see if controller off

boolean leftPrev = false;      //left top button previous state
boolean rightPrev = false;     //right top button previous state

void setup(){
  
  // setup LED
  pinMode(0,OUTPUT);
  
  // setup serial for commander communications 
  Serial.begin(38400);
  
  // setup interpolation, slowly raise turret to a 'home' positon. 2048 are the 'center' positions for both servos
  pan = DEFAULT_PAN;
  tilt = DEFAULT_TILT;
  
  delay(100);//short delay
  
  bioloid.poseSize = 2;//2 servos, so the pose size will be 2
  bioloid.readPose();//find where the servos are currently
  bioloid.setNextPose(PAN,pan);//prepare the PAN servo to the centered position, pan/2048
  bioloid.setNextPose(TILT,tilt);//preprare the tilt servo to the centered position, tilt/2048
  bioloid.interpolateSetup(2000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
  while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
  {
    bioloid.interpolateStep();//move servos 1 'step
    delay(3);
  }
  
  lastMsgTime = millis();    // log time, to calculate  disconnect time
}
 
void loop(){

  // try to read a command from commander
  if(command.ReadMsgs() > 0)
  {
      turretActive = true;//if a command has been received, 'activate' the turret
      
      // toggle LEDs
      digitalWrite(0,HIGH-digitalRead(0));
      
      //set pan /tilt based on walk/left joystick, by defualt moves quickly
      //the walk/left joystick needs a deadband around the center because it is issuing finer movements. Without a deadband
      //the turret will stutter and drift
      //create a deadband around the left joystick by comapring the absolute value (command.walk's range is -128 to 127) to the deadband
      if(abs(command.walkH) > WALKDEADBAND)
      { 
      pan +=  - command.walkH/panMod;//use 'panMod' to adjust the speed of the pan servo
      }
      if(abs(command.walkV) > WALKDEADBAND)
      {
      tilt -=  - command.walkV/tiltMod;//use 'tiltMod' to adjust the speed of the tilt servo
      }
      
      //set pan/tilt based on right joystick, move turret slowly
      pan += - command.lookH/20;
      tilt -= - command.lookV/20;
  
    
      //set pre-defined positions based on pushbuttons R1-L6
      if(command.buttons & BUT_R1)
      {
        pan = PAN_LOWER_LIMIT;
      }
      else if(command.buttons & BUT_R2)
      {
        pan = DEFAULT_PAN;
      }
      else if(command.buttons & BUT_R3)
      {
        pan = PAN_UPPER_LIMIT;
      }
      else if(command.buttons & BUT_L4)
      {
         tilt = TILT_UPPER_LIMIT;
      }
      else if(command.buttons & BUT_L5)
      {
        tilt = DEFAULT_TILT;
      }
      else if(command.buttons & BUT_L6)
      {
         tilt = TILT_LOWER_LIMIT;
      }
      
      //the RT(Right Top/right shoulder) button controls the speed for pan of the left joystick
      else if((command.buttons & BUT_RT)&& rightPrev == false)
      {
        rightPrev = true; //set prevoius state to true, so that 1 button press is will only change the speed 1 level
        panMod = panMod + 5;//increase modifier reduces the speed of the left joystick
        if (panMod > 50)
        {
          panMod = 1;//if the modifier becomes to high, reset modifier to 1
        }
        
      }
      
      //the LT(left top/left shoulder) button controls the speed for tilt of the left joystick
      else if((command.buttons & BUT_LT)&& leftPrev == false)
      {
        leftPrev = true;//set prevoius state to true, so that 1 button press is will only change the speed 1 level
        tiltMod = tiltMod + 5;//increase modifier reduces the speed of the left joystick
        if (tiltMod > 50)
        {
          tiltMod = 1;//if the modifier becomes to high, reset modifier to 1
        }
        
      }
      
      //set the previous state of the right top button, so that one button press moves 1 speed level
      if(!(command.buttons & BUT_RT))
      {
        rightPrev = false;
      }
  
      //set the previous state of the left top button, so that one button press moves 1 speed level
      if(!(command.buttons & BUT_LT))
      {
        leftPrev = false;
      }
      
      
      //enforce upper/lower limits for tilt servo
      if (tilt < TILT_LOWER_LIMIT)
      {
        tilt =TILT_LOWER_LIMIT;
      }  
    
      else if (tilt > TILT_UPPER_LIMIT)
      {
        tilt =TILT_UPPER_LIMIT;
      }
    
    
      //enforce upper/lower limits for pan servo
      if (pan < PAN_LOWER_LIMIT)
      {
        pan =PAN_LOWER_LIMIT;
    
      }  
    
      else if (pan > PAN_UPPER_LIMIT)
      {
        pan =PAN_UPPER_LIMIT;
    
      }
      
      //send pan and tilt goal positions to the pan/tilt servos 
      SetPosition(PAN,pan);
      SetPosition(TILT,tilt);    
      
      lastMsgTime = millis();    // log the current time of the last command succesfully received and executed
  }    
  
  
  else //no commands from commander
  {

      // check to see if...
      //HOME_ON_DISCONNECT is 'true', meaning the turret should return to the home position after it stops receiving commands
      //turretActive is 'true' meaning the turret has received commands in the past
      //the program has exceeded the timeout, ARBOTIX_TIMEOUT milliseconds, i.e. the turret has not received a command in that time period
      // If these have all occured, set the turret to its home position
      if (HOME_ON_DISCONNECT &&(turretActive) && ((millis() - lastMsgTime) > ARBOTIX_TIMEOUT)) 
      {
        turretActive = false;//deactivate turret
        //set defualt servo positions
        pan = DEFAULT_PAN;
        tilt = DEFAULT_TILT;
        
        delay(100);//short delay
        bioloid.readPose();//find where the servos are currently
        bioloid.setNextPose(PAN,pan);//prepare the PAN servo to the centered position, pan/2048
        bioloid.setNextPose(TILT,tilt);//preprare the tilt servo to the centered position, tilt/2048
        bioloid.interpolateSetup(2000);//setup for interpolation from the current position to the positions set in setNextPose, over 2000ms
        while(bioloid.interpolating > 0)  //until we have reached the positions set in setNextPose, execute the instructions in this loop
        {
          bioloid.interpolateStep();//move servos 1 'step
          delay(3);
        }
      }
      
  
  }

}

