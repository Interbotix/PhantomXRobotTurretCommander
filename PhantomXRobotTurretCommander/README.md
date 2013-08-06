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