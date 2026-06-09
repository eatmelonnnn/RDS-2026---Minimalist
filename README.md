# Minimalist Finger


## Instructions to Run Finger Program/Operate Finger Once Assembled

1. Locate main folder in the Electrical and upload `main.ino` to the Teensy 4.1.

2. Let calibration sequence run without interference.

3. Choose desired behavior/test.
   - For all states except the flexion-extension test, simply press the specified button to trigger the state switch.
   - You can switch between any two states by pressing the corresponding buttons.
   - For the MCP flexion-extension test:
     - Move the jumper wire on Pin 33 from **GND** to **3.3V**.
     - This will trigger the flexion-extension test.
     - At this point, there is no way to return to the other states without rerunning the program from the start on the Teensy.

4. When finished, press the **E-stop** to turn off the motors.
