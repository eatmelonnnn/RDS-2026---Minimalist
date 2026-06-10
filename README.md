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

# Assembly Instructions

## 1. Finger Assembly

1. Put bearings into flexion pulleys (3x).
   > ![Step 1](images/finger1.png)
2. Put bearings and pulleys onto bearing pins (2x).
   > ![Step 2a](images/finger2a.png)
   > ![Step 2b](images/finger2b.png)
3. Tie slipknots onto spring/tendon pins (2x).
4. Snap spring/tendon pin into fingertip and pull tendon into groove.
   > ![Step 4](images/finger4.png)
5. Snap bearing pin into place.
   > ![Step 5](images/finger5.png)
6. Snap middle linkage onto bearing pin.
   > ![Step 6](images/finger6.png)
7. Route tendon through and pull into groove.
   > ![Step 7](images/finger7.png)
8. Snap second tendon pin on and pull tendon into groove.
   > ![Step 8](images/finger8.png)
9. Snap second bearing pin into middle link.
   > ![Step 9](images/finger9.png)
10. Snap knuckle onto bearing pin.
    > ![Step 10](images/finger10.png)
11. Thread tendons through knuckle and onto grooves.
    > ![Step 11](images/finger11.png)
12. Put bearings into splay pulleys (2x).
    > ![Step 12](images/finger12.png)
13. Put belt pulley onto shaft.
    > ![Step 13](images/finger13.png)
14. Align all splay components and push shaft through.
    > ![Step 14](images/finger14.png)
15. Wrap tendons around splay pulleys and thread through base.
    > ![Step 15](images/finger15.png)
16. Put springs onto pins and snap onto linkages (x2).
    > ![Step 16](images/finger16.png)
17. Gently pry up tendon/spring pins and loop springs on (x2).
    > ![Step 17](images/finger17.png)

---

## 2. Mounting Assembly

1. Assemble main 80-20 frame with plates and bolts.
   > ![Step 1](images/mounting1.png)
2. Put vertical 80-20 on with brackets and bolts.
   > ![Step 2](images/mounting2.png)
3. Put pulley on splay motor.
   > ![Step 3](images/mounting3.png)
4. Put splay motor on splay plate.
   > ![Step 4](images/mounting4.png)
5. Put tensioner on splay plate.
   > ![Step 5](images/mounting5.png)
6. Put splay plate on vertical 80-20 with brackets and bolts.
   > ![Step 6](images/mounting6.png)
7. Put finger plate on vertical 80-20 with bolts.
   > ![Step 7](images/mounting7.png)
8. Put tension plates on 80-20 with bolts (2x).
   > ![Step 8](images/mounting8.png)
9. Put motor plates on 80-20 with bolts (2x).
   > ![Step 9](images/mounting9.png)
10. Put pulleys on flexion motors (2x).
    > ![Step 10](images/mounting10.png)
11. Put tension sensors on tension plates (2x).
    > ![Step 11](images/mounting11.png)
12. Put motors on motor plates (2x).
    > ![Step 12](images/mounting12.png)

---

## 3. Finger/Mounting Integration Assembly

1. Bolt finger onto plate and put splay belt on.
   > ![Step 1](images/all1.png)
2. Tension splay belt with allen key.
   > ![Step 2](images/all2.png)
3. Thread tendon from finger through tension sensor and onto pulley.
   > ![Step 3](images/all3.png)
4. Terminate tendon on motor pulley with slipknot and screw.