import serial
import csv
import time


port = "COM4"
baud = 115200

freq = 0.5                  
num_periods = 6
duration = num_periods / freq   # seconds



ser = serial.Serial(port, baud, timeout=1)

start_time = time.time()

with open("step_data_cool.csv", "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "t_ms",
        "target1", "pos1",
        "target2", "pos2",
        "target3", "pos3",
        "targetm1", "am1",
        "targetm2", "am2",
        "targetm3", "am3"

    ])

    while True:
        if time.time() - start_time > duration:
            break

        try:
            line = ser.readline().decode().strip()
            values = [float(x) for x in line.split(",")]

            writer.writerow(values)

        except:
            continue

print("Done logging 6 periods")

