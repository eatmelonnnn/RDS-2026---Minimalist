import serial

PORT = "COM4"          # change to your port
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

with open("flex_ext_log_final.csv", "w") as csvfile:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        print(line)
        csvfile.write(line + "\n")
        csvfile.flush()