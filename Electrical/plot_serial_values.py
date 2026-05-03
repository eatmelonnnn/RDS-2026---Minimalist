import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = 'COM5'       # change to your port
BAUD = 115200
MAX_POINTS = 400

ser = serial.Serial(PORT, BAUD)

pos1, pos2, pos3 = deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS)
target1, target2, target3 = deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS), deque(maxlen=MAX_POINTS)

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('Motor Position Tracking')

# One subplot per motor
labels = ['Motor 1', 'Motor 2', 'Motor 3']
actual_lines = []
target_lines = []

for i, ax in enumerate(axes):
    t_line, = ax.plot([], [], '--', label=f'Target', color='gray')
    a_line, = ax.plot([], [], label=f'Actual', color=['blue','green','red'][i])
    ax.set_ylabel(labels[i])
    ax.set_ylim(-1, 1)
    ax.legend(loc='upper right')
    ax.grid(True)
    actual_lines.append(a_line)
    target_lines.append(t_line)

axes[-1].set_xlabel('Samples')

def update(frame):
    try:
        line = ser.readline().decode().strip()
        # expects: "T1:0.12\tA1:0.12\tT2:0.34\tA2:0.34\tT3:0.56\tA3:0.56"
        parts = dict(p.split(':') for p in line.split('\t'))

        target1.append(float(parts['T1']))
        pos1.append(float(parts['A1']))
        target2.append(float(parts['T2']))
        pos2.append(float(parts['A2']))
        target3.append(float(parts['T3']))
        pos3.append(float(parts['A3']))

        x = range(len(pos1))
        actual_lines[0].set_data(x, pos1)
        target_lines[0].set_data(x, target1)
        actual_lines[1].set_data(x, pos2)
        target_lines[1].set_data(x, target2)
        actual_lines[2].set_data(x, pos3)
        target_lines[2].set_data(x, target3)

        for ax in axes:
            ax.set_xlim(0, len(pos1))

    except Exception as e:
        pass

    return actual_lines + target_lines

ani = animation.FuncAnimation(fig, update, interval=10, blit=True)
plt.tight_layout()
plt.show()