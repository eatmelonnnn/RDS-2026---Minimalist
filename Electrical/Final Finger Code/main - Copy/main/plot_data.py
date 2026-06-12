import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("step_data_splay_only.csv")

# Convert time to seconds
t = df["t_ms"] / 1000.0

plt.figure()

plt.plot(t, df["target1"], label="Splay Target")
plt.plot(t, df["pos1"], label="Splay Actual")

plt.plot(t, df["target2"], label="MCP Flexion Target")
plt.plot(t, df["pos2"], label="MCP Flexion Actual")

plt.plot(t, df["target3"], label="DIP Flexion Target")
plt.plot(t, df["pos3"], label="DIP Flexion Actual")

plt.xlabel("Time (s)")
plt.ylabel("Joint Angle (rad)")
plt.title("All Joint Responses")
plt.legend()
plt.grid()

plt.show()



# Motor 1
plt.figure()
plt.plot(t, df["targetm1"], label="Target")
plt.plot(t, df["am1"], label="Actual")
plt.title("Splay Motor Response")
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.legend()
plt.grid()
plt.show()

# Motor 2
plt.figure()
plt.plot(t, df["targetm2"], label="Target")
plt.plot(t, df["am2"], label="Actual")
plt.title("MCP Motor Response")
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.legend()
plt.grid()
plt.show()

# Motor 3
plt.figure()
plt.plot(t, df["targetm3"], label="Target")
plt.plot(t, df["am3"], label="Actual")
plt.title("DIP Motor Response")
plt.xlabel("Time (s)")
plt.ylabel("Position (rad)")
plt.legend()
plt.grid()
plt.show()