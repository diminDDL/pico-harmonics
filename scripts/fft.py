import numpy as np
import matplotlib.pyplot as plt
import serial

# Setup serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)
print("Serial connection established.")

# Read data from serial
def read_serial_data():
    data = []

    # Wait for the delimiter
    print("Waiting for delimiter...")
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if "====" in line:
            print("Delimiter received!")
            break

    # Now start reading data after delimiter
    print("Reading data...")
    while True:
        line = ser.readline().decode('utf-8').strip()
        if "===" in line:
            print("End of data detected!")
            break
        print(f"Received line: {line}")  # Print every line received for inspection
        freq, amplitude = map(float, line.split(','))
        data.append((freq, amplitude))
    return data

# Set up the plot
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_xlim([0, 100000])
ax.set_ylim([1, 100000])
plt.xticks(np.arange(0, plt.xlim()[1]+5000, 5000))
ax.set_yscale('symlog', linthresh=1e-10)
ax.set_title('FFT of data')
ax.set_xlabel('Freq (Hz)')
ax.set_ylabel('|Y(freq)|')
ax.grid(True)

# Update function to refresh the plot
def update_plot(data):
    frq = [item[0] for item in data]
    Y = [item[1] for item in data]

    line.set_data(frq, Y)
    plt.draw()
    plt.pause(0.1)

try:
    while True:
        data = read_serial_data()
        update_plot(data)

        # Check if the Matplotlib window is still open
        if not plt.fignum_exists(fig.number):
            break

finally:
    ser.close()  # Close the serial connection in case of exceptions or breaks
    print("Serial connection closed.")
    plt.ioff()  # Turn interactive mode off
    plt.show()  # Keep the plot window open
