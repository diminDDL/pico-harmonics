import numpy as np
import matplotlib.pyplot as plt

# Compute FFT
data = [12.4, 11.0, 10.3]
fs = 500000  # Sampling frequency, adjust as needed
n = len(data)
k = np.arange(n)
T = n/fs
frq = k/T
frq = frq[range(n//2)]  # One side frequency range

Y = np.fft.fft(data)/n  # FFT computation and normalization
Y = Y[range(n//2)]

# Plot up to the 4th harmonic
plt.plot(frq,abs(Y))
plt.xlim([0, 100000])  # This will make the x-axis only show up to the 4th harmonic (assuming bins of 1kHz)
plt.title('FFT of data up to the 4th harmonic')
plt.xlabel('Freq (Hz)')
plt.ylabel('|Y(freq)|')
plt.grid(True)
plt.show()