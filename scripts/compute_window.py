import numpy as np
import scipy.signal.windows as windows

def compute_hft90d_coefficients(N):
    # HFT90D parameters
    a = [1, 1.942604, 1.340318, 0.440811, 0.043097]
    return windows.general_cosine(N, a, sym=False)

N = 1024  # Or whatever size your FFT is
window = compute_hft90d_coefficients(N)

# Scale down the window coefficients to avoid overflow
scale_factor = 3276.7  # About 1/10th of int16_t range
scaled_window = window * scale_factor

# Convert to Q15 format
q15_window = scaled_window.astype(np.int16)

# Print out the values for C
for value in q15_window:
    print(str(value) + ",", end="")

