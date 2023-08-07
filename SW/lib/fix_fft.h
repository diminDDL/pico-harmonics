#ifndef FIX_FFT_H
#define FIX_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

int fix_fftr(short*, int, int);
int fix_fft(short fr[], short fi[], short m, short inverse);

#ifdef __cplusplus
}
#endif

#endif
