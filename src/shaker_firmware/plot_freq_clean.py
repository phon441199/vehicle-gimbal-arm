import csv, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = r"c:\Users\chang\OneDrive\문서\PlatformIO\Projects\servo_test\imu_capture.csv"
t, acc = [], []
with open(path, encoding="utf-8-sig") as f:
    for row in csv.DictReader(f):
        try: t.append(float(row["t"])); acc.append(float(row["accAC"]))
        except: pass
t = np.array(t); acc = np.array(acc)

# 1) 기동 과도구간 제거 (정상 진동만)
m = t > 2.5
t, acc = t[m], acc[m]

# 2) 균일 그리드 재샘플 + 평균 제거
fs = 50.0
tu = np.arange(t[0], t[-1], 1/fs)
au = np.interp(tu, t, acc); au -= au.mean()

# 3) 윈도우 + 제로패딩 FFT (매끄러운 스펙트럼)
win = np.hanning(len(au)); aw = au * win
N = 1 << 14
A = np.abs(np.fft.rfft(aw, N)); A /= A.max()
fr = np.fft.rfftfreq(N, 1/fs)
band = (fr > 0.5) & (fr < 6)
f0 = fr[band][np.argmax(A[band])]
T0 = 1.0 / f0

# ---- plot ----
plt.rcParams.update({"font.size": 12, "axes.grid": True, "grid.alpha": 0.25})
fig, ax = plt.subplots(2, 1, figsize=(9, 6.4))
ACC = "#1f6fb2"

# (a) time series (steady)
ax[0].plot(t, acc, color=ACC, lw=1.4)
ax[0].set_xlim(t[0], t[0] + 5)          # 5초만 보여 깔끔하게
ax[0].set_xlabel("time  [s]"); ax[0].set_ylabel("vertical accel  [g]")
ax[0].set_title("(a)  Vertical acceleration of the plate", loc="left", fontsize=12.5, fontweight="bold")
# 한 주기 표시
x0 = t[0] + 0.5
ax[0].annotate("", xy=(x0+T0, 0), xytext=(x0, 0),
               arrowprops=dict(arrowstyle="<->", color="#444", lw=1.4))
ax[0].text(x0 + T0/2, 0.04, f"T ≈ {T0*1000:.0f} ms", ha="center", fontsize=11, color="#444")

# (b) spectrum
ax[1].plot(fr, A, color=ACC, lw=1.6)
ax[1].plot(f0, 1.0, "o", color="#d1422f", ms=7)
ax[1].annotate(f"f₀ ≈ {f0:.1f} Hz", xy=(f0, 1.0), xytext=(f0+0.5, 0.85),
               fontsize=13, fontweight="bold", color="#d1422f",
               arrowprops=dict(arrowstyle="-", color="#d1422f", lw=1))
ax[1].set_xlim(0, 6); ax[1].set_ylim(0, 1.08)
ax[1].set_xlabel("frequency  [Hz]"); ax[1].set_ylabel("normalized amplitude")
ax[1].set_title("(b)  Amplitude spectrum (FFT)", loc="left", fontsize=12.5, fontweight="bold")

fig.suptitle(f"Excitation frequency from IMU:  f₀ ≈ {f0:.1f} Hz  ({f0*60:.0f} RPM)",
             fontsize=14.5, fontweight="bold")
plt.tight_layout(rect=[0, 0, 1, 0.96])
out = r"c:\Users\chang\OneDrive\문서\PlatformIO\Projects\servo_test\imu_freq_clean.png"
plt.savefig(out, dpi=200, facecolor="white")
print(f"f0 = {f0:.3f} Hz, T0 = {T0*1000:.1f} ms -> {out}")
