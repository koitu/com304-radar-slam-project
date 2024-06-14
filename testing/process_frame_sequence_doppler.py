import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import dsp
from matplotlib.animation import FuncAnimation


# load matlab file
data = sio.loadmat('record/rdc_casa2.mat')
data = data['data_raw']
print(f"Data shape: {data.shape}")

def animate(i):
    plt.cla()
    frame = data[i]
    radar_cube = dsp.range_processing(frame, window_type_1d=dsp.utils.Window.BLACKMAN)
    det_matrix, aoa_input = dsp.doppler_processing(radar_cube, num_tx_antennas=3, clutter_removal_enabled=True, window_type_2d=dsp.utils.Window.HAMMING)
    det_matrix_vis = np.fft.fftshift(det_matrix, axes=1)
    plt.imshow(det_matrix_vis / det_matrix_vis.max(),origin='lower',extent=[-1.334,1.334,0,15.366],aspect='auto')
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Range (m)")


fig = plt.figure()
ani = FuncAnimation(fig, animate,frames = 800, interval=33)

ani.save('casa2.mp4', writer = 'ffmpeg', fps = 30.303) 

    


