from numpy.lib.function_base import unwrap
from mmwave.dataloader import DCA1000
import numpy as np
import time
from scipy.fftpack import fft
import processing.utility as utility

# def producer_real_time_1843(q, index, lua_file):
#     """
#     This function is one of the processed called in realtime_streaming.py.
#     Paramters:
#     - q: Queue to push new data
#     - index: dummy index
#     - lua_file: lua file containing radar chirp paramters

#     This function reads data from the ethernet port connected to the radar, reformats the data, and pushes the range fft to q.
#     Note: This function can be called in parallel with other functions (see realtime_streaming.py for more details). 
#     """
#     num_rx, num_tx, samples_per_chirp, periodicity, num_frames, chirp_loops, data_rate, freq_plot_len, range_plot_len = utility.read_radar_params(lua_file) 
#     dca = DCA1000()

#     # chirp configurations based on your config lua file
#     dca.sensor_config(chirps=num_tx*chirp_loops, chirp_loops=1, num_rx=num_rx, num_samples=samples_per_chirp)
#     prev_time = time.time()

#     while True:
#         # here we actually reads from the ethernet port
#         adc_data = dca.read()

#         # print(f"adc: {adc_data.shape}")
#         # (12288,)

#         # reorganize the interleaved data into channels corresponding to (tx*chirp loops, rx, adc samples)
#         # number of frames is 1 because we are always reading the 
#         org_data = dca.organize(
#             raw_frame=adc_data,
#             num_chirps=num_tx*chirp_loops,
#             num_rx=num_rx,
#             num_samples=samples_per_chirp,
#             num_frames=1,
#             model='1843')
#         x = 0

#         # print(f"org: {org_data.shape}")
#         # (3, 4, 512)
        
#         # here we are just processing the range fft to plot later on
#         x = np.sum(org_data, axis=(0, 1))
#         x -= np.mean(x, axis=-1, keepdims=True)
#         fx = fft(x, axis=-1)
#         afx = np.squeeze(np.abs(fx))

#         now = time.time()
#         # So as to not overload the updating, we will just refresh the data every 0.1 seconds
#         if now - prev_time > 0.1:
#             # put the tuple containing the data's name and the data into the queue that will be sent to plotting (see realtime_streaming.py)
#             q.put(["rfft", afx])
#             prev_time = now


def producer_real_time_1843(q, index, lua_file):
    num_rx, num_tx, samples_per_chirp, periodicity, num_frames, chirp_loops, data_rate, freq_plot_len, range_plot_len = utility.read_radar_params(lua_file) 
    dca = DCA1000()

    # chirp configurations based on your config lua file
    dca.sensor_config(chirps=num_tx*chirp_loops, chirp_loops=1, num_rx=num_rx, num_samples=samples_per_chirp)
    prev_time = time.time()


    lm = 3e8/77e9 # define lambda for the antenna spacing

    # define rx pos
    rx_pos = np.arange(1, 5, dtype=float)
    rx_pos = rx_pos * -lm/2
    rx_pos = rx_pos - rx_pos[0]

    # define tx offsets
    x_pos = np.arange(1, 4, dtype=float)
    x_pos = x_pos * lm
    x_pos = x_pos - x_pos[0]

    # define virtual ant positions
    ant_pos = np.reshape(np.array([rx_pos + x_pos[i] for i in range(len(x_pos))]), (-1,1))
    # print(ant_pos/lm)


    theta_s, theta_e = 0, 180
    theta_res = 1
    theta = np.arange(
        theta_s * (np.pi/180),
        theta_e * (np.pi/180),
        theta_res * (np.pi/180))
    rfi = np.exp(1j * (2 * np.pi * (ant_pos * np.cos(theta))/lm))[:, :, np.newaxis]  # I have no idea what to name this


    while True:
        # THE WAY THIS PRODUCER IS SNYCONIZED WITH THE CONSUMER IS SHIT!!!
        
        time.sleep(0.2)

        # here we actually reads from the ethernet port
        adc_data = dca.read()

        # print(f"adc: {adc_data.shape}")
        # (12288,)

        # reorganize the interleaved data into channels corresponding to (tx*chirp loops, rx, adc samples)
        # number of frames is 1 because we are always reading the 
        org_data = dca.organize(
            raw_frame=adc_data,
            num_chirps=num_tx*chirp_loops,
            num_rx=num_rx,
            num_samples=samples_per_chirp,
            num_frames=1,
            model='1843')
        x = 0

        # print(f"org: {org_data.shape}")
        # (3, 4, 512)
        
        x = fft(org_data, axis=-1)
        x = np.reshape(x, (-1, 512))
        res = np.sum(np.multiply(x[:, np.newaxis, :], rfi), axis=0)

        # plot_lim_y = [10,200]
        # plot_lim_x = [0 - theta_s,179 - theta_s]
        res = res[0:179, 10:200] / np.max(res)
        res = abs(res[:,::-1]).T

        now = time.time()
        # So as to not overload the updating, we will just refresh the data every 0.1 seconds
        # if now - prev_time > 0.1:
        if now - prev_time > 1:
            # put the tuple containing the data's name and the data into the queue that will be sent to plotting (see realtime_streaming.py)
            q.put(["bf", res])
            prev_time = now