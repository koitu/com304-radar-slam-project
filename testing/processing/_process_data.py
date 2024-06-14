import numpy as np
import scipy
import scipy.io as sio
from scipy import signal


### start helper functions ###


def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff, btype='high', analog=False)
    return b, a
def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = signal.filtfilt(b, a, data)
    return y


def beamform(X, locs):
    """
    Performs beamforming on the input data.

    Parameters:
    - X: Input data of shape (# frames x # Rx x # Tx x ADC samples).

    Returns:
    - sph_pwr: Beamforming output
    """

    # Perform FFT on the input data
    X = X[0:-1:2,:,:]
    locs = locs[:,0:-1:2,:]
    beat_freq = scipy.fft.fft(X, axis=2)

    # Define index parameters
    idxs = np.arange(0,20,1) #np.arange(0,X.shape[2])

    # Define limits for theta and phi
    theta_s, theta_e = 40, 140
    theta_s *= (np.pi/180)
    theta_e *= (np.pi/180)
    theta_rad_lim = [theta_s,theta_e]
    d_theta = 1/180*np.pi

    # Generate arrays for theta and phi
    theta = np.arange(theta_rad_lim[0], theta_rad_lim[1], d_theta)
    N_theta = len(theta)
    lm = 3e8/77e9

    # Initialize arrays for spherical power
    sph_pwr = np.zeros((N_theta, len(idxs)), dtype=complex)
    x_idx = np.array(np.squeeze(locs[0,:,:]))

    print("Running this many iterations: %d " % (N_theta))

    # Perform beamforming
    for kt in range(N_theta):
            sin_theta = np.sin(theta[kt])
            Vec = np.exp(1j*(2*np.pi*(x_idx*sin_theta)/lm)) 
            VecRFI = np.repeat(Vec[:, :, np.newaxis], len(idxs), axis=2)
            sph_pwr[kt, :] = np.squeeze(np.sum(np.multiply(beat_freq[:, :, idxs], VecRFI), axis=(0, 1)))
            print("Processed angle (%.2f)" % (theta[kt]*180/np.pi), end='\r')

    return sph_pwr

# file = sio.loadmat("bf_data.mat")
# 
# sph_pwr = beamform(file['a'], file['locs_z_original'])
# sio.savemat('bf_output15.mat',{"sph_pwr": sph_pwr})

### end helper functions ###



