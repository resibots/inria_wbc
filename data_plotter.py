from matplotlib import pyplot as plt
import numpy as np
import re

def plot_data_from_file(file_path):
    dic = {"x":[],"y":[],"z":[],"valid":[]}
    file = open(file_path,"r")
    for line in file:
        l = [word for word in re.split("[ \n]",line)]
        dic["x"].append(float(l[1]))
        dic["y"].append(float(l[3]))
        dic["z"].append(float(l[5]))
        dic["valid"].append(int(l[7]))
        #print(l)
    return dic

def get_power_of_two(nb):
    power = 0
    temp = nb
    while (temp != 0):
        temp = temp//2
        power += 1
    print(2**(power))
    return power

##main code ####
parsed_left_data = plot_data_from_file("/home/pal/inria_wbc/vive_left_data_sample1.csv")
parsed_right_data = plot_data_from_file("/home/pal/inria_wbc/vive_right_data_sample1.csv")

#number of samples collected
n_data = len(parsed_left_data["x"])
#print(f"number of samples: {n_data}")
n_fft = 2**get_power_of_two(n_data)

#sampling frequency
fs = 100

#total duration
T = n_data/fs

#fft frequency
fft_fs = n_fft/T

#time
t = np.linspace(0,T,n_data)

### plot figures ########################################################################
f,figures = plt.subplots(4,2)

figures[0,0].plot(t,parsed_left_data["x"],"-r")
figures[0,0].set_title("plot left x")

figures[1,0].plot(t,parsed_left_data["y"],"-b")
figures[1,0].set_title("plot left y")

figures[2,0].plot(t,parsed_left_data["z"],"-g")
figures[2,0].set_title("plot left z")

figures[3,0].plot(t,parsed_left_data["valid"],"-y")
figures[3,0].set_title("plot left valid")

figures[0,1].plot(t,parsed_right_data["x"],"-r")
figures[0,1].set_title("plot right x")

figures[1,1].plot(t,parsed_right_data["y"],"-b")
figures[1,1].set_title("plot right y")

figures[2,1].plot(t,parsed_right_data["z"],"-g")
figures[2,1].set_title("plot right z")

figures[3,1].plot(t,parsed_right_data["valid"],"-y")
figures[3,1].set_title("plot right valid")

plt.show()

#just to wait until we continue and plot the ffts
#cont = int(input("press 1 to continue: "))
##########################################################################################

#stock variables for the next, easier
l_x = parsed_left_data["x"]
l_y = parsed_left_data["y"]
l_z = parsed_left_data["z"]
r_x = parsed_right_data["x"]
r_y = parsed_right_data["y"]
r_z = parsed_right_data["z"]

#calculate the ffts of the signals
freq = np.fft.fftfreq(n_data,1/fs)
f = np.fft.fftshift(freq)
fft_lx = np.fft.fft(l_x,n_data)
fft_lx = 1/fs*np.fft.fftshift(fft_lx)