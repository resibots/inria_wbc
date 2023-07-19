from matplotlib import pyplot as plt
import numpy as np
import re

def plot_data_from_file(file_path):
    dic = {"x":[],"y":[],"z":[],"hx":[],"hy":[],"hz":[],"valid":[]}
    file = open(file_path,"r")
    for line in file:
        l = [word for word in re.split("[ \n]",line)]
        dic["x"].append(float(l[1]))
        dic["y"].append(float(l[3]))
        dic["z"].append(float(l[5]))
        dic["hx"].append(float(l[7]))
        dic["hy"].append(float(l[9]))
        dic["hz"].append(float(l[11]))
        dic["valid"].append(int(l[13]))
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

def norm(x_list,y_list,z_list):
    res = []
    for i in range(len(x_list)):
        res.append(np.sqrt(x_list[i]**2 + y_list[i]**2 + z_list[i]**2))
    return res

def substract_lists(l1,l2):
    if len(l1) != len(l2):
        print("lists not of the same size, error. Plz give right sized lists as inputs")
    else:
        return [l1[i]- l2[i] for i in range(len(l1))]


##main code ####
parsed_left_data = plot_data_from_file("/home/pal/inria_wbc/vive_left_data_sample1.csv")
parsed_right_data = plot_data_from_file("/home/pal/inria_wbc/vive_right_data_sample1.csv")

#number of samples collected
n_data_l = len(parsed_left_data["x"])
n_data_r = len(parsed_right_data["x"])
#print(f"number of samples: {n_data}")

#data vectors lists
error_left = norm(substract_lists(parsed_left_data["x"],parsed_left_data["hx"]),substract_lists(parsed_left_data["y"],parsed_left_data["hy"]),substract_lists(parsed_left_data["z"],parsed_left_data["hz"]))
error_right = norm(substract_lists(parsed_right_data["x"],parsed_right_data["hx"]),substract_lists(parsed_right_data["y"],parsed_right_data["hy"]),substract_lists(parsed_right_data["z"],parsed_right_data["hz"]))

#sampling frequency
fs = 100

#total duration
Tr = n_data_r/fs
Tl = n_data_l/fs
#fft frequency


#time
tr = np.linspace(0,Tr,n_data_r)
tl = np.linspace(0,Tl,n_data_l)

### plot figures ########################################################################
f,figures = plt.subplots(4,6)

figures[0,0].plot(tl,parsed_left_data["x"],"-r")
figures[0,0].set_title("plot left x")

figures[1,0].plot(tl,parsed_left_data["y"],"-g")
figures[1,0].set_title("plot left y")

figures[2,0].plot(tl,parsed_left_data["z"],"-b")
figures[2,0].set_title("plot left z")

figures[3,0].plot(tl,parsed_left_data["valid"],"-y")
figures[3,0].set_title("plot left valid")

figures[0,1].plot(tl,parsed_left_data["hx"],"-r")
figures[0,1].set_title("plot left robot hand x")

figures[1,1].plot(tl,parsed_left_data["hy"],"-g")
figures[1,1].set_title("plot left robot hand y")

figures[2,1].plot(tl,parsed_left_data["hz"],"-b")
figures[2,1].set_title("plot left robot hand z")

figures[3,2].plot(tl,error_left,"-y")
figures[3,2].set_title("error left")

figures[0,2].plot(tl,substract_lists(parsed_left_data["x"],parsed_left_data["hx"]),"-r")
figures[0,2].set_title("error left x")

figures[1,2].plot(tl,substract_lists(parsed_left_data["y"],parsed_left_data["hy"]),"-g")
figures[1,2].set_title("error left y")

figures[2,2].plot(tl,substract_lists(parsed_left_data["z"],parsed_left_data["hz"]),"-b")
figures[2,2].set_title("error left z")

figures[0,3].plot(tr,parsed_right_data["x"],"-r")
figures[0,3].set_title("plot right x")

figures[1,3].plot(tr,parsed_right_data["y"],"-g")
figures[1,3].set_title("plot right y")

figures[2,3].plot(tr,parsed_right_data["z"],"-b")
figures[2,3].set_title("plot right z")

figures[3,3].plot(tr,parsed_right_data["valid"],"-y")
figures[3,3].set_title("plot right valid")

figures[0,4].plot(tr,parsed_right_data["hx"],"-r")
figures[0,4].set_title("plot right robot hand x")

figures[1,4].plot(tr,parsed_right_data["hy"],"-g")
figures[1,4].set_title("plot right robot hand y")

figures[2,4].plot(tr,parsed_right_data["hz"],"-b")
figures[2,4].set_title("plot right robot hand z")

figures[3,5].plot(tr,error_right,"-y")
figures[3,5].set_title("error right")

figures[0,5].plot(tr,substract_lists(parsed_right_data["x"],parsed_right_data["hx"]),"-r")
figures[0,5].set_title("error right x")

figures[1,5].plot(tr,substract_lists(parsed_right_data["y"],parsed_right_data["hy"]),"-g")
figures[1,5].set_title("error right y")

figures[2,5].plot(tr,substract_lists(parsed_right_data["z"],parsed_right_data["hz"]),"-b")
figures[2,5].set_title("error right z")

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

