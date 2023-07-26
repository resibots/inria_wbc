import matplotlib.pyplot as plt
import numpy as np
import re

def file_reader(path):
    abs_path = "/home/pal/inria_wbc/build/"
    file_obs = open(abs_path+path+"/observations.dat","r")
    file_s = open(abs_path+path+"/samples.dat","r")
    dico_K = {"kx":[],"ky":[],"kz":[]}
    notes = []
    index = 0
    for line in file_obs:
        l = [word for word in re.split("[ \n]",line)]
        if l[0] == str(index):
            notes.append(-float(l[1]))
            index += 1
    
    index = 0
    for line in file_s:
        l = [word for word in re.split("[ \n]",line)]
        if l[0] == str(index):
            dico_K["kx"].append(float(l[1]))
            dico_K["ky"].append(float(l[2]))
            dico_K["kz"].append(float(l[3]))
            index += 1

    return (dico_K,notes)

dico_K,notes = file_reader("poe2.loria.fr_2023-07-26_16_24_37_1208975")
trials = np.linspace(1,len(notes),len(notes))

f,figures = plt.subplots(4,2)

figures[0,0].plot(trials,dico_K["kx"],"-r")
figures[0,0].set_title("Kx value per trial")

figures[1,0].plot(trials,dico_K["ky"],"-g")
figures[1,0].set_title("Ky value per trial")

figures[2,0].plot(trials,dico_K["kz"],"-b")
figures[2,0].set_title("Kz value per trial")

figures[3,0].plot(trials,notes,"-y")
figures[3,0].set_title("notes per trial")

plt.show()