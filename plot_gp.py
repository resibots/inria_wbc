import numpy as np
from matplotlib import pyplot as plt
import sys
import glob

def find(elt,list):
    for i in range(len(list)):
        if list[i] == elt:
            return i
    return len(list)

path = sys.argv[1]
list_gp = glob.glob(path+"/gp_*.dat")
print(list_gp)
n_rows_plot = int(len(list_gp)/3)+1
f,figures = plt.subplots(n_rows_plot,3)

inc_r = 0
inc_c = 0

for index in range(len(list_gp)-2):
    
    i = find(path+"gp_"+str(index)+".dat",list_gp)
    print(list_gp[i])
    
    data = np.loadtxt(list_gp[i], skiprows=1)
    samples = np.loadtxt(path+"/samples.dat", skiprows=1)
    observations = np.loadtxt(path+"/observations.dat", skiprows=1)

    figures[inc_r,inc_c].plot(data[:,0], data[:,1])
    figures[inc_r,inc_c].fill_between(data[:,0], data[:,1] - data[:,2], data[:,1] + data[:,2],alpha=0.2)
    figures[inc_r,inc_c].plot(samples[0:index + 3,1], observations[0:index + 3,1], 'o')
    # figures[inc_r,inc_c].ylim(-100, 120)
    # figures[inc_r,inc_c].savefig('gp.pdf')
    figures[inc_r,inc_c].set_title("plot for gp_"+str(index)+"'s results")
    # figures[inc_r,inc_c].set_ylim(-1,1)
    
    if inc_c < 2:
        inc_c += 1
    else:
        inc_c = 0
        inc_r += 1
        
plt.savefig('gp.pdf')

plt.show()