import   _thread
import   matplotlib.pyplot as plt 
import   matplotlib.animation as animation
from     mpl_toolkits.mplot3d import Axes3D
from     matplotlib import style 
import   serial
import   time
import   math

#plt.style.use('fivethirtyeight')

fig = plt.figure()
ax1 = plt.subplot(111, projection='polar')
linex, = ax1.plot([], [])
liney, = ax1.plot([], [])
linez, = ax1.plot([], [])

#ax1 = fig.gca(projection='3d')

#X_axis = ax1.plot([],[],[])
#Y_axis = ax1.plot([],[],[])
#Z_axis = ax1.plot([],[],[])



ser = serial.Serial('COM4', 9600)

serialData = ""

def init1():
    linex.set_data([], [])
    liney.set_data([], [])
    linez.set_data([], [])
    return linex, liney, linez,

def init2():
    X_axis.set_data([],[],[])
    Y_axis.set_data([],[],[])
    Z_axis.set_data([],[],[])
    return X_axis, Y_axis, Z_axis


def run_animation():
    
    ani = animation.FuncAnimation(fig, animate_2D, init_func = init1, interval=100)
    plt.show()
        
def animate_3D(i):
    global serialData
    global X_axis
    global Y_axis
    global Z_axis

    vars = serialData.split(',')
    print(vars)

    if len(vars) == 6:
        mag_x = vars[0]
        mag_y = vars[1]
        mag_z = vars[2]
    
        mag_theta_x = vars[3]
        mag_theta_y = vars[4]
        mag_theta_z = vars[5]
    
        ax1.set_rorigin(0)


        linex, = plot_lines(mag_theta_x , mag_x, X_axis)
        liney, = plot_lines(mag_theta_y , mag_y, Y_axis)
        linez, = plot_lines(mag_theta_z , mag_z, Z_axis)
       
        return linex, liney, linez,

def animate_2D(i):
    global serialData
    global linex
    global liney
    global linez

    vars = serialData.split(',')
    print(vars)

    if len(vars) == 6:
        mag_x = vars[0]
        mag_y = vars[1]
        mag_z = vars[2]
    
        mag_theta_x = vars[3]
        mag_theta_y = vars[4]
        mag_theta_z = vars[5]
    
        ax1.set_rorigin(0)


        X_axis = plot_lines(mag_theta_x , mag_x, linex)
        Y_axis = plot_lines(mag_theta_y , mag_y, liney)
        Z_axis = plot_lines(mag_theta_z , mag_z, linez)
       
        return linex, liney, linez,



def plot_lines(mtheta, mr, line,):

    theta = [0, math.radians(float(mtheta))]
    r = [0, abs(float(mr))]
    line.set_data(theta, r)

    return line,

def read_serial():
    global serialData
    while True:
        data = ser.readline()
        serialData = data.decode('ascii').strip()
 
try:
    print('Start Thread 1')
    _thread.start_new_thread(read_serial, ())
    run_animation()
except:
    print("unable to start thread")

