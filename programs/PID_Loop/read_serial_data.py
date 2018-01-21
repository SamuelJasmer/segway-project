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
ax1 = plt.subplot(111)
line, = ax1.plot([], [])

ser = serial.Serial('COM4', 9600)

serialData = ""

error_f = 0
p_f = 0    
i_f = 0    
d_f = 0    
pid_f = 0 
tf = 0
ti = 0

def init1():
    #line.set_data([], [])
    #return line,
    return

def run_animation():
    
    ani = animation.FuncAnimation(fig, animate, init_func = init1, interval=100)
    plt.show()

def animate(i):
    global serialData
    global line
    global error_f
    global p_f    
    global i_f    
    global d_f    
    global pid_f  
    global tf
    global ti
    
    vars = serialData.split(',')
    print(vars)

    if len(vars) == 5:
        error_i    = vars[0]
        p_i        = vars[1]
        i_i        = vars[2]
        d_i        = vars[3]
        pid_i      = vars[4]
        ti += 1

        plot_lines(ti, tf, error_i, error_f, line,)

        error_f    = error_i
        p_f        = p_i
        i_f        = i_i
        d_f        = d_i
        pid_f      = pid_i
        tf         = ti

def plot_lines(xi, xf, yi, yf, line,):
    px = [xf,xi]
    py = [yf,yi]
    line.set_data(px, py)

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

