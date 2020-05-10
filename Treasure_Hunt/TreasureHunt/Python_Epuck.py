# -*- coding: utf-8 -*-
"""
Created on Wed May  6 17:32:17 2020

@author: fanny.borza
"""
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import serial
import struct
import sys
import signal
import time
from threading import Thread

goodbye = """
                   /\_/\\
                 =( °w° )=
                   )   (  //
                  (__ __)//
 _____                 _ _                _ 
|  __ \               | | |              | |
| |  \/ ___   ___   __| | |__  _   _  ___| |
| | __ / _ \ / _ \ / _` | '_ \| | | |/ _ \ |
| |_\ \ (_) | (_) | (_| | |_) | |_| |  __/_|
 \____/\___/ \___/ \__,_|_.__/ \__, |\___(_)
                                __/ |       
                               |___/        
"""

#constants to compute the plot



def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)

#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader_thd.plot_updated()

#returns a sinus in an np.array of float64
#def do_sinus(freq, amp): 
#    m = np.linspace(0, n, num=(fs*n))
#    sinus = amp*np.sin(2*np.pi*freq*m)
#    sinus = sinus[0:n]
#    return sinus

#returns the norm of the values of the FFT performed on the dataset given 
#an n.array of float64
#def do_fft(array): 
#    FFT = np.fft.fft(array)
#    FFT_norme = np.sqrt(np.add(np.multiply(np.real(FFT),np.real(FFT)),(np.multiply(np.imag(FFT),np.imag(FFT)))))
#    return FFT_norme

#function used to update the plot of the sinus
#def update_sinus_plot(val):
#    amp = samp.val
#    freq = sfreq.val
#    
#    sinus = do_sinus(freq,amp)
#    
#    sinus_plot.set_ydata(sinus)
#                       
#    graph_sinus.relim()
#    graph_sinus.autoscale()
#
#    reader_thd.tell_to_update_plot()
    

#function used to update the plot 
def update_position_plot(port):

    fft_data = readFloatSerial(port) #here need to make an append instead of equal 
    x,y position 
    SoundDirection
    TravelDirection
    ObstacleDirection          
    
    if(len(fft_data)>0):
        fft_plot.set_ydata(fft_data)
        
        graph_fft.relim()
        graph_fft.autoscale()

        reader_thd.tell_to_update_plot()

#reset the sinus plot
def reset(event):
    sfreq.reset()
    samp.reset()

#sends the data of the sinus to the serial port in int16
def sendFloatSerial(port):
    data = (position_plot.get_ydata()).astype(np.int16)

    #to convert to int16 we need to pass via numpy
    size = np.array([data.size], dtype=np.int16)

    send_buffer = bytearray([])

    i = 0
    while(i < size[0]):
        send_buffer += struct.pack('<h',data[i])
        i = i+1

    port.write(b'START')
    port.write(struct.pack('<h',2*size[0]))
    port.write(send_buffer)
    print('sent !')

#reads the values in float32 from the serial
def readFloatSerial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size*4)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == 4*size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<f',rcv_buffer, i*4))
            i = i+1

        print('received !')
        return data
    else:
        print('Timout...')
        return []

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = False
        self.contSendAndReceive = False
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))
        
        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        
        while(self.alive):
            if(self.contSendAndReceive):
                sendFloatSerial(self.port)
                update_fft_plot(self.port)

            elif(self.contReceive):
                update_fft_plot(self.port)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceive(self, val):  
        self.contSendAndReceive = False
        self.contReceive = True

    #disables the continuous reading
    #and enables the continuous sending and receiving
    def setContSendAndReceive(self, val):
        self.contSendAndReceive = True
        self.contReceive = False

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self, val):
        self.contSendAndReceive = False
        self.contReceive = False

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)
    
#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

#figure config
fig, ax = plt.subplots(num=None, figsize=(10, 8), dpi=80)
fig.canvas.set_window_title('Noisy plot')
plt.subplots_adjust(left=0.1, bottom=0.25)
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

#sinus graph config with initial plot
graph_position = plt.subplot(211)
sinus = do_sinus(f0,A)
position_plot, = plt.plot(sinus, lw=1, color='red')


#timer to update the plot from within the state machine of matplotlib
#because matplotlib is not thread safe...
timer = fig.canvas.new_timer(interval=50)





#starts the matplotlib main
plt.show()