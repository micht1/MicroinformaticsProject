# -*- coding: utf-8 -*-
"""
@author: fanny.borza
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.widgets import Button
import matplotlib.pyplot as plt
import serial
import struct
import sys
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
#Definition variable 
NewData=[]


def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)
    
#update the plots
def Plot():
    size=len(NewData)
    Nb_sequence=size/8

    x=np.arange(Nb_sequence,dtype=float)
    y=np.arange(Nb_sequence,dtype=float)
    SoundDirection_X=np.arange(Nb_sequence,dtype=float)
    SoundDirection_Y=np.arange(Nb_sequence,dtype=float)
    TravelDirection_X=np.arange(Nb_sequence,dtype=float)
    TravelDirection_Y=np.arange(Nb_sequence,dtype=float)
    ObstacleDirection_X=np.arange(Nb_sequence,dtype=float) 
    ObstacleDirection_Y=np.arange(Nb_sequence,dtype=float)

    #loop to assign the different data
    i=0
    j=0
    while i < size  :
        x[j]=NewData[i]
        y[j]=NewData[i+1]
        SoundDirection_X[j]=NewData[i+2]
        SoundDirection_Y[j]=NewData[i+3]
        TravelDirection_X[j]=NewData[i+4]
        TravelDirection_Y[j]=NewData[i+5]
        ObstacleDirection_X[j]=NewData[i+6]
        ObstacleDirection_Y[j]=NewData[i+7]
        i=i+8
        j=j+1

    
    
    graph_position=plt.scatter(x,y,marker='x')
    k=0
    while k < Nb_sequence :
        text=plt.annotate(k,(x[k],y[k]),xytext=(x[k]+0.03,y[k]))
        k=k+1
        text.set_fontsize(20)
        
    origin=[x],[y]
    plt.quiver(*origin, SoundDirection_X, SoundDirection_Y, color=['m'],label="Sound Direction")
    plt.quiver(*origin, TravelDirection_X,TravelDirection_Y, color=['b'],label="Travel Direction")
    plt.quiver(*origin, ObstacleDirection_X,ObstacleDirection_Y, color=['g'],label="Obstacle Direction")



    
        
def Upload_Data(port):
    
    Data=readFloatSerial(port)
    if(len(Data)>0):
        
        mylist=[i[0] for i in Data]  
        print('mylist',mylist)
        
        NewData.extend(mylist)
        
        print('--','NewData',NewData,'--')

        reader_thd.tell_to_update_plot()
    


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
        self.contReceive = True 
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
            if(self.contReceive):
                Upload_Data(self.port)
            else:
                Plot()

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceive(self, val):  

        self.contReceive = True

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self, val):
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
fig.canvas.set_window_title('Position plot with indication of various directions')
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

#Config Button
colorAx            = 'lightgoldenrodyellow'
GetAx              = plt.axes([0.35, 0.025, 0.1, 0.04])
Get                = Button(GetAx, 'Get Values', color=colorAx, hovercolor='0.975')


#config plot 
graph_position = plt.subplot(111)

plt.title("Plot of the positions of the Robot and different directions")
plt.xlabel("x position [cm]")
plt.ylabel("y position [cm]")
plt.axis((-200,250,-200,250))


#End of reading thread and generation of the plot
Get.on_clicked(reader_thd.stop_reading)


plt.text(140,190,'Sound Direction',color='m')
plt.text(140,180,'Travel Direction',color='b')
plt.text(133,170,'Obstacle Direction',color='g')

plt.show()











        
