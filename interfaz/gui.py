import Tkinter as tk
import subprocess
import time
import os
import signal

global p

def add():
    global p    
    p = subprocess.Popen('./launchScriptAssembled',preexec_fn=os.setsid)
    b2["state"] = tk.NORMAL
    b3["state"] = tk.NORMAL
    b4["state"] = tk.NORMAL
    b5["state"] = tk.NORMAL



def stop():
    global p
    os.killpg(os.getpgid(p.pid),signal.SIGKILL)
    b2["state"] = tk.DISABLED
    b3["state"] = tk.DISABLED
    b4["state"] = tk.DISABLED
    b5["state"] = tk.DISABLED
    b6["state"] = tk.DISABLED
    b7["state"] = tk.DISABLED

def cloud():
    subprocess.Popen('./launchColorCloud',preexec_fn=os.setsid)
    b6["state"] = tk.NORMAL
    b7["state"] = tk.NORMAL

def singleCloud():
    subprocess.Popen('./launchSingleCloud',preexec_fn=os.setsid)

def bothCloud():
    subprocess.Popen('./launchBothCloud',preexec_fn=os.setsid)
    b6["state"] = tk.NORMAL
    b7["state"] = tk.NORMAL

def getObject():
#    command = ["rosrun but_calibration_camera_velodyne getObject _param:="+str(txt.get())] 
#    print(command)
    p = subprocess.Popen('./getObject '+str(txt.get()), shell=True, preexec_fn=os.setsid)

def extractObjects():
    subprocess.Popen('./extractObjects',preexec_fn=os.setsid)

win=tk.Tk()
win.title('GUI')

################# CANVAS ###################################################################

C = tk.Canvas(win, bg="white", height=360, width=410)
filename = tk.PhotoImage(file = "./imagen_gui2.png")
background_label = tk.Label(win, image=filename)
background_label.place(x=0, y=0, relwidth=1, relheight=1)

################# LABELS ###################################################################
        
lbl1=tk.Label(win, text='3D-2D SYSTEM',font=('Calibri',26,'bold'),bg='navy',fg='white',width=16)
lbl2=tk.Label(win, text='Assemble',font=('Calibri',15,'bold'),bg='LightCyan2',fg='black',width=15)
lbl3=tk.Label(win, text=' Actions ',font=('Calibri',15,'bold'),bg='LightCyan2',fg='black',width=15)


lbl1.place(x=30, y=18)
lbl2.place(x=30, y=160)
lbl3.place(x=200, y=160)

################# BUTTONS ##################################################################

b1=tk.Button(win, text='Start',command=add,fg='white',bg='green')
b1.place(x=60, y=100)

b2=tk.Button(win, text='Stop ',command=stop,fg='white',bg='red')
b2.place(x=240, y=100)

b3=tk.Button(win, text='Pointcloud',command=singleCloud,fg='white',bg='gray27',width=13)
b3.place(x=40, y=210)

b4=tk.Button(win, text='Colored pointcloud',command=cloud,fg='white',bg='gray27',width=13)
b4.place(x=40, y=260)

b5=tk.Button(win, text='Both',command=bothCloud,fg='white',bg='gray27',width=13)
b5.place(x=40, y=310)

b6=tk.Button(win, text='Extract objects',command=extractObjects,fg='white',bg='gray27',width=13)
b6.place(x=210, y=210)

b7=tk.Button(win, text='Get object',command=getObject,fg='white',bg='gray27',width=13)
b7.place(x=210, y=260)

txt = tk.Entry(win,width=15)
 
txt.place(x=212, y=315)



b2["state"] = tk.DISABLED
b3["state"] = tk.DISABLED
b4["state"] = tk.DISABLED
b5["state"] = tk.DISABLED
b6["state"] = tk.DISABLED
b7["state"] = tk.DISABLED


############################################################################################


C.pack()
win.mainloop()
