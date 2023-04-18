#!/usr/bin/python3
from tkinter import *
import socket
import struct
import sys
import time

HOST = 'localhost'    # The remote host
PORT = 9999           # The same port as used by the server

argvs = sys.argv 
argc = len(argvs)
if (argc != 3):
    print('Num of arguments are not two. Use default [HOST = %s, PORT = %d]' % (HOST, PORT)) 
else:
    HOST = argvs[1]
    PORT = int(argvs[2])
    print('HOST = %s, PORT = %d' % (HOST, PORT))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

root = Tk()

LocalizeMUSIC_num_source = IntVar()
LocalizeMUSIC_num_source.set(2)
LocalizeMUSIC_min_deg = IntVar()
LocalizeMUSIC_min_deg.set(-180)
LocalizeMUSIC_max_deg = IntVar()
LocalizeMUSIC_max_deg.set(180)
LocalizeMUSIC_lower_bound_frequency = IntVar()
#LocalizeMUSIC_lower_bound_frequency.set(500)
LocalizeMUSIC_lower_bound_frequency.set(3000)
LocalizeMUSIC_upper_bound_frequency = IntVar()
#LocalizeMUSIC_upper_bound_frequency.set(2800)
LocalizeMUSIC_upper_bound_frequency.set(6000)
SourceTracker_thresh = DoubleVar()
#SourceTracker_thresh.set(30.0)
SourceTracker_thresh.set(25.0)
SourceTracker_pause_length = DoubleVar()
#SourceTracker_pause_length.set(800.0)
SourceTracker_pause_length.set(1200.0)
SourceTracker_min_src_interval = DoubleVar()
SourceTracker_min_src_interval.set(20.0)
SourceTracker_min_tfindex_interval = DoubleVar()
SourceTracker_min_tfindex_interval.set(6.0)
SourceTracker_compare_mode = IntVar()
SourceTracker_compare_mode.set(0)
HRLE_lx = DoubleVar()
#HRLE_lx.set(0.85)
HRLE_lx.set(0.2)
HRLE_time_constant = IntVar()
HRLE_time_constant.set(16000)

save_buffer = StringVar()
save_buffer.set("")

load_buffer = StringVar()
load_buffer.set("")

def change_param( n ):
    global sock
    tmp = [
        float(LocalizeMUSIC_num_source.get()), 
        float(LocalizeMUSIC_min_deg.get()), 
        float(LocalizeMUSIC_max_deg.get()), 
        float(LocalizeMUSIC_lower_bound_frequency.get()), 
        float(LocalizeMUSIC_upper_bound_frequency.get()), 
        float(SourceTracker_thresh.get()), 
        float(SourceTracker_pause_length.get()), 
        float(SourceTracker_min_src_interval.get()), 
        float(SourceTracker_min_tfindex_interval.get()), 
        float(SourceTracker_compare_mode.get()), 
        float(HRLE_lx.get()), 
        float(HRLE_time_constant.get()) 
        ]
    msg = struct.pack("f"*len(tmp), *tmp) 
    sock.send(msg)

def file_save():
    if save_buffer.get():
        f = open(save_buffer.get(), 'w')
        f.write(str(int(LocalizeMUSIC_num_source.get()))+'\n') 
        f.write(str(int(LocalizeMUSIC_min_deg.get()))+'\n')
        f.write(str(int(LocalizeMUSIC_max_deg.get()))+'\n')
        f.write(str(int(LocalizeMUSIC_lower_bound_frequency.get()))+'\n') 
        f.write(str(int(LocalizeMUSIC_upper_bound_frequency.get()))+'\n')
        f.write(str(float(SourceTracker_thresh.get()))+'\n')
        f.write(str(float(SourceTracker_pause_length.get()))+'\n') 
        f.write(str(float(SourceTracker_min_src_interval.get()))+'\n')
        f.write(str(float(SourceTracker_min_tfindex_interval.get()))+'\n')
        f.write(str(int(SourceTracker_compare_mode.get()))+'\n')
        f.write(str(float(HRLE_lx.get()))+'\n')
        f.write(str(int(HRLE_time_constant.get()))+'\n') 
        f.close()
        print('%s saved' % (save_buffer.get()))

def file_load():
    if load_buffer.get():
        f = open(load_buffer.get(), 'r')
        LocalizeMUSIC_num_source.set(int(f.readline().rstrip()))
        LocalizeMUSIC_min_deg.set(int(f.readline().rstrip()))
        LocalizeMUSIC_max_deg.set(int(f.readline().rstrip()))
        LocalizeMUSIC_lower_bound_frequency.set(int(f.readline().rstrip()))
        LocalizeMUSIC_upper_bound_frequency.set(int(f.readline().rstrip()))
        SourceTracker_thresh.set(float(f.readline().rstrip()))
        SourceTracker_pause_length.set(float(f.readline().rstrip()))
        SourceTracker_min_src_interval.set(float(f.readline().rstrip()))
        SourceTracker_min_tfindex_interval.set(float(f.readline().rstrip()))
        SourceTracker_compare_mode.set(int(f.readline().rstrip()))
        HRLE_lx.set(float(f.readline().rstrip()))
        HRLE_time_constant.set(int(f.readline().rstrip()))
        f.close()
        print('%s loaded' % (load_buffer.get()))

# --------------------------------------------------------------------

win=Frame(root)

win.pack(fill=BOTH,expand=1)

c = Canvas(win,width=450, height=450,
           scrollregion=(0,0,450,1100),
           confine='true')
c.grid(column=0, row=0, sticky=N+E+S+W)
win.grid_columnconfigure(0,weight=1)
win.grid_rowconfigure(0,weight=1)

vsb=Scrollbar(win,orient="vertical", command=c.yview, width=10)
c["yscrollcommand"]=vsb.set
vsb.grid(column=1, row=0, sticky=N+S)

# hsb=Scrollbar(win,orient="horizontal", command=c.xview, width=10)
# c["xscrollcommand"]=hsb.set
# hsb.grid(column=0, row=1, sticky=E+W)

win2=Frame(root)

# --------------------------------------------------------------------

loc_f = Frame(win2)

loc_f_l = Frame(loc_f)

loc_buff = StringVar()
loc_buff.set("LocalizeMUSIC")
loc_label = Label(loc_f_l, textvariable = loc_buff)
loc_label.pack()

loc_f_l.pack(side = 'left')

loc_f_r = Frame(loc_f)

s_LocalizeMUSIC_num_source = Scale(loc_f_r, label = 'NUM_SOURCE', orient = 'h',
                                  from_ = 1, to = 6, variable = LocalizeMUSIC_num_source,
                                  command = change_param, length = 300)
s_LocalizeMUSIC_num_source.pack(fill = 'both')

s_LocalizeMUSIC_min_deg = Scale(loc_f_r, label = 'MIN_DEG', orient = 'h',
                                from_ = -180, to = 180, variable = LocalizeMUSIC_min_deg,
                                command = change_param, length = 300)
s_LocalizeMUSIC_min_deg.pack(fill = 'both')

s_LocalizeMUSIC_max_deg = Scale(loc_f_r, label = 'MAX_DEG', orient = 'h',
                                from_ = -180, to = 180, variable = LocalizeMUSIC_max_deg,
                                command = change_param, length = 300)
s_LocalizeMUSIC_max_deg.pack(fill = 'both')

s_LocalizeMUSIC_lower_bound_frequency = Scale(loc_f_r, label = 'LOWER_BOUND_FREQUENCY', orient = 'h',
                                              from_ = 0, to = 8000, variable = LocalizeMUSIC_lower_bound_frequency,
                                              command = change_param, length = 300)
s_LocalizeMUSIC_lower_bound_frequency.pack(fill = 'both')

s_LocalizeMUSIC_upper_bound_frequency = Scale(loc_f_r, label = 'UPPER_BOUND_FREQUENCY', orient = 'h',
                                              from_ = 0, to = 8000, variable = LocalizeMUSIC_upper_bound_frequency,
                                              command = change_param, length = 300)
s_LocalizeMUSIC_upper_bound_frequency.pack(fill = 'both')

loc_f_r.pack(side = 'left')

loc_f.pack(fill = BOTH)

# --------------------------------------------------------------------

line1 = Canvas(win2, width = 400, height = 20)
line1.create_line(10, 10, 390, 10)
line1.pack()

# --------------------------------------------------------------------

src_f = Frame(win2)

src_f_l = Frame(src_f)

src_buff = StringVar()
src_buff.set("SourceTracker")
src_label = Label(src_f_l, textvariable = src_buff)
src_label.pack()

src_f_l.pack(side = 'left')

src_f_r = Frame(src_f)

s_SourceTracker_thresh = Scale(src_f_r, label = 'THRESH', orient = 'h',
                               from_ = 0, to = 50.0, variable = SourceTracker_thresh,
                               command = change_param, resolution = 0.01, length = 300)
s_SourceTracker_thresh.pack(fill = 'both')

s_SourceTracker_pause_length = Scale(src_f_r, label = 'PAUSE_LENGTH', orient = 'h',
                               from_ = 0, to = 5000.0, variable = SourceTracker_pause_length,
                               command = change_param, length = 300)
s_SourceTracker_pause_length.pack(fill = 'both')

s_SourceTracker_min_src_interval = Scale(src_f_r, label = 'MIN_SRC_INTERVAL', orient = 'h',
                                         from_ = 0, to = 180.0, variable = SourceTracker_min_src_interval,
                                         command = change_param, length = 300)
s_SourceTracker_min_src_interval.pack(fill = 'both')

s_SourceTracker_min_tfindex_interval = Scale(src_f_r, label = 'MIN_TFINDEX_INTERVAL', orient = 'h',
                                             from_ = 0, to = 36.0, variable = SourceTracker_min_tfindex_interval,
                                             command = change_param, length = 300)
s_SourceTracker_min_tfindex_interval.pack(fill = 'both')

s_SourceTracker_compare_mode = Scale(src_f_r, label = 'COMPARE_MODE', orient = 'h',
                                     from_ = 0, to = 1, variable = SourceTracker_compare_mode,
                                     command = change_param, length = 300)
s_SourceTracker_compare_mode.pack(fill = 'both')

src_f_r.pack(side = 'left')

src_f.pack(fill = BOTH)

# --------------------------------------------------------------------

line2 = Canvas(win2, width = 400, height = 20)
line2.create_line(10, 10, 390, 10)
line2.pack()

# --------------------------------------------------------------------

hrl_f = Frame(win2)

hrl_f_l = Frame(hrl_f)

hrl_buff = StringVar()
hrl_buff.set("HRLE             ")
hrl_label = Label(hrl_f_l, textvariable = hrl_buff)
hrl_label.pack()

hrl_f_l.pack(side = 'left')

hrl_f_r = Frame(hrl_f)

s_HRLE_lx = Scale(hrl_f_r, label = 'LX', orient = 'h',
                  from_ = 0, to = 1, variable = HRLE_lx,
                  command = change_param, resolution = 0.01, length = 300)
s_HRLE_lx.pack(fill = 'both')

s_HRLE_time_constant = Scale(hrl_f_r, label = 'TIME_CONSTANT', orient = 'h',
                             from_ = 160, to = 960000, variable = HRLE_time_constant,
                             command = change_param, length = 300)
s_HRLE_time_constant.pack(fill = 'both')

hrl_f_r.pack(side = 'left')

hrl_f.pack(fill = BOTH)

# --------------------------------------------------------------------

line3 = Canvas(win2, width = 400, height = 20)
line3.create_line(10, 10, 390, 10)
line3.pack()

# --------------------------------------------------------------------

f0 = Frame(win2)

s_buff = StringVar()
s_buff.set("File Save")
s_label = Label(f0, textvariable = s_buff)
s_label.pack()

e_save = Entry(f0, textvariable = save_buffer, width = 40)
e_save.pack(side = 'left')
# e_save.bind('<Return>', disp_form)
button_save = Button(f0, text = "Save", command = file_save)
button_save.pack(side = 'left')

f0.pack(fill = BOTH)

f1 = Frame(win2)

l_buff = StringVar()
l_buff.set("File Load")
l_label = Label(f1, textvariable = l_buff)
l_label.pack()

e_load = Entry(f1, textvariable = load_buffer, width = 40)
e_load.pack(side = 'left')
# e_load.bind('<Return>', disp_form)
button_load = Button(f1, text = "Load", command = file_load)
button_load.pack(side = 'left')

f1.pack(fill = BOTH)

# --------------------------------------------------------------------

c.create_window(10,10, window=win2, anchor="nw")

root.mainloop();

sock.close()
