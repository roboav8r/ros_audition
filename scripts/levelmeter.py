#!/usr/bin/python3
# -*- coding: utf-8 -*-

import curses
import numpy

class CursesWindow(object):
    def __init__(self):
        self.stdscr = curses.initscr()
        self.use_color = curses.has_colors()
        if self.use_color:
            curses.start_color()
            curses.use_default_colors()
            for i in range(0, curses.COLORS):
                curses.init_pair(i + 1, i, -1)
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(True)
        (self.max_y, self.max_x) = self.getmaxyx()

    def __del__(self):
        self.stdscr.keypad(False)
        curses.nocbreak()
        curses.echo()
        curses.endwin()

    def clear(self):
        self.stdscr.clear()

    def refresh(self):
        self.stdscr.refresh()

    def getkey(self):
        return self.stdscr.getkey()

    def getyx(self):
        return self.stdscr.getyx()

    def getmaxyx(self):
        return self.stdscr.getmaxyx()

    def getyx(self):
        return self.stdscr.getyx()

    def getch(self):
        return self.stdscr.getch()

    def delch(self):
        self.stdscr.delch()

    def addch(self, char, y=-1, x=-1, attr=curses.A_NORMAL):
        (cy, cx) = self.getyx()
        if y < 0:
            y = cy
        if x < 0:
            x = cx
        self.stdscr.addch(y, x, char, attr)

    def addnstr(self, msg, n, y=-1, x=-1, attr=curses.A_NORMAL):
        (cy, cx) = self.getyx()
        if y < 0:
            y = cy
        if x < 0:
            x = cx
        self.stdscr.addstr(y, x, msg, n, attr)

    def addstr(self, msg, y=-1, x=-1, attr=curses.A_NORMAL):
        (cy, cx) = self.getyx()
        if y < 0:
            y = cy
        if x < 0:
            x = cx
        self.stdscr.addstr(y, x, msg, attr)

    def print_err(self, msg):
        self.clear()
        self.stdscr.addstr(msg, curses.color_pair(197))

    def move(self, y=-1, x=-1):
        (cy, cx) = self.getyx()
        if y < 0:
            y = cy
        if x < 0:
            x = cx
        self.stdscr.move(y, x)

    def print_rect(self, sy, sx, ey, ex):
        if (ey > sy) and (ex > sx) and (sy >= 0) and (sx >= 0) and (self.max_y > ey) and (self.max_x > ex):
            for y in range(sy, ey+1):
                line = []
                for x in range(sx, ex+1):
                    if y == sy and x == sx:
                        self.addch(curses.ACS_ULCORNER, y, x)
                    elif y == sy and x == ex:
                        self.addch(curses.ACS_URCORNER, y, x)
                    elif y == ey and x == sx:
                        self.addch(curses.ACS_LLCORNER, y, x)
                    elif y == ey and x == ex:
                        self.addch(curses.ACS_LRCORNER, y, x)
                    elif y == sy or y == ey:
                        self.addch(curses.ACS_HLINE, y, x)
                    elif x == sx or x == ex:
                        self.addch(curses.ACS_VLINE, y, x)
        else:
            self.print_err("print_rect(): range error")

    def print_scale_number(self, y, x, value, bar_length=1):
        if (y >= 0) and (x-int(len(str(value))/2) >= 0) and (self.max_y > y+bar_length+1) and (self.max_x > x+int(len(str(value))/2)):
            for i in range(bar_length):
                self.addstr('|', y+i, x, curses.A_BOLD)
            self.addstr(str(value), y+bar_length, x-int(len(str(value))/2), curses.A_BOLD)

    def print_scale(self, sy, sx, ey, ex, interval, min_val, max_val, precision=2):
        if (ey > sy) and (ex > sx) and (sy >= 0) and (sx >= 0) and (self.max_y > ey) and (self.max_x > ex) and (max_val > min_val):
            scales = [str(round(x, precision)) for x in numpy.arange(min_val, max_val+1, ((max_val-min_val)/(ex-sx))*interval, dtype='float32')]
            value_len = max([len(x) for x in scales])
            #if (value_len/2 + 1) > interval:
            #    self.print_err("interval too small")
            scale_len = len(scales)
            max_scale_size = ((value_len+1)*scale_len)-1
            scale_lines = int(numpy.ceil(max_scale_size/float(self.max_x)))
            y_list = [y for y in range(1, scale_lines+1, 1)]
            i = 0
            for x in range(sx, ex, interval):
                if i < scale_len:
                    self.print_scale_number(sy, x, scales[i], y_list[i%len(y_list)])
                i = i+1
        else:
            self.print_err("print_scale(): range error")

    def print_special_bar(self, sy, sx, size, min_val, max_val, val, color=197):
        if (self.max_y > sy+2) and (self.max_x > sx+size+1) and (size >= 0) and (min_val <= val <= max_val):
            bar = int(round(((float(val)-min_val)/(max_val-min_val))*float(size+1), 0))
            if not (0 <= bar <= size+1):
                self.print_err("print_special_bar(): bar location error")
            if bar > 0:
                self.addstr('|', sy+0, sx+1+bar, curses.A_BOLD | curses.color_pair(color))
                self.addstr('|', sy+1, sx+1+bar, curses.A_BOLD | curses.color_pair(color))
                self.addstr('V', sy+2, sx+1+bar, curses.A_BOLD | curses.color_pair(color))
        else:
            self.print_err("print_special_bar(): range error")

    def print_meter(self, sy, sx, size, interval, min_val, max_val, val):
        '''
        Level meter axis : (sx,sy) - (sx+size+1,sy+2)
        Scale axis       : (sx-[value_len/2+1],sy+3) - (sx+size+1+[value_len/2+1],sy+3+[scale_lines])) ## [] from print_scale()
        '''
        if (self.max_y > sy+2) and (self.max_x > sx+size+1) and (size >= 0) and (min_val <= val <= max_val):
            bar = int(round(((float(val)-min_val)/(max_val-min_val))*float(size+1), 0))
            if not (0 <= bar <= size+1):
                self.print_err("print_meter(): bar location error")
            self.print_rect(sy, sx, sy+2, sx+size+1)
            self.addstr('|'*bar, sy+1, sx+1, curses.A_BOLD)
            self.addstr(' '*(size-bar))
            self.print_scale(sy+3, sx, sy+4, sx+size+1, interval, min_val, max_val)
        else:
            self.print_err("print_meter(): range error")

class SpetrumPowerMeter(object):
    def __init__(self):
        self.cuiwindow = CursesWindow()
        self.cuiwindow.clear()
        self.meter_pos = (3, 3)         # (y, x)
        self.meter_bar = (74, 3)        # (size, span)
        self.meter_scale = (20.0, 35.0) # (min, max)
        #self.colors = { "red": 197, "green": 83, "blue": 22 }
        self.colors = { "red": 208, "green": 83, "blue": 52 }
        self.cur_val = 0.0
        self.cur_min = 100.0
        self.cur_max = 0.0
        self.hysteresis = 1.0
        self.list_size = 72
        self.min_list = []
        self.max_list = []

    def __del__(self):
        #self.cuiwindow.getkey()
        del self.cuiwindow

    def set_pos(self, y=3, x=3, size=74, span=3, min_val=20.0, max_val=50.0):
        self.meter_pos = (y, x)
        self.meter_bar = (size, span)
        self.meter_scale = (min_val, max_val)

    def set_params(self, history_size=100, hysteresis=1.0):
        self.list_size = history_size
        self.hysteresis = hysteresis

    def clear_history(self):
        self.min_list = []
        self.max_list = []

    def update(self, val):
        self.cur_val = val
        if not self.min_list or numpy.mean(self.min_list)+self.hysteresis > val:
            self.min_list.append(val)
            if len(self.min_list) > self.list_size:
                self.min_list = self.min_list[1:]
            self.cur_min = numpy.mean(self.min_list)
        if not self.max_list or numpy.mean(self.max_list)-self.hysteresis < val:
            self.max_list.append(val)
            if len(self.max_list) > self.list_size:
                self.max_list = self.max_list[1:]
            self.cur_max = numpy.mean(self.max_list)

    def draw(self):
        (pos_y, pos_x) = self.meter_pos
        (scale_min, scale_max) = self.meter_scale
        (bar_size, bar_span) = self.meter_bar
        (cur_val, cur_min, cur_max, cur_ave) = (self.cur_val, self.cur_min, self.cur_max, (self.cur_min+self.cur_max)/2)
        self.cuiwindow.print_meter(pos_y+4, pos_x, bar_size, bar_span, scale_min, scale_max, cur_val)
        self.cuiwindow.print_special_bar(pos_y+4, pos_x, bar_size, scale_min, scale_max, cur_min, self.colors["red"])
        self.cuiwindow.print_special_bar(pos_y+4, pos_x, bar_size, scale_min, scale_max, cur_max, self.colors["blue"])
        self.cuiwindow.print_special_bar(pos_y+4, pos_x, bar_size, scale_min, scale_max, cur_ave, self.colors["green"])
        self.cuiwindow.addstr("Environments noise power (silence period average) : {}".format(cur_min), pos_y, pos_x, curses.A_BOLD | curses.color_pair(self.colors["red"]))
        self.cuiwindow.addstr("Speaking power (talking period average)           : {}".format(cur_max), pos_y+1, pos_x, curses.A_BOLD | curses.color_pair(self.colors["blue"]))
        self.cuiwindow.addstr("SourceTracker recommend threshold                 : {}".format(cur_ave), pos_y+2, pos_x, curses.A_BOLD | curses.color_pair(self.colors["green"]))
        self.cuiwindow.refresh()

if __name__ == '__main__':
    import time
    # create test data
    data = []
    data.extend([22.0+i*5.0 for i in numpy.random.rand(50)])
    data.extend([25.5+i*15.0 for i in numpy.random.rand(300)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(150)])
    data.extend([26.5+i*14.0 for i in numpy.random.rand(500)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(150)])
    data.extend([24.5+i*16.0 for i in numpy.random.rand(400)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(50)])
    data.extend([25.5+i*15.0 for i in numpy.random.rand(300)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(100)])
    # test code
    count = 0
    meter = SpetrumPowerMeter()
    meter.set_pos(3, 3, 74, 3, 20.0, 50.0)
    try:
        for i in data:
            meter.update(i)
            if count % 1 == 0:
                meter.draw()
            #time.sleep(0.01)
            count += 1
        del meter
    except KeyboardInterrupt:
        del meter

"""
if __name__ == '__main__':
    import time
    # create test data
    data = []
    data.extend([22.0+i*5.0 for i in numpy.random.rand(50)])
    data.extend([25.5+i*15.0 for i in numpy.random.rand(300)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(150)])
    data.extend([26.5+i*14.0 for i in numpy.random.rand(500)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(150)])
    data.extend([24.5+i*16.0 for i in numpy.random.rand(400)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(50)])
    data.extend([25.5+i*15.0 for i in numpy.random.rand(300)])
    data.extend([22.0+i*5.0 for i in numpy.random.rand(100)])
    # test code
    cuiwindow = CursesWindow()
    cuiwindow.clear()
    meter_pos_y = 7
    meter_pos_x = 3
    meter_bar_size = 74
    meter_bar_span = 2
    scale_min = 20.0
    scale_max = 50.0
    color_red = 197
    color_blue = 22
    color_green = 83
    cur_min = 100.0
    cur_max = 0.0
    hysteresis = 1.0
    list_size = 100
    min_list = []
    max_list = []
    count = 0
    for i in data:
        if not min_list or numpy.mean(min_list)+hysteresis > i:
            min_list.append(i)
            if len(min_list) > list_size:
                min_list = min_list[1:]
        if not max_list or numpy.mean(max_list)-hysteresis < i:
            max_list.append(i)
            if len(max_list) > list_size:
                max_list = max_list[1:]
        cuiwindow.print_meter(meter_pos_y, meter_pos_x, meter_bar_size, meter_bar_span, scale_min, scale_max, i)
        cuiwindow.print_special_bar(meter_pos_y, meter_pos_x, meter_bar_size, scale_min, scale_max, numpy.mean(min_list), color_red)
        cuiwindow.print_special_bar(meter_pos_y, meter_pos_x, meter_bar_size, scale_min, scale_max, numpy.mean(max_list), color_blue)
        cuiwindow.print_special_bar(meter_pos_y, meter_pos_x, meter_bar_size, scale_min, scale_max, (numpy.mean(min_list)+numpy.mean(max_list))/2, color_green)
        cuiwindow.addstr("Measured silence period ave. power : {}".format(numpy.mean(min_list)), 3, 3, curses.A_BOLD | curses.color_pair(color_red))
        cuiwindow.addstr("Measured sounding ave. power       : {}".format(numpy.mean(max_list)), 4, 3, curses.A_BOLD | curses.color_pair(color_blue))
        cuiwindow.addstr("SourceTracker recommend threshold  : {}".format((numpy.mean(min_list)+numpy.mean(max_list))/2), 5, 3, curses.A_BOLD | curses.color_pair(color_green))
        cuiwindow.refresh()
        #time.sleep(0.01)
        count += 1
    #cuiwindow.getkey()
    del cuiwindow
"""

