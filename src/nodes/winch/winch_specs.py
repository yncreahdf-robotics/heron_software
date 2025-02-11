#!/usr/bin/env python




MINHEIGHT = 695 #mm  
MAXHEIGHT = 1071 #mm

MAXSPEEDTICKS = 1700 #ticks
MAXTICKS = 14330 #ticks  Max height in ticks

TICKSPERMM = MAXTICKS/(MAXHEIGHT-MINHEIGHT) #ticks for 1 mm  38,1117
MAXSPEED_M_S = (MAXSPEEDTICKS/TICKSPERMM)*0.001 # m.s-1

ACCELTICKS = 2000 #ticks
DECELTICKS = 2000#ticks