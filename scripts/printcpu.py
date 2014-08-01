#!/usr/bin/env python
import rospy
from rqt_graphprofiler.msg import *
import sys
import curses

def callback(data):
    global lastbufferlen
    procs = list()
    for node in data.nodes:
        procs.append(tuple([node.name,node.cpu_load_mean]))
    procs.sort(lambda x,y: -1 if x[1] > y[1] else 1)
    strbuffer = ""
    strbuffer+= "%-30sCPU\n"%"Name"
    strbuffer+= "-"*33+"\n"
    for x in procs:
        strbuffer+= "%-30s%d\n"%(x[0][0:29],x[1])
    print strbuffer


if __name__ == "__main__":
    rospy.init_node("printcpu",anonymous=True)
    s = rospy.Subscriber('/graphprofile',HostProfile,callback)
    rospy.spin()
