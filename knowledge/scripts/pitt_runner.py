#! /usr/bin/env python

import roslaunch_api_wrapper as raw
import signal
import sys
import time
import roslib
import rospy
import tf
import numpy
import rospy
from std_msgs.msg import String
import roslaunch
import subprocess# as child

packageName = 'pitt_object_table_segmentation'
LauncherName = 'table_segmentation.launch'
newCondition=False
runnerFlag=False
# child

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def signal_handler(signal, frame):
    print('Bye!')
    sys.exit(0)
    
def callback(data):
    rospy.loginfo("callbackLauncher: I heard %s",data.data)
    if(data.data=='RUN_PITT'):
        print(bcolors.BOLD + bcolors.WARNING +"********** Run the pitt! **********"+ bcolors.ENDC)
        global child;    
        child = subprocess.Popen(["roslaunch","pitt_object_table_segmentation","table_segmentation.launch"])
        print("parent process")
        print(child.poll())
        rospy.loginfo('The PID of child: %d', child.pid)
        print ("The PID of child:", child.pid)
        
    elif(data.data=='KILL_PITT'):
        print(bcolors.BOLD + bcolors.WARNING +"********** Kill the pitt! **********"+ bcolors.ENDC)
        child.send_signal(signal.SIGINT)
    
    else:
        print(bcolors.BOLD + bcolors.WARNING +"********** No Command **********"+ bcolors.ENDC)
        
    print('pitt_runner process is alive!')

# def callbackKiller(data):
#     rospy.loginfo("callbackKiller: I heard %s",data.data)
#     if(data.data=='KILL_PITT'):
#         print("Kill the pitt!")
#         child.send_signal(signal.SIGINT)
#     print('pitt_runner process is alive!')

def main():    
    rospy.init_node('pitt_Launcher')

    Sub = rospy.Subscriber('PerceptionRunner', String, callback)
    print('pitt runner is alive!')


    rospy.spin()
    signal.signal(signal.SIGINT, signal_handler)

    print('Press Ctrl+C')
    signal.pause()

if __name__ == '__main__':
    main()
