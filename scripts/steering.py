#!/usr/bin/env python

import rospy
import std_msgs
import motor
import time
import threading
import sys

lock = threading.Lock()
#desired_ticks = 0

def init_motor(steeringMotor, p, i, d):     #add other parameters later
    steeringMotor.connect()

    steeringMotor.setCurrentLimit(4000)
    steeringMotor.windUpGaurd(4000)
    steeringMotor.setEncoderTicks(735*4)
    steeringMotor.setControlMode(2)
    steeringMotor.setWheelDiameter(.4)

    steeringMotor.setPid(0.15, 0.13, 0.0) #0.2, 0.2, 0
    steeringMotor.setAcceleration(6)
    steeringMotor.setDeceleration(6)


def ticks_to_angle(ticks):
    return (52.5/10000)*(ticks)

                    #assume linear interpolation, ticks range from -10000->+10000, angles range from -52.5->+52.5 degrees
                    #assume steering is calibrated such that tick 0 corresponds to angle 0

def angle_to_ticks(angle):          
    return (10000/52.5)*angle

def set_tick():
    global desired_ticks
    try:
        while True:
            lock.acquire()

            if(desired_ticks > 10000 or desired_ticks < -10000):
                print("Invalid input, angle range is from +50 to -50 degrees")
                continue
            time.sleep(0.01)
            current_tick = steeringMotor.currentEncoderTicks()
            
            if(desired_ticks > current_tick+25):
                time.sleep(0.01)            #add delay for pcb communication time
                steeringMotor.requestTickVelocity(30)
                time.sleep(0.01)
                print("desired ticks {0}\t {1} \tpositive".format(desired_ticks,steeringMotor.currentEncoderTicks()))

            elif(desired_ticks < current_tick-25):
                time.sleep(0.01)
                steeringMotor.requestTickVelocity(-30)
                time.sleep(0.01)
                print("desired ticks {0}\t {1} \tnegative".format(desired_ticks,steeringMotor.currentEncoderTicks()))
            else:
                time.sleep(0.01)
                steeringMotor.requestTickVelocity(0)
                time.sleep(0.01)
                desired_ticks = steeringMotor.currentEncoderTicks()
            
            lock.release()
            time.sleep(0.01)

    finally:
        time.sleep(0.01)
        steeringMotor.requestTickVelocity(0)
        print("Exiting")


def callback(data):
    pub = rospy.Publisher("current_angle", std_msgs.msg.Float32, queue_size=10)
    rate = rospy.Rate(10)
    init_ticks = steeringMotor.currentEncoderTicks()
    init_angle = ticks_to_angle(init_ticks)
    print("init angle {0}".format(init_angle))
    pub.publish(init_angle)

    lock.acquire()
    global desired_ticks
    desired_ticks = angle_to_ticks(data.data)
    print("desired ticks {0}".format(desired_ticks)) 
    lock.release()

def listener():
    rospy.Subscriber("desired_angle", std_msgs.msg.Float32, callback)
    rospy.spin()

def main():
    
    rospy.init_node('steering_node', anonymous=True)

    address = rospy.get_param("~address")
    port = rospy.get_param("~port")
    p = rospy.get_param("~p")
    i = rospy.get_param("~i")
    d = rospy.get_param("~d")
    
    global steeringMotor
    steeringMotor = motor.nextEng(address, port)
    
    init_motor(steeringMotor, p, i, d)
    init_position = steeringMotor.currentEncoderTicks()
    print("Init position {0}".format(init_position))

    global desired_ticks
    desired_ticks = init_position
    print("main {0}".format(desired_ticks))
    set_tick_thread = threading.Thread(target=set_tick, args=())
    set_tick_thread.daemon = True
    set_tick_thread.start()

    listener()

if __name__ == "__main__":
    
    main()