#!/usr/bin/env python3
import numpy as np
import math
import rospy
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs

g = [0,0,-9.8]

#vTarget = [0,0,0]
#pTarget = [3,0,0]

#pSelf = [0,0,0]
projectileSpeed = 15

robot_state = [0,0,0,0,0,0]
targets_vision = [0.1,0.1,0.1,0.1,0.1] #xywhd

fov = [58,87]   #y,x  #https://www.intel.com/content/www/us/en/support/articles/000030385/emerging-technologies/intel-realsense-technology.html

eps = np.finfo(np.float64).eps
sqrt3 = math.sqrt(3)

cbrt = 1/3


def solveQuadratic(a, b, c):
    k = -b/(2*a)
    d = c/a - k*k
    #x^2 + d = 0
    #x^2 = -d
    if d > 0:  
        return []
    s = math.sqrt(-d)
    return [k - s, k + s]

def solveCubic(a, b, c, d):
    m = -b/(3*a)
    p = (c + m*(2*b + 3*a*m))/(3*a)
    q = -(d + m*(c + m*(b + a*m)))/(2*a)

    #t^3 + 3pt - 2q = 0

    M = p**3 + q*q

    if M < -eps:
        r = math.sqrt(-p)
        t = math.atan2(math.sqrt(-M), q)/3
        co = r*math.cos(t)
        si = r*math.sin(t)
        return [m - co - sqrt3*si, m - co + sqrt3*si, m + 2*co]
    elif M < eps:
        qr = (q)**cbrt
        if qr >= 0:
            return [m - qr, m - qr, m + 2*qr]
        return [m + 2*qr, m - qr, m - qr]
    else:
        r = math.sqrt(M)
        return [m + (q + r)**cbrt + (q - r)**cbrt]

def solveQuartic(a, b, c, d, e):
    k = -b/(4*a)
    p = (c + 3*k*(b + 2*a*k))/a
    q = (d + k*(2*c + k*(3*b + 4*a*k)))/a
    r = (e + k*(d + k*(c + k*(b + a*k))))/a

    #u^4 + pu^2 + qu + r = 0
    
    #When q = 0 then it's a biquadratic
    if abs(q) < eps:
        #u^4 + pu^2 + r = 0
        z = solveQuadratic(1, p, r)
        if z.__len__() == 0 or z[1] < 0:
            return []
        z0, z1 = math.sqrt(z[0]), math.sqrt(z[1])
        return [k - z1, k - z0, k + z0, k + z1]
    #Uses ferrari method
    _y = solveCubic(2, -p, -2*r, p*r - q*q/4)
    y = _y[2] if _y.__len__() == 3 else _y[0]

    m0 = math.sqrt(2*y - p)
    m1 = q/(2*m0)

    n0 = solveQuadratic(1, m0, y - m1)
    n1 = solveQuadratic(1, -m0, y + m1)
    n0l = n0.__len__()
    n1l = n1.__len__()

    sol = []

    if n1l == 2 and n0l == 2:
        sol.append(k + n0[0])
        sol.append(k + n0[1])
        sol.append(k + n1[0])
        sol.append(k + n1[1])
    elif n0l == 2:
        sol.append(k + n0[0])
        sol.append(k + n0[1])
    elif n1l == 2:
        sol.append(k + n1[0])       
        sol.append(k + n1[1])
    
    sol.sort()

    
    return sol



def calculateFinalPosition(time, position, velocity):
    return (position[0]+time*velocity[0],position[1]+time*velocity[1],position[2])

def dot(x,y):
    return x[0]*y[0]+x[1]*y[1]+x[2]*y[2]


#a = (1/4)*dot(g,g)
#b = dot(vTarget,g)
#c = dot(pTarget,g)+dot(vTarget,vTarget)-projectileSpeed**2           IF UNCOMMENT G MUST BE NEGATIVE
#d = 2*dot(pTarget,vTarget)
#e = dot(pTarget,pTarget)

def getBallistics(pSelf, pTarget, vTarget, g):

    p = [pTarget[0]-pSelf[0],pTarget[1]-pSelf[1],pTarget[2]-pSelf[2]]

    a = (1/4)*dot(g,g)
    b = -1*dot(vTarget,g)
    c = -dot(p,g)+dot(vTarget,vTarget)-projectileSpeed**2          
    d = 2*dot(p,vTarget)
    e = dot(p,p)

    time = solveQuartic(a,b,c,d,e)

    smallest = 99999
    for i in time:
        if i<smallest and i>0:
            smallest = i
    
    if smallest == 99999:
        print("No viable shot")
        return [0,0]

    time = smallest


    finalPos = calculateFinalPosition(time,pTarget,vTarget)

    yaw = math.degrees(math.atan2(finalPos[0]-pSelf[0],finalPos[1]-pSelf[1]))

    distance = math.sqrt((abs(finalPos[0]-pSelf[0]))**2+(abs(finalPos[1]-pSelf[1]))**2)

    pitch = 0

    ap = (g[2]*distance**2)/(2*projectileSpeed**2)
    bp = distance
    cp = ap-(pTarget[2]-pSelf[2])

    val = np.roots([ap,bp,cp])

    
    val = min(i for i in val)



    #print(distance)

    try:
        pitch = math.degrees(math.atan(val))
    except: 
        print("Failed!",distance)
        #print("Distance: {}, FinalPos: {}, pSelf: {}".format(distance,finalPos,pSelf))
    

    #print("time: {}, Our position: {}, Target position: {}, Target velocity: {}, yaw: {}, pitch: {}".format(time,pSelf,pTarget,vTarget,yaw,pitch))
    #print("pitch: ",pitch)

    
    return [yaw, pitch]


def chooseTarget(targets):

    smallest = targets[0]
    currDistance = 99999
    for i in targets:
        distance = np.sqrt((i[0]-(i[2])/2)**2 +
                              (i[1]-(i[3])/2)**2)
        if(distance<currDistance):
            currDistance = distance
            smallest = i   
        
    return smallest

def findGlobalAngle(target,fov,robotState):
    # imgAngleX = ((target[2]-target[0])/target[2])*fov[1]

    angleX = (fov[1]*target[0])-(fov[1]/2.0)    
    angleY = (fov[0]*target[1])-(fov[0]/2.0) + 90

    # imgAngleY = ((target[3]-target[1])/target[3])*fov[0]

    psi = robotState[4]  #yaw   #CHECK INDICES FROM ENC MAG POS
    phi = robotState[3]  #pitch
    # x = imgAngleX + psi
    # y = imgAngleY + phi

    return [angleX+psi,angleY+phi]  #degrees

def findTargetPos(angle, robotState, depth):  #assumes camera is at the x,y pos given in robotState


    angle[0] = np.deg2rad(angle[0])
    angle[1]= np.deg2rad(angle[1])

    x = np.cos(angle[0])*np.sin(angle[1])*depth
    y = np.sin(angle[0])*np.sin(angle[1])*depth
    z = np.cos(angle[1])

    return [x,y,z]  #eventually need to adjust for current robot pos

def vision_callback(data):
    global targets_vision
    targets_vision = data.data
    # print(data.data)
        

def robotstate_callback(data):
   global robot_state
   robot_state = data.data  #[X,Y,Thet,Pitch,Yaw,Feeder,Shooter] according to md file in firmware


#xywh


    
def rosSetup(pub_topic, r_sub_topic,v_sub_topic):
        rospy.init_node('ballistics')
        pub = rospy.Publisher(pub_topic, Float64MultiArray,queue_size=1)
        rospy.Subscriber(r_sub_topic,Float64MultiArray,robotstate_callback)
        rospy.Subscriber(v_sub_topic,Float64MultiArray,vision_callback)
        return pub

if __name__ == "__main__":
        pub = rosSetup('gimbal_control_input','enc_mag_pos','vision') #gimbal_control_input

        rate = rospy.Rate(10)
        msg = Float64MultiArray()

        while not rospy.is_shutdown():
           if(targets_vision[2]!=0):

            angle = findGlobalAngle(targets_vision, fov, robot_state)
            pos = findTargetPos(angle, robot_state, targets_vision[2])
            # print(pos)
            # print(targets_vision[2])

            b = getBallistics([0,0,0],pos,[0,0,0],g)

            # print("BALLISTICS: ",b)

            b[0] = b[0] - 90 #adjust axis

            print(b)
            
            # msg.data = np.array(b)
                
            # pub.publish(msg)     
            rate.sleep()
           else: print("0 DEPTH")



        




    


        


