'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
from time import sleep
from numpy.matlib import identity
import numpy as np
from threading import Thread

import sys
sys.path.append('../joint_control')

from keyframes import *

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        process = Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        process.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        process = Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        process.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.s = xmlrpc.client.ServerProxy('http://localhost:9999')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.s.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return self.s.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.s.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.s.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return np.asarray(self.s.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.s.set_transform(effector_name, transform.tolist())

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    #print(agent.s.system.listMethods())
    
    print(agent.get_angle("RKneePitch"))
    
    print(agent.set_angle("RKneePitch", 0.5))
    sleep(1)
    print(agent.get_angle("RKneePitch"))
    print(agent.get_posture())
    sleep(1)
    print(agent.post.execute_keyframes(leftBackToStand()))
    sleep(0.1)
    while True:
        try:
            print(agent.get_posture())
        except:
            print("Loading Keyframes")
            continue
        break
    
    print(agent.get_transform("HeadYaw"))
    T = identity(4)
    T[-1, 0] = -0.5
    print(agent.post.set_transform("Head",T))
    
    sleep(0.1)
    while True:
        try:
            print(agent.get_transform("HeadYaw"))
        except:
            print("Loading set_transform")
            continue
        break
        
    #print(agent.get_posture())
    print("check")
    

