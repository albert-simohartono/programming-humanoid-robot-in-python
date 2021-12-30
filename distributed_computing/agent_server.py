'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

#import RPC module
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import logging
import threading
from time import sleep
import numpy as np

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        
        logging.basicConfig(level=logging.DEBUG)
        server = SimpleXMLRPCServer(('localhost', 9999), logRequests=True)
        print("Listening on localhost:9999")
        server.register_instance(self)
        server.register_introspection_functions()
        server.register_multicall_functions()
        #server.serve_forever()
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        print("Server thread started")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE 
        angle = self.perception.joint[joint_name]
        return joint_name + " angle = " + str("%.2f" % angle)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name]=angle
        return ('set ' + joint_name + " angle to " + str(angle))

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return "Posture = " + self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes=keyframes
                
        #blocking, max_time = flatten 2d list of time(keyframes[1] = time)
        max_time = (max(max(x) if isinstance(x, list) else x for x in keyframes[1]))
        sleep(max_time)
        return "keyframe executed in " + str("%.2f" % max_time) + "s"

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, np.asarray(transform))
        return "set transform " + effector_name

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

