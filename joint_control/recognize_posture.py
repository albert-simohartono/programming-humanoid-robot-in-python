'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import *
from os import listdir
import pickle
import numpy as np
ROBOT_POSE_CLF = 'robot_pose.pkl'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF, 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        #self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF, 'rb'))
        data = []
        joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
                  'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']

        for i in joints:
            data.append(perception.joint[i])

        data.append(perception.imu[0])
        data.append(perception.imu[1])

        prediction = self.posture_classifier.predict([data])
        posture = listdir('robot_pose_data')[prediction[0]]
        print(posture)
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
