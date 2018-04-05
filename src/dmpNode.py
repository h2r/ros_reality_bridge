#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
from std_msgs.msg import String
import numpy as np
from dmp.srv import *
from dmp.msg import *
from numpy import linspace
from dataFilter import DataFilter 

class LFDHandler(object):
    def __init__(self):
        # data for all the demonstrations
        self.traj_data = []
        self.supports = []

        self.curr_skill_data = []
        self.curr_skill_supports = [] # index of critical points in traj_data
        
        # start / end points for each skill (note end of i-th is start of i+1-th)
        self.endpoints = []
        self.gripper_cmds = []

        # the lfd requests for all the skills
        self.LFDRequests = []

        # the current loc of the end effector
        self.curr_loc = None

        self.pub = rospy.Publisher("/ein/right/forth_commands", String, queue_size = 0)

    #Learn a DMP from demonstration data
    def makeLFDRequest(self, dims, traj, dt, K_gain, 
                       D_gain, num_bases):
        demotraj = DMPTraj()
            
        for i in xrange(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
                
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
            
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"    
                
        return resp;


    #Set a DMP as active for planning
    def makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    #Generate a plan from a DMP
    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, 
                        seg_length, tau, dt, integrate_iter):
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"   
                
        return resp;


    def moveToGlobalStart(self):
        # code to linearly get to start position
        curr_loc = self.curr_loc
        num_steps = 20
        start = self.endpoints[0]
        x_vec = linspace(curr_loc[0], start[0], num=num_steps)
        y_vec = linspace(curr_loc[1], start[1], num=num_steps)
        z_vec = linspace(curr_loc[2], start[2], num=num_steps)
        for i in xrange(len(x_vec)):
            self.curr_loc = [x_vec[i], y_vec[i], z_vec[i]]
            msg = str(x_vec[i]) + ' ' + str(y_vec[i]) + ' ' + str(z_vec[i]) + ' 0 1 0 0 moveToEEPose' 
            self.pub.publish(msg)
            rospy.sleep(0.2)
        rospy.sleep(2)

    def onEOS(self):
        if len(self.curr_skill_data) == 0:
            return
        filtered_data = DataFilter().filter(np.array(self.curr_skill_data), np.array(self.curr_skill_supports))
        dims = 3                
        dt = 0.1 #1.0                
        K = 100                 
        D = 2.0 * np.sqrt(K)      
        # num_bases = 4
        num_bases = len(self.supports)          
        # traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
        self.LFDRequests.append(self.makeLFDRequest(dims, filtered_data, dt, K, D, num_bases))

    def onEXE(self, skill_num):
        #Set it as the active DMP
        if self.LFDRequests == None or len(self.LFDRequests) == 0:
            return
        if skill_num >= len(self.LFDRequests) or skill_num < 0:
            return

        self.makeSetActiveRequest(self.LFDRequests[skill_num].dmp_list)

        #Now, generate a plan (with same start and end as demonstration)
        x_0 = self.endpoints[skill_num]#[0.0,0.0]          #Plan starting at a different point than demo 
        x_dot_0 = [0.0,0.0,0.0]   
        t_0 = 0                
        goal = self.endpoints[skill_num+1] #[8.0,7.0]         #Plan to a different goal than demo
        goal_thresh = [0.001, 0.001, 0.001]#[0.2,0.2,0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = self.LFDRequests[skill_num].tau       #Desired plan should take as long as demo
        dt = 0.5
        integrate_iter = 1       #dt is rather large, so this is > 1  
        plan_meta = self.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                               seg_length, tau, dt, integrate_iter)
        if plan_meta == None:
            return

        # save the parameterized motion plan
        plan = plan_meta.plan.points
        
        if plan == None:
            return

        # code to execute the motion plan
        for dp in plan:
            point = dp.positions
            self.curr_loc = point
            msg = str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + ' 0 1 0 0 moveToEEPose' 
            self.pub.publish(msg)
            rospy.sleep(0.2)

    def interpGripperCommand(self, cmd):
        if cmd not in xrange(0, 3):
            assert False
        if cmd == 1: # case where we should open (should really use better style here...)
            self.pub.publish('openGripper')
        elif cmd == 0:
            self.pub.publish('closeGripper')

    def callback(self, data):
        if data == None:
            return
        msg = data.data.split()
        if msg[0] == "EXE":
            # case where we want to execute a motion plan
            data = msg[1:]
            assert len(data) % 4 == 0 # x,y,z,gripper_cmd for each point
            num_endpoints = len(data)/4
            assert num_endpoints > 1
            self.endpoints = []
            self.gripper_cmds = []
            for i in xrange(0, len(data), 4):
                self.endpoints.append([float(data[j]) for j in xrange(i, i+3)])
                self.gripper_cmds.append(int(data[i+3]))
            self.moveToGlobalStart()
            assert len(self.endpoints)-1 == len(self.LFDRequests)
            self.interpGripperCommand(self.gripper_cmds[0])
            for i in xrange(len(self.traj_data)):
                self.onEXE(i) # sets active LFD and executes plan
                rospy.sleep(1)
                self.interpGripperCommand(self.gripper_cmds[i+1])
                rospy.sleep(2)
        elif msg[0] == "EOS": # end of skill
            if len(self.curr_skill_data) != 0:
                self.traj_data.append(self.curr_skill_data)
                self.supports.append(self.curr_skill_supports)
                self.onEOS() # filters data and makes and LFDrequest
                self.curr_skill_data = []
                self.curr_skill_supports = []
        elif msg[0] == "PT":
            # case where a point is getting sent
            assert len(msg) == 5
            self.curr_loc = [float(dim) for dim in msg[1:-1]]
            self.curr_skill_data.append(self.curr_loc)
            if float(msg[-1]) == 1:
                self.curr_skill_supports.append(len(self.curr_skill_data) - 1)
        

    def start(self):
        rospy.init_node('dmp_bridge')
        # note dmp_train_data is created on the unity side as part of rosbridge
        rospy.Subscriber('dmp_train_data', String, self.callback)
        rospy.spin()
    

if __name__ == '__main__':
    handler = LFDHandler()
    handler.start()