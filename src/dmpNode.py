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
        self.traj_data = []
        self.supports = [] # index of critical points in traj_data
        self.plan = None
        self.LFDRequest = None
        self.curr_loc = None


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


    def onEOF(self):
        # rospy.loginfo("GOT EOF")
        # rospy.loginfo("NUM DATA POINTS: " + str(len(self.traj_data)))
        filtered_data = DataFilter().filter(np.array(self.traj_data), np.array(self.supports))
        dims = 3                
        dt = 0.1 #1.0                
        K = 100                 
        D = 2.0 * np.sqrt(K)      
        # num_bases = 4
        num_bases = len(self.supports)          
        # traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
        self.LFDRequest = self.makeLFDRequest(dims, filtered_data, dt, K, D, num_bases)

    def onEXE(self, s, g):
        #Set it as the active DMP
        if self.LFDRequest == None:
            return

        self.makeSetActiveRequest(self.LFDRequest.dmp_list)

        #Now, generate a plan (with same start and end as demonstration)
        x_0 = s#[0.0,0.0]          #Plan starting at a different point than demo 
        x_dot_0 = [0.0,0.0,0.0]   
        t_0 = 0                
        goal = g #[8.0,7.0]         #Plan to a different goal than demo
        goal_thresh = [0.01, 0.01, 0.01]#[0.2,0.2,0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = self.LFDRequest.tau       #Desired plan should take as long as demo
        dt = 0.5
        integrate_iter = 1       #dt is rather large, so this is > 1  
        plan = self.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                               seg_length, tau, dt, integrate_iter)
        if plan == None:
            return

        # save the parameterized motion plan
        self.plan = plan.plan.points
        
        if self.plan == None:
            return


        # case where we are going to publish the motion plan and move the robot
        topic = "/ein/right/forth_commands"
        pub = rospy.Publisher(topic, String, queue_size = 0)

        # code to linearly get to start position
        curr_loc = self.curr_loc
        num_steps = 20
        x_vec = linspace(curr_loc[0], s[0], num=num_steps)
        y_vec = linspace(curr_loc[1], s[1], num=num_steps)
        z_vec = linspace(curr_loc[2], s[2], num=num_steps)
        for i in xrange(len(x_vec)):
            msg = str(x_vec[i]) + ' ' + str(y_vec[i]) + ' ' + str(z_vec[i]) + ' 0 1 0 0 moveToEEPose' 
            pub.publish(msg)
            rospy.sleep(0.2)

        rospy.sleep(2)

        # code to execute the motion plan
        for dp in self.plan:
            point = dp.positions
            msg = str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + ' 0 1 0 0 moveToEEPose' 
            pub.publish(msg)
            rospy.sleep(0.2)

        self.traj_data = []

    def callback(self, data):
        if data == None:
            return

        msg = data.data.split()
        if msg[0] == "EOF":
            self.onEOF()
        elif msg[0] == "EXE":
            # case where we want to execute a motion plan

            s = [float(msg[i]) for i in xrange(1, 4)]
            g = [float(msg[i]) for i in xrange(4, 7)]
            self.onEXE(s, g)
        else:
            # case where a point is getting sent
            assert len(msg) == 4
            self.curr_loc = [float(dim) for dim in msg[:-1]]
            self.traj_data.append(self.curr_loc)
            if float(msg[-1]) == 1:
                self.supports.append(len(self.traj_data) - 1)
        

    def start(self):
        rospy.init_node('dmp_bridge')

        # note dmp_train_data is created on the unity side as part of rosbridge
        rospy.Subscriber('dmp_train_data', String, self.callback)
        rospy.spin()
    

if __name__ == '__main__':
    handler = LFDHandler()
    handler.start()