#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
from std_msgs.msg import String
import numpy as np
from dmp.srv import *
from dmp.msg import *

class LFDHandler(object):
    def __init__(self):
        self.traj_data = []
        self.plan = None

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
        rospy.loginfo("GOT EOF")
        rospy.loginfo("NUM DATA POINTS: " + str(len(self.traj_data)))
        dims = 3                
        dt = 1.0                
        K = 100                 
        D = 2.0 * np.sqrt(K)      
        # num_bases = 4
        num_bases = len(self.traj_data)          
        # traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
        resp = self.makeLFDRequest(dims, self.traj_data, dt, K, D, num_bases)

        #Set it as the active DMP
        self.makeSetActiveRequest(resp.dmp_list)

        #Now, generate a plan (with same start and end as demonstration)
        x_0 = self.traj_data[0]#[0.0,0.0]          #Plan starting at a different point than demo 
        x_dot_0 = [0.0,0.0,0.0]   
        t_0 = 0                
        goal = self.traj_data[-1] #[8.0,7.0]         #Plan to a different goal than demo
        goal_thresh = [0.2,0.2,0.2]
        seg_length = -1          #Plan until convergence to goal
        tau = 2 * resp.tau       #Desired plan should take twice as long as demo
        dt = 1.0
        integrate_iter = 5       #dt is rather large, so this is > 1  
        plan = self.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                               seg_length, tau, dt, integrate_iter)

        # save the parameterized motion plan
        self.plan = plan.plan.points
        rospy.loginfo(plan.plan.points)
        self.traj_data = []


    def onEXE(self):
        if self.plan == None:
            return
        # case where we are going to publish the motion plan and move the robot
        topic = "/ein/right/forth_commands"
        pub = rospy.Publisher(topic, String, queue_size = 0)
        for dp in self.plan:
            point = dp.positions
            msg = str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + ' 0 1 0 0 moveToEEPose' 
            pub.publish(msg)
            rospy.sleep(0.1)


    def callback(self, data):
        if data.data == "EOF":
            self.onEOF()
            return

        msg = data.data.split()
        if msg[0] == "EXE":
            # case where we want to execute a motion plan
            self.onEXE()
        else:
            # case where a point is getting sent
            self.traj_data.append([float(dim) for dim in msg])
            rospy.loginfo(str(self.traj_data))
        

    def start(self):
        rospy.init_node('dmp_bridge')

        # note dmp_train_data is created on the unity side as part of rosbridge
        rospy.Subscriber('dmp_train_data', String, self.callback)
        rospy.spin()
    

if __name__ == '__main__':
    handler = LFDHandler()
    handler.start()