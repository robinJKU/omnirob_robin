from __future__ import division
import math
import os
import random
import time
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, qWarning, QTimer, QEvent
from python_qt_binding.QtGui import QIcon, QMenu, QTreeWidgetItem, QWidget, QLabel, QCheckBox

import rospkg
import rospy
import genpy
import rosservice
import tf_conversions

from rqt_py_common.extended_combo_box import ExtendedComboBox
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion


class OmnirobDashboardWidget(QWidget):
    column_names = ['service', 'type', 'expression']
    
    def __init__(self):
        super(OmnirobDashboardWidget, self).__init__()
        self.setObjectName('OmnirobDashboardWidget')
        
        # create context for the expression eval statement
        self._eval_locals = {}
        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('omnirob_robin_rqt'), 'resource', 'OmnirobDashboard.ui')
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox})

        self._column_index = {}
        for column_name in self.column_names:
            self._column_index[column_name] = len(self._column_index)
        
        if rospy.has_param('robot_description'):     
            robot = URDF.from_parameter_server()
            for joint in robot.joints:          
                if joint.name == 'lwa/joint_1':
                    self.lwa_goal_1.setMinimum(joint.limit.lower)
                    self.lwa_goal_1.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_2':
                    self.lwa_goal_2.setMinimum(joint.limit.lower)
                    self.lwa_goal_2.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_3':
                    self.lwa_goal_3.setMinimum(joint.limit.lower)
                    self.lwa_goal_3.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_4':
                    self.lwa_goal_4.setMinimum(joint.limit.lower)
                    self.lwa_goal_4.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_5':
                    self.lwa_goal_5.setMinimum(joint.limit.lower)
                    self.lwa_goal_5.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_6':
                    self.lwa_goal_6.setMinimum(joint.limit.lower)
                    self.lwa_goal_6.setMaximum(joint.limit.upper)
                if joint.name == 'lwa/joint_7':
                    self.lwa_goal_7.setMinimum(joint.limit.lower)
                    self.lwa_goal_7.setMaximum(joint.limit.upper)
                if joint.name == 'pan_tilt/pan_joint':
                    self.pan_tilt_goal_1.setMinimum(joint.limit.lower)
                    self.pan_tilt_goal_1.setMaximum(joint.limit.upper)
                if joint.name == 'pan_tilt/tilt_joint':
                    self.pan_tilt_goal_2.setMinimum(joint.limit.lower)
                    self.pan_tilt_goal_2.setMaximum(joint.limit.upper)
                if joint.name == 'gripper/finger_left_joint':
                    self.gripper_goal.setMinimum(joint.limit.lower*1000*2)
                    self.gripper_goal.setMaximum(joint.limit.upper*1000*2)
                    
        self.auto = 0
        self.counter = 0 
        self.mode = 0             
        self._twist = Twist() 
        self.base_activated = False
        self._pub =  rospy.Publisher('/omnirob_robin/base/drives/control/cmd_vel', Twist, queue_size = 1)
        self._goal_pub =  rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
        self._lwa_state_callback_count = 0
        
        self._lwa_goal_had_focus = [False for i in range(7)]
        self._pan_tilt_goal_had_focus = [False for i in range(2)]
        self._gripper_goal_had_focus = False
        
        self._timer_refresh_state = QTimer(self)
        self._timer_refresh_state.timeout.connect(self._timer_update)
        self.start()
        
        self._sub_lwa_state = rospy.Subscriber('omnirob_robin/lwa/state/joint_state_array', Float64MultiArray, self.lwa_state_callback)
        self._gripper_lwa_state = rospy.Subscriber('omnirob_robin/pan_tilt/state/joint_state_array', Float64MultiArray, self.pan_tilt_state_callback)
        self._sub_pan_tilt_state = rospy.Subscriber('omnirob_robin/gripper/state/joint_state_array', Float64MultiArray, self.gripper_state_callback)
        
        
    def start(self):
        self._timer_refresh_state.start(10)                  

    #def save_settings(self, plugin_settings, instance_settings):

    def shutdown_plugin(self):
        self._timer_refresh_state.stop()
        self._sub_lwa_state.unregister()
        self._sub_gripper_state.unregister()
        self._sub_pan_tilt_state.unregister()

    #def restore_settings(self, plugin_settings, instance_settings):

    #def trigger_configuration(self):
    
    def _timer_update(self):
        self._publish_twist()
        self._update_goals()
        self._auto_move()
        
    def _update_goals(self):
        
        #lwa
        if self.lwa_goal_1.hasFocus():
            self._lwa_goal_had_focus[0] = True
            
        if self.lwa_goal_2.hasFocus():
            self._lwa_goal_had_focus[1] = True
            
        if self.lwa_goal_3.hasFocus():
            self._lwa_goal_had_focus[2] = True
            
        if self.lwa_goal_4.hasFocus():
            self._lwa_goal_had_focus[3] = True
            
        if self.lwa_goal_5.hasFocus():
            self._lwa_goal_had_focus[4] = True
            
        if self.lwa_goal_6.hasFocus():
            self._lwa_goal_had_focus[5] = True
           
        if self.lwa_goal_7.hasFocus():
            self._lwa_goal_had_focus[6] = True            
            
            
        if not self._lwa_goal_had_focus[0]:
            self.lwa_goal_1.setValue(float(self.lwa_state_1.text()))

        if not self._lwa_goal_had_focus[1]:
            self.lwa_goal_2.setValue(float(self.lwa_state_2.text()))
        
        if not self._lwa_goal_had_focus[2]:
            self.lwa_goal_3.setValue(float(self.lwa_state_3.text()))

        if not self._lwa_goal_had_focus[3]:
            self.lwa_goal_4.setValue(float(self.lwa_state_4.text()))

        if not self._lwa_goal_had_focus[4]:
            self.lwa_goal_5.setValue(float(self.lwa_state_5.text()))

        if not self._lwa_goal_had_focus[5]:
            self.lwa_goal_6.setValue(float(self.lwa_state_6.text()))

        if not self._lwa_goal_had_focus[6]:
            self.lwa_goal_7.setValue(float(self.lwa_state_7.text()))
            
        #pan_tilt
        if self.pan_tilt_goal_1.hasFocus():
            self._pan_tilt_goal_had_focus[0] = True
            
        if self.pan_tilt_goal_2.hasFocus():
            self._pan_tilt_goal_had_focus[1] = True
            
        if not self._pan_tilt_goal_had_focus[0]:
            self.pan_tilt_goal_1.setValue(float(self.pan_tilt_state_1.text()))

        if not self._pan_tilt_goal_had_focus[1]:
            self.pan_tilt_goal_2.setValue(float(self.pan_tilt_state_2.text()))
            
        #gripper
        if self.gripper_goal.hasFocus():
            self._gripper_goal_had_focus = True
            
        if not self._gripper_goal_had_focus:
            self.gripper_goal.setValue(float(self.gripper_state.text()))    
           
    def _reset_focus_lwa(self):
        for i in range(7):
            self._lwa_goal_had_focus[i] = False       
            
    def _reset_focus_pan_tilt(self):
        for i in range(2):
            self._pan_tilt_goal_had_focus[i] = False  
            
    def _reset_focus_gripper(self):
        self._gripper_goal_had_focus = False  
    

    def _publish_twist(self):
               
        if self.base_activated:
            self._pub.publish(self._twist)
         
    def _auto_move(self):
        self.counter = self.counter + 1        
        if self.auto == 1:
            self.auto_timer.setText("{:.3f}".format(30-self.counter/100.0)) 
            if self.counter > 3000:        
                if self.mode == 0:
                    self._move_base_pose(0.8, 0.8, 0)  #ecke hinten
                elif self.mode == 1:
                    self._move_base_pose(1.5, 1.5, 1.57)   #mitte
                elif self.mode == 2:
                    self._move_base_pose(2.8, 2.8, 1.57)  #vorne ecke           
                elif self.mode == 3:
                    self._move_base_pose(2.7, 1.0, 1.57+3.14)   #tisch             
                elif self.mode == 4:
                    self._move_base_pose(1.7, 2.9, 1.57)  #gruener kasten 
                self.counter = 0
                self.mode = self.mode + 1
                self.mode = self.mode % 5
 
    def lwa_state_callback(self, data):        
        self.lwa_state_1.setText("{:.3f}".format(data.data[0]))
        self.lwa_state_2.setText("{:.3f}".format(data.data[1]))
        self.lwa_state_3.setText("{:.3f}".format(data.data[2]))
        self.lwa_state_4.setText("{:.3f}".format(data.data[3]))
        self.lwa_state_5.setText("{:.3f}".format(data.data[4]))
        self.lwa_state_6.setText("{:.3f}".format(data.data[5]))
        self.lwa_state_7.setText("{:.3f}".format(data.data[6]))   

    def pan_tilt_state_callback(self, data):
        self.pan_tilt_state_1.setText("{:.3f}".format(data.data[0]))
        self.pan_tilt_state_2.setText("{:.3f}".format(data.data[1]))


    def gripper_state_callback(self, data):
        self.gripper_state.setText("{:.0f}".format(data.data[0]))
       

    @Slot()
    def refresh_state(self):
        self.test_label.setText('test')   
        
    def call_service( self, string ):
        service_name = string
        service_class = rosservice.get_service_class_by_name( service_name )
        service = rospy.ServiceProxy(service_name, service_class)
        request = service_class._request_class()
        try:
            response = service()
        except rospy.ServiceException as e:
            qWarning('service_caller: request:\n%r' % (request))
            qWarning('service_caller: error calling service "%s":\n%s' % (self._service_info['service_name'], e))           
        #else:
            #print(response)
            
    @Slot()
    def on_demo_move_clicked(self):
        self.call_service('/omnirob_robin/base_demo_detect_srv')
        
    @Slot()
    def on_demo_localize_clicked(self):
        self.call_service('/omnirob_robin/base_demo_localize_srv')
        
    @Slot()
    def on_demo_detect_clicked(self):
        self.call_service('/omnirob_robin/detect_objects_srv')
        
    @Slot()
    def on_demo_move_center_clicked(self):
        self._move_base_pose(1.5, 1.5, -1.57)
        
    @Slot()
    def on_demo_start_auto_clicked(self):
        self.auto = 1
        self.counter = 6000;
        
    @Slot()
    def on_demo_stop_auto_clicked(self):
        self.auto = 0
            
##### Base
    def _base_move(self, x, y, yaw):
        twist = Twist()
        twist.linear.x = x;
        twist.linear.y = y;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = yaw; 
        self._twist =  twist

    def _base_stop_motion(self):
	self.base_activated = False
        self._base_move(0.0, 0.0, 0.0)
	self._pub.publish(self._twist)
    
    def _move_base_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        self._goal_pub.publish(pose)
	
        
    @Slot()
    def on_base_can_start_clicked(self):
        self.call_service( '/omnirob_robin/base/canserver/start' )
        rospy.sleep(1.0)
        self.call_service( '/omnirob_robin/base/drives/control/start' )
     
    @Slot()    
    def on_base_can_stop_clicked(self):
        self.call_service( '/omnirob_robin/base/drives/control/stop' )
        rospy.sleep(1.0)
        self.call_service( '/omnirob_robin/base/canserver/stop' )        
            
    @Slot()
    def on_base_forward_pressed(self):
        self._base_move(0.3, 0.0, 0.0)
	self.base_activated = True
 
    @Slot()
    def on_base_forward_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_back_pressed(self):
        self._base_move(-0.3, 0.0, 0.0)
	self.base_activated = True
 
    @Slot()
    def on_base_back_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_left_pressed(self):
        self._base_move(0.0, 0.3, 0.0)
	self.base_activated = True
 
    @Slot()
    def on_base_left_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_right_pressed(self):
        self._base_move(0.0, -0.3, 0.0)
	self.base_activated = True
 
    @Slot()
    def on_base_right_released(self):
        self._base_stop_motion()

    @Slot()
    def on_base_turn_left_pressed(self):
        self._base_move(0.0, 0.0, 0.5)
	self.base_activated = True
 
    @Slot()
    def on_base_turn_left_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_turn_right_pressed(self):
        self._base_move(0.0, 0.0, -0.5)
	self.base_activated = True
 
    @Slot()
    def on_base_turn_right_released(self):
        self._base_stop_motion()
        
    #diagonal
    @Slot()
    def on_base_forward_left_pressed(self):
        self._base_move(0.2, 0.2, 0.0)
	self.base_activated = True
     
    @Slot()
    def on_base_forward_left_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_forward_right_pressed(self):
        self._base_move(0.2, -0.2, 0.0)
	self.base_activated = True
     
    @Slot()
    def on_base_forward_right_released(self):
        self._base_stop_motion()
        
     
    @Slot()   
    def on_base_back_left_pressed(self):
        self._base_move(-0.2, 0.2, 0.0)
	self.base_activated = True
     
    @Slot()
    def on_base_back_left_released(self):
        self._base_stop_motion()
        
    @Slot()
    def on_base_back_right_pressed(self):
        self._base_move(-0.2, -0.2, 0.0)
	self.base_activated = True
     
    @Slot()
    def on_base_back_right_released(self):
        self._base_stop_motion()
       
        
            
            
##### LWA
    def _lwa_start_motion(self):
        self.call_service( '/omnirob_robin/lwa/control/start_motion' )
         
    @Slot()
    def on_lwa_initialize_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/initialize_modules' )
    
    @Slot()
    def on_lwa_acknowledge_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/send_acknowledge_to_modules' )
    
    @Slot()
    def on_lwa_start_motion_clicked(self):
        self._lwa_start_motion()
    
    @Slot()
    def on_lwa_stop_motion_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/stop_motion' )
    
    @Slot()
    def on_lwa_enable_point_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/enable_point_to_point_motion' )
    
    @Slot()
    def on_lwa_test_clicked(self):
        pub = rospy.Publisher('omnirob_robin/lwa/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._lwa_start_motion()
        self._reset_focus_lwa()
        
    @Slot()
    def on_lwa_home_clicked(self):
        pub = rospy.Publisher('omnirob_robin/lwa/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._lwa_start_motion()
        self._reset_focus_lwa()
        
    @Slot()
    def on_lwa_move_clicked(self):
        pub = rospy.Publisher('omnirob_robin/lwa/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.data[0] = self.lwa_goal_1.value()
        msg.data[1] = self.lwa_goal_2.value()
        msg.data[2] = self.lwa_goal_3.value()
        msg.data[3] = self.lwa_goal_4.value()
        msg.data[4] = self.lwa_goal_5.value()
        msg.data[5] = self.lwa_goal_6.value()
        msg.data[6] = self.lwa_goal_7.value()        
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._lwa_start_motion()
        self._reset_focus_lwa()

    @Slot()
    def on_lwa_reference_all_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_1' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_2' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_3' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_4' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_5' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_6' )
	self.call_service( '/omnirob_robin/lwa/control/reference_module_7' )        
    @Slot()
    def on_lwa_reference_1_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_1' )        
    @Slot()
    def on_lwa_reference_2_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_2' )        
    @Slot()
    def on_lwa_reference_3_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_3' )        
    @Slot()
    def on_lwa_reference_4_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_4' )
    @Slot()
    def on_lwa_reference_5_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_5' )
    @Slot()
    def on_lwa_reference_6_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_6' )
    @Slot()
    def on_lwa_reference_7_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reference_module_7' )
        
    @Slot()
    def on_lwa_reboot_1_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_1' )        
    @Slot()
    def on_lwa_reboot_2_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_2' )        
    @Slot()
    def on_lwa_reboot_3_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_3' )        
    @Slot()
    def on_lwa_reboot_4_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_4' )
    @Slot()
    def on_lwa_reboot_5_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_5' )
    @Slot()
    def on_lwa_reboot_6_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_6' )
    @Slot()
    def on_lwa_reboot_7_clicked(self):
        self.call_service( '/omnirob_robin/lwa/control/reboot_module_7' )
      
      
##### Pan Tilt
    def _pan_tilt_start_motion(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/start_motion' )
         
    @Slot()
    def on_pan_tilt_initialize_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/initialize_modules' )
    
    @Slot()
    def on_pan_tilt_acknowledge_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/send_acknowledge_to_modules' )
    
    @Slot()
    def on_pan_tilt_start_motion_clicked(self):
        self._pan_tilt_start_motion()
    
    @Slot()
    def on_pan_tilt_stop_motion_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/stop_motion' )
    
    @Slot()
    def on_pan_tilt_enable_point_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/enable_point_to_point_motion' )
    
    @Slot()
    def on_pan_tilt_home_clicked(self):
        pub = rospy.Publisher('omnirob_robin/pan_tilt/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._pan_tilt_start_motion()
        
    @Slot()
    def on_pan_tilt_test_clicked(self):
        pub = rospy.Publisher('omnirob_robin/pan_tilt/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [-0.2, 0.2]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._pan_tilt_start_motion()
        
    @Slot()
    def on_pan_tilt_move_clicked(self):
        pub = rospy.Publisher('omnirob_robin/pan_tilt/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        msg.data[0] = self.pan_tilt_goal_1.value()
        msg.data[1] = self.pan_tilt_goal_2.value()    
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._pan_tilt_start_motion()
    
    @Slot()
    def on_pan_tilt_reboot_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/reboot_modules' ) 
        
    @Slot()
    def on_pan_tilt_reference_clicked(self):
        self.call_service( '/omnirob_robin/pan_tilt/control/reference_modules' ) 

##### gripper
    def _gripper_start_motion(self):
        self.call_service( '/omnirob_robin/gripper/control/start_motion' )
         
    @Slot()
    def on_gripper_initialize_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/initialize_modules' )
    
    @Slot()
    def on_gripper_acknowledge_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/send_acknowledge_to_modules' )
    
    @Slot()
    def on_gripper_start_motion_clicked(self):
        self._gripper_start_motion()
    
    @Slot()
    def on_gripper_stop_motion_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/stop_motion' )
    
    @Slot()
    def on_gripper_enable_point_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/enable_point_to_point_motion' )
        
    @Slot()
    def on_gripper_move_clicked(self):
        pub = rospy.Publisher('omnirob_robin/gripper/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [0.0]
        msg.data[0] = self.gripper_goal.value()   
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._gripper_start_motion()
        
    @Slot()
    def on_gripper_open_clicked(self):
        pub = rospy.Publisher('omnirob_robin/gripper/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [10.0]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._gripper_start_motion()
    
    @Slot()
    def on_gripper_close_clicked(self):
        pub = rospy.Publisher('omnirob_robin/gripper/control/commanded_joint_state', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [50.0]
        rospy.sleep(1.0)
        pub.publish(msg)
        rospy.sleep(1.0)
        self._gripper_start_motion()
    
    @Slot()
    def on_gripper_reboot_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/reboot_modules' ) 
        
    @Slot()
    def on_gripper_reference_clicked(self):
        self.call_service( '/omnirob_robin/gripper/control/reference_modules' ) 
        
 
    
