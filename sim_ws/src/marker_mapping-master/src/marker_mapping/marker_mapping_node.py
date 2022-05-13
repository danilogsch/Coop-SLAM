#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy 

import time, threading

from robotnik_msgs.msg import State
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray, TransformStamped, PoseWithCovarianceStamped, Pose2D
from marker_mapping.srv import InitPoseFromMarker, SaveMarker
from marker_mapping.msg import MarkerMappingState

from tf import TransformListener, Exception as tfException, ConnectivityException, LookupException, ExtrapolationException
import tf
from tf2_ros import TransformBroadcaster
import numpy
from tf import transformations

import rospkg
import os
import exceptions
import yaml



DEFAULT_FREQ = 10.0
MAX_FREQ = 500.0
MARKER_TIMER = 2.0
DEFAULT_FRAME_ID = 'map'
DEFAULT_BASE_FRAME_ID = 'base_link'
MAX_MARKER_ID = 10000
MIN_MARKER_ID = 0

	
# Class Template of Robotnik component for Pyhton
class MarkerMapping:
	
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
		self._frame_id = args['frame_id'] 
		self._base_frame_id = args['base_frame_id'] 
		self._publish_saved_markers_tf = args['publish_saved_markers_tf'] 
		self._folder_path = args['folder_path'] 
		self._markers_filename = args['markers_filename'] 
		self._load_markers_on_init = args['load_markers_on_init'] 
		self._max_marker_id = args['max_marker_id'] 
		self._min_marker_id = args['min_marker_id'] 
		
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
	
	
		self.real_freq = 0.0
		
		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = MarkerMappingState()
		# Timer to publish state
		self.publish_state_timer = 1
		# Saves the time with 
		self.last_marker_time = rospy.Time.now()
		# Saves the received markers
		self._marker_msg = AlvarMarkers()
		# Dict to save the markers
		self._marker_dict = {}
		# The covariance set whenever setting the global pose
		self._default_pose_covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		
			
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		
		if self._load_markers_on_init:
			if self.loadMarkersFromFile(self._folder_path, self._markers_filename) != 0:
				rospy.loginfo("%s::setup: Error loading config file",self.node_name)
				
		# Create local folder if it does not exist
		if self.createDirectoryTree(self._folder_path) == 0:
			rospy.loginfo("%s::setup: Created config folder %s",self.node_name, self._folder_path)	
			self.initialized = True
			return 0
		else:
			return -1
		
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		
		self._transform_listener = TransformListener()
		self._transform_broadcaster = TransformBroadcaster()

		# Publishers
		self._state_publisher = rospy.Publisher('~state', MarkerMappingState, queue_size=10)
		self._initialpose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
		self._global_pose_publisher = rospy.Publisher('/global_pose', Pose2D, queue_size=10)
		# Subscribers
		# topic_name, msg type, callback, queue_size
		self.markers_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.markersCb, queue_size = 10)
		
		
		# Service Servers
		self.save_marker_service_server = rospy.Service('~save_maker', SaveMarker, self.saveMarkerServiceCb)
		self.init_pose_from_marker_service_server = rospy.Service('~init_pose_from_marker', InitPoseFromMarker, self.initPoseFromMarkerServiceCb)
		# Service Clients
		# self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
		# ret = self.service_client.call(ServiceMsg)
		
		self.ros_initialized = True
		
		self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)
		
		# Cancels current timers
		self.t_publish_state.cancel()
		
		self._state_publisher.unregister()
		
		self.initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			
			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0
		
		
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		t_now = rospy.Time.now()
				
		if self._publish_saved_markers_tf:
			
			for marker in self._marker_dict:
				marker_pose = self._marker_dict[marker].pose
				transform = TransformStamped()
				transform.header.stamp = t_now
				transform.header.frame_id = marker_pose.header.frame_id
				transform.child_frame_id = 'marker_%d'%self._marker_dict[marker].id
				transform.transform.translation.x = marker_pose.pose.position.x
				transform.transform.translation.y = marker_pose.pose.position.y
				transform.transform.translation.z = marker_pose.pose.position.z
				transform.transform.rotation.x = marker_pose.pose.orientation.x
				transform.transform.rotation.y = marker_pose.pose.orientation.y
				transform.transform.rotation.z = marker_pose.pose.orientation.z
				transform.transform.rotation.w = marker_pose.pose.orientation.w
				self._transform_broadcaster.sendTransform(transform)
		
		# publish the global pose based on the transform from frame_id (map) to base_frame_id (base_footprint)
		if self._transform_listener.frameExists(self._frame_id) and self._transform_listener.frameExists(self._base_frame_id):
				
			try:
				t = self._transform_listener.getLatestCommonTime(self._frame_id, self._base_frame_id)
				position, quaternion = self._transform_listener.lookupTransform(self._frame_id, self._base_frame_id, t)
				pose2d = Pose2D()
				pose2d.x = position[0]
				pose2d.y = position[1]
				euler = tf.transformations.euler_from_quaternion(quaternion)
				pose2d.theta = euler[2]
				self._global_pose_publisher.publish(pose2d)
				
				#print position, quaternion
			except tfException, e:
				rospy.logerr('%s::rosPublish: %s'%(self.node_name, e))
			except ConnectivityException, e:
				rospy.logerr('%s::rosPublish: %s'%(self.node_name, e))
			except LookupException, e:
				rospy.logerr('%s::rosPublish: %s'%(self.node_name, e))
			except ExtrapolationException, e:
				rospy.logerr('%s::rosPublish: %s'%(self.node_name, e))			
					
		return 0
		
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		
		if not self.initialized:
			self.setup()
			
		else: 		
			self.switchToState(State.STANDBY_STATE)
		
		
		return
	
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		t_now = rospy.Time.now()
		diff = (t_now - self.last_marker_time).to_sec()
		#print self.last_marker_time.to_sec(), t_now.to_sec(), diff
		if diff <= MARKER_TIMER:
			rospy.loginfo('%s::standbyState: marker detected'%self.node_name)
			self.switchToState(State.READY_STATE)
		
		return
	
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		t_now = rospy.Time.now()
		
		
		if (t_now - self.last_marker_time).to_sec() > MARKER_TIMER:
			rospy.loginfo('%s::readyState: marker not detected anymore'%self.node_name)
			self.switchToState(State.STANDBY_STATE)
		
		return
		
	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		
			
		return
	
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
		
		return
	
		
	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'
	
		
	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state.state = self.state
		self.msg_state.state.state_description = self.stateToString(self.state)
		self.msg_state.state.desired_freq = self.desired_freq
		self.msg_state.state.real_freq = self.real_freq
		self.msg_state.ids_detected = []
		for marker in self._marker_msg.markers:
			# Filter markers by max & min
			self.msg_state.ids_detected.append(marker.id)
		self._state_publisher.publish(self.msg_state)
		
		
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
	
	
	def markersCb(self, msg):
		'''
			Callback for inelfe_video_manager state
			@param msg: received message
			@type msg: std_msgs/Int32
		'''
		#rospy.loginfo('MarkerMapping:topicCb')
		self._marker_msg = msg
		if len(msg.markers) > 0:
			# Filter the markers by min & max id
			for marker in msg.markers:
				if marker.id >= self._min_marker_id and marker.id <= self._max_marker_id:
					self.last_marker_time = marker.header.stamp
	

	
	def saveMarkerServiceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		#rospy.loginfo('MarkerMapping:saveMarkerServiceCb')	
		# saving by ID not implemented yet
		# saving into custom filename not implemented yet
		
		# only in ready
		if self.state == State.READY_STATE:
			self.msg_state.ids_recorded = []
			for marker in self._marker_msg.markers:
				# Filter markers by max & min
				if marker.id >= self._min_marker_id and marker.id <= self._max_marker_id:
					
					if marker.id in self._marker_dict:
						rospy.logwarn('%s:saveMarkerServiceCb: overwritting marker %d',self.node_name, marker.id)	
					else:
						rospy.loginfo('%s:saveMarkerServiceCb: saving marker %d',self.node_name, marker.id)	
					
					ret = self.transformMarker(marker, self._frame_id)
					
					if ret == 0:					
						self._marker_dict[marker.id] = marker
					else:
						rospy.logerr('%s:saveMarkerServiceCb: error transforming marker id  %d',self.node_name, marker.id)
						return False
					#print self._marker_dict
				
			# Saving to file
			self.saveMarkersToFile(self._folder_path, self._markers_filename)	
			
			
			
			return True
		else:
			return False
	
	
	def initPoseFromMarkerServiceCb(self, req):
		'''
			ROS service server. Relocates the robot based on the reading markers and compared with saved ones
			@param req: Required action
			@type req: std_srv/Empty
		'''
		# Functionaly of selected id not implemented yet
		
		# only in ready
		if self.state == State.READY_STATE:
			
			
			if self._transform_listener.frameExists(self._frame_id) and self._transform_listener.frameExists(self._base_frame_id):
				
				# look for received markers
				for marker in self._marker_msg.markers:
					# if the marker is in our list
					if marker.id in self._marker_dict:
						############################### FOR TESTING
						try:
							t = self._transform_listener.getLatestCommonTime(self._frame_id, self._base_frame_id)
							position, quaternion = self._transform_listener.lookupTransform(self._frame_id, self._base_frame_id, t)
							rospy.loginfo('%s::initPoseFromMarkerServiceCb: current transform from %s to %s -> (%s) (%s)',self.node_name, self._frame_id, self._base_frame_id, position, quaternion)
							#print position, quaternion
						except tfException, e:
							rospy.logerr('%s::initPoseFromMarkerServiceCb: %s'%(self.node_name, e))
						except ConnectivityException, e:
							rospy.logerr('%s::initPoseFromMarkerServiceCb: %s'%(self.node_name, e))
						except LookupException, e:
							rospy.logerr('%s::initPoseFromMarkerServiceCb: %s'%(self.node_name, e))
						except ExtrapolationException, e:
							rospy.logerr('%s::initPoseFromMarkerServiceCb: %s'%(self.node_name, e))
						#################################
							
						#rospy.logwarn('%s:initPoseFromMarkerServiceCb: using marker %d to set the position',self.node_name, marker.id)	
						
						# Creates a transform matrix from base_frame_id to the current marker
						translation_base_to_marker = (marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z)
						rotation_base_to_marker = (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)
						t_mat44_base_to_marker = self._transform_listener.fromTranslationRotation(translation_base_to_marker, rotation_base_to_marker)
						
						t_mat44_marker_to_base = numpy.linalg.inv(t_mat44_base_to_marker)
						translation_marker_to_base = tuple(transformations.translation_from_matrix(t_mat44_marker_to_base))[:3]
						rotation_marker_to_base = tuple(transformations.quaternion_from_matrix(t_mat44_marker_to_base))
						
						rospy.logwarn('%s:initPoseFromMarkerServiceCb: from base to marker %s, %s',self.node_name, translation_base_to_marker, rotation_base_to_marker)
						rospy.logwarn('%s:initPoseFromMarkerServiceCb: from marker to base %s, %s',self.node_name, translation_marker_to_base, rotation_marker_to_base)
						
						
						# Creates a transform matrix from frame_id (map) to the previously saved marker
						saved_marker = self._marker_dict[marker.id]
						translation_map_to_saved_marker = (saved_marker.pose.pose.position.x, saved_marker.pose.pose.position.y, saved_marker.pose.pose.position.z)
						rotation_map_to_saved_marker = (saved_marker.pose.pose.orientation.x, saved_marker.pose.pose.orientation.y, saved_marker.pose.pose.orientation.z, saved_marker.pose.pose.orientation.w)
						t_mat44_map_to_saved_marker = self._transform_listener.fromTranslationRotation(translation_map_to_saved_marker, rotation_map_to_saved_marker)
						# Calculates the transform from map to base based on the marker
						t_mat44_map_to_base = numpy.dot(t_mat44_map_to_saved_marker, t_mat44_marker_to_base)
						translation_map_to_base = tuple(transformations.translation_from_matrix(t_mat44_map_to_base))[:3]
						rotation_map_to_base = tuple(transformations.quaternion_from_matrix(t_mat44_map_to_base))
						rospy.loginfo('%s::initPoseFromMarkerServiceCb: new transform from %s to %s -> (%s) (%s)',self.node_name, self._frame_id, self._base_frame_id, translation_map_to_base, rotation_map_to_base)
						
						
						# Set the new pose
						msg = PoseWithCovarianceStamped()
						msg.header.stamp = rospy.Time.now()
						msg.header.frame_id = self._frame_id
						msg.pose.covariance = self._default_pose_covariance
						msg.pose.pose.position.x = translation_map_to_base[0]
						msg.pose.pose.position.y = translation_map_to_base[1]
						msg.pose.pose.position.z = translation_map_to_base[2]
						msg.pose.pose.orientation.x = rotation_map_to_base[0]
						msg.pose.pose.orientation.y = rotation_map_to_base[1]
						msg.pose.pose.orientation.z = rotation_map_to_base[2]
						msg.pose.pose.orientation.w = rotation_map_to_base[3]
						self._initialpose_publisher.publish(msg)
						
						return True
						
			else:
				rospy.logerr('%s::initPoseFromMarkerServiceCb: no transform from %s to %s'%(self.node_name, self._frame_id, self._base_frame_id))
				
				return False
				
		else:
			return False
		
		
	def transformMarker(self, marker, frame_id):
		'''
			Transform a marker to the desired frame
		'''
		# sets the header to pose attribute (since it's not present)
		marker.pose.header = marker.header
		
		try:
			new_pose= self._transform_listener.transformPose(frame_id, marker.pose)
			marker.pose = new_pose
			marker.header = marker.pose.header
			return 0
		except tfException, e:
			rospy.logerr('%s::transformMarker: %s'%(self.node_name, e))
		#Another exception when there is no transform
		except ConnectivityException, e:
			rospy.logerr('%s::transformMarker: %s'%(self.node_name, e))
		except LookupException, e:
			rospy.logerr('%s::transformMarker: %s'%(self.node_name, e))
		except ExtrapolationException, e:
			rospy.logerr('%s::transformMarker: %s'%(self.node_name, e))
		
		return -1
		
	
	def createDirectoryTree(self, path):
		"""
			Creates the directory tree to save the logs
			@param path as string
		"""
		try:
			os.makedirs(path)
			return 0
		except exceptions.OSError, e:
			if e.errno != 17:
				rospy.logerr('%s:createDirectoryTree: %s'%(self.node_name,e))
				return -1
			else:
				return 0
	
	
	def saveMarkersToFile(self, folder, filename):
		"""
			Creates the file markers config
		"""
		file_path = folder+'/'+filename
		try:
			rospy.loginfo('%s::saveMarkersToFile: opening file %s',self.node_name, file_path)
			markers_file = file(file_path, 'w')
		except exceptions.OSError, e:
			rospy.logerr('%s:saveMarkersToFile: %s'%(self.node_name,e))
			return -1
		
		for marker_id in self._marker_dict:
			rospy.loginfo('%s::saveMarkersToFile: Saving marker %d',self.node_name, marker_id)
			marker = self._marker_dict[marker_id]	
			markers_file.writelines(['- id: %d\n'%(marker.id), '  frame: %s\n'%(self._frame_id)])
			markers_file.writelines(['  position: [%f, %f, %f]\n'%(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z),
			 '  orientation: [%f, %f, %f, %f]\n'%(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w)])
			self.msg_state.ids_recorded.append(marker_id)
		markers_file.close()
		
		return 0
		
		
	def loadMarkersFromFile(self, folder, filename):
		"""
			Load markers from the config files
		"""
		file_path = folder+'/'+filename
		
		with open(file_path, 'r') as stream:
			try:
				rospy.loginfo('%s::loadMarkersFromFile: opening file %s',self.node_name, file_path)
				
				config_yaml = yaml.load(stream)
			except yaml.YAMLError, e:
				rospy.logerr('%s:loadMarkersFromFile: error parsing yaml file %s. %s'%(self.node_name,	file_path, e))
				return -1
				
			# clear the current list
			self._marker_dict.clear()
			#print config_yaml
			
			try:
				for marker_in_config in config_yaml:
					new_marker = AlvarMarker()
					new_marker.header.frame_id = marker_in_config['frame']
					new_marker.id = marker_in_config['id']
					new_marker.id = marker_in_config['id']
					new_marker.pose.header.frame_id = marker_in_config['frame']
					new_marker.pose.pose.position.x = marker_in_config['position'][0]
					new_marker.pose.pose.position.y = marker_in_config['position'][1]
					new_marker.pose.pose.position.z = marker_in_config['position'][2]
					new_marker.pose.pose.orientation.x = marker_in_config['orientation'][0]
					new_marker.pose.pose.orientation.y = marker_in_config['orientation'][1]
					new_marker.pose.pose.orientation.z = marker_in_config['orientation'][2]
					new_marker.pose.pose.orientation.w = marker_in_config['orientation'][3]
					
					# inserting the new marker
					self._marker_dict[new_marker.id] = new_marker
					self.msg_state.ids_recorded.append(new_marker.id)
					#print self._marker_dict[new_marker.id] 
					
			except KeyError, e:
				self._marker_dict.clear()
				rospy.logerr('%s:loadMarkersFromFile: error parsing yaml file %s. %s'%(self.node_name,	file_path, e))
				return -1
			except IndexError, e:
				self._marker_dict.clear()
				rospy.logerr('%s:loadMarkersFromFile: error parsing yaml file %s. %s'%(self.node_name,	file_path, e))
				return -1
			except TypeError, e:
				self._marker_dict.clear()
				rospy.logerr('%s:loadMarkersFromFile: error parsing yaml file %s. %s'%(self.node_name,	file_path, e))
				return -1
				
			rospy.loginfo('%s::loadMarkersFromFile: yaml loaded successfully',self.node_name)
		
			#print self._marker_dict
		
			return 0
			
		return -1
		
	
		
		
def main():

	rospy.init_node("marker_mapping_node")
	
	
	_name = rospy.get_name().replace('/','')
	
	rp = rospkg.RosPack()
		
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'frame_id': DEFAULT_FRAME_ID,
	  'base_frame_id': DEFAULT_BASE_FRAME_ID,
	  'publish_saved_markers_tf': True,
	  'folder_path': os.path.join(rp.get_path('marker_mapping'), 'config'),
	  'markers_filename': 'markers.yaml',
	  'load_markers_on_init': True,
	  'max_marker_id': MAX_MARKER_ID,	# used to filter the markers detected
	  'min_marker_id': MIN_MARKER_ID	# used to filter the markers detected
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))
			
	
	rc_node = MarkerMapping(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
