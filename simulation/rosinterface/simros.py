import rospy
from std_msgs.msg import Float32MultiArray

'''
JointPosition, JointTorque, FootContact:

[arg1][arg2]

arg1 is number of segment (in this setup: 0-2)
arg2 is 
	for joint position and torque: number of motor where
		0 = BC_Li
		1 = CF_Li
		2 = FT_Li
		3 = BC_Ri
		4 = CF_Ri
		5 = FT_Ri
		6 = BBHi
		7 = BBVi
	for foot contact: 0 = left, 1 = right
'''
class simRos:

	def __init__(self):
		self.seg_num = 3
		self.motor_num = 8
		rospy.init_node('ring_nett', anonymous=True)
		self.JointPositionCommandPub=[]
		self.JointPositionSub=[]
		self.JointTorqueSub=[]
		self.FootContactSub=[]
		
		self.JointPosition=[[0]*self.motor_num]*self.seg_num
		self.JointTorque=[[0]*self.motor_num]*self.seg_num
		self.FootContact=[[0]*2]*self.seg_num
		
		for i in range(self.seg_num):
			self.JointPositionCommandPub.append(rospy.Publisher('/seg%d' % i + '/jointPositionCommand', Float32MultiArray, queue_size=10))
			
			self.JointPositionSub.append(rospy.Subscriber('/seg%d' % i + '/jointPosition', Float32MultiArray, self.JointPosition_cb))
			self.JointTorqueSub.append(rospy.Subscriber('/seg%d' % i + '/jointTorque', Float32MultiArray, self.JointTorque_cb))
			self.FootContactSub.append(rospy.Subscriber('/seg%d' % i + '/footContact', Float32MultiArray, self.FootContact_cb))
		
		self.rate = rospy.Rate(30)
		
		rospy.sleep(1)
		for i in range(self.seg_num):
			self.setMotorPosition(i,[0.0]*8)
		rospy.sleep(1)
			
	def setMotorPosition(self,seg,motor_command):
		publishdata = Float32MultiArray()
		motor_command[6] += 3.1413
		motor_command[7] += 1.571 
		publishdata.data = [float(seg)] + motor_command
		self.JointPositionCommandPub[seg].publish(publishdata)
		
	def setLegPosition(self,seg,leg_command):
		motor_command = leg_command[0:6] + [0.0]*2
		self.setMotorPosition(seg,motor_command)
		
	def JointPosition_cb(self,data):
		self.JointPosition[int(data.data[0])] = data.data[1:8]
	
	def JointTorque_cb(self,data):
		self.JointTorque[int(data.data[0])] = data.data[1:8]
		
	def FootContact_cb(self,data):
		self.FootContact[int(data.data[0])] = data.data[1:3]
	
	
		

