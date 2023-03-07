from rosinterface.simros import simRos
import rospy


sim_Ros = simRos()


if __name__ == '__main__':
	try:	
		while not rospy.is_shutdown():
			
			#setLegPosition(i,[BC_Li, CF_Li, FT_Li, BC_Ri, CF_Ri, FT_Ri]) where i is the number of segment and value in array is joint angle (in radian)
			sim_Ros.setLegPosition(0,[0, 0, 0, 0, 0, 0])
			
			#setMotorPosition(i,[BC_Li, CF_Li, FT_Li, BC_Ri, CF_Ri, FT_Ri, BBHi, BBVi]) where i is the number of segment and value in array is joint angle (in radian)			
			#sim_Ros.setMotorPosition(1,[0, 0, 0, 0, 0, 0, 0, 0])
			
			print(sim_Ros.JointPosition[0][1])
	
	except rospy.ROSInterruptException:
		pass
