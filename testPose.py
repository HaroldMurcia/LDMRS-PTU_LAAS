import rover, time, sys, rospy

# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
# Harold F MURCIA - November 2017

# This file is a test of reading and printing
# the rover (minnie) position. [it needs execution of nodes from minnie for position]

if __name__ == "__main__":
    minnie = rover.ROVER()
    minnie.ini_Rover()
    try:
        rospy.init_node('test_Pose')
	time.sleep(1)
        print "X: "+ str(minnie.x)
        print "Y: "+ str(minnie.y)
        print "Z: "+ str(minnie.z)
	print "Qw: "+ str(minnie.qw)
	print "Qx: "+ str(minnie.qx)
	print "Qy: "+ str(minnie.qy)
	print "Qz: "+ str(minnie.qz)
	print "Ts: "+ str(minnie.ts)
        minnie.stopReading()
        time.sleep(1)
        sys.exit()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"  
