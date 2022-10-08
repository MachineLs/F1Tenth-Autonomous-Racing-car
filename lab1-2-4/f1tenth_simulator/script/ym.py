<node pkg="lab1" name="robot_0" type="evader2.py" output="screen" />
 
 <node pkg="lab1" name="robot_1" type="listener.py" />




 (trans,rot) = listener.lookupTransform('/robot_1', '/robot_0', rospy.Time(0))