# iphone-attitude
Get iPhone position in space, animate on Mac with SceneKit.

# Purpose
Shows how sensor fusion built into iOS (CMMotionManager) is used to 
determine the iPhone attitude in space as a quaternion. Also 
demonstrates how to use SceneKit to build a simple model of an iPhone and 
animate it in 3D based on quaternions. Socket based TCP-IP communication
(CFStream) is used to pass data from the phone to the Mac.

# Details
There are two applications:

1. attitude

  An iphone app to get the iPhone's orientation in space ('attitude').
  The attitude is displayed on screen as a quaternion.
  You can enter an IP-address and port number of a server to send the quaternions to.

2. model3d

  A Mac app that will display an iPhone in 3D and move it around
  according to the quaternion info it receives on port 2718.



