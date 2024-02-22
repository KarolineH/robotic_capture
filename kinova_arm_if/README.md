# kinova_arm_if
Custom code using the Kortex Python API to interface a Kinova Gen3 robot without gripper.

# WARNING
There are no specific safties or protection zones in place.
When moving the arm with this API, YOU are responsible to check that the specified movements are safe.
Consider your workspace, the robot mounting point, the mounted tool/camera (that includes the robot's own vision module) and make sure there are no collisions. Also consider collisions of the mounted tool/camera/vision module with the robot arm itself. If the mounted tool or camera is tethered by any cables, please identify some safe joint motion ranges to prevent tangling.

# Prerequisites
Install the Kinova Python API
Follow [theses instructions](https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md), download the .whl file and install.\
Install any required Python libraries from the requirements.txt\

We believe that the Kortex API works best with Python<=3.9 

Also please ensure that your robot is set up correctly according to the manufactorer's instructions.\
You can use the Kinova Web Aplication to make sure that an Ethernet connection to the robot has been established successfully before using this interface module.