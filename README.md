# Kalman-Filter

Here will be presented Sensor fusion for the HiveTracker. It makes fusion between Lighthouse positioning and the IMU.

There is three kind of files :
  - The preprocessing of data from the HiveTracker : this transform accelerations from the IMU and Lighthouses data into two 3D points of the space.
  - The sensor fusion : This is actually Kalman Filter. It estimate the true position of the Tracker using IMU and Lighthouse positioning.
  - The representation on a 3D space : This use directly the preprocessing IMU and Lighthouse calculated positions.
  
  Libraries used on Python code are all on the Anaconda package : https://www.anaconda.com/download/. Functions used there will be sooner extracted and placed into preprocessing file.
  
  A simulation of Lighthouse positioning system is available on the Dev branch. Called LH_Simu.blend, you can open it and find a Python script inside. The simulation is a mini-game. Move the blue car with Z, Q, S, D key (it's from a french Azerty keyboard), and the scanning/calculated position will appear with a green dot.
  There is also an other way of calculating position from the Lighthouses. It use the asumption that Diodes are on solid. This Python code allows to know position and orientation of Lighthouses but isn't accurate enough for the moment. A vizaulisation of the calculated points is also an the folder.

