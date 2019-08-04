# Kalman-Filter

Here will be presented Sensor fusion for the HiveTracker. It makes fusion between Lighthouse positioning and the IMU.

Nota : Here is the meaning of my abbreviation. LH = Lighthouse ; KF = Kalman filter ; HT = HiveTracker

There is three kind of files :
  - The preprocessing of data from the HiveTracker : this transform accelerations from the IMU and Lighthouses data into two 3D points of the space.
  - The sensor fusion : This is actually Kalman Filter. It estimate the true position of the Tracker using IMU and Lighthouse positioning.
  - The representation on a 3D space : This the Blender file. It directly use position calculated after Sensor fusion.
  
  Libraries used on Python code are all on the Anaconda package : https://www.anaconda.com/download/. Functions used there will be sooner extracted and placed into preprocessing file.
  Serial library can be found here : https://github.com/pyserial/pyserial/
  Find Blender on their website : https://www.blender.org/download/
  
  Find enclosures to the HIVETracker into Frame folder. These are in .stl format, and have to be 3D printed. 4 arms and 1 body frame have to be printed for the current developping enclosure. The tetredron enclosure will allow more visibility for at least one diode, no matter how the tracker is oriented.
  
  A simulation of Lighthouse positioning system is available on the Dev branch. Called LH_Simu.blend, you can open it and find a Python script inside. The simulation is a mini-game. Move the blue car with Z, Q, S, D key (it's from a french Azerty keyboard), and the scanning/calculated position will appear with a green dot.
  There is also an other way of calculating position from the Lighthouses. It use the asumption that Diodes are on solid. This Python code allows to know position and orientation of Lighthouses but isn't accurate enough for the moment. A vizaulisation of the calculated points is also an the folder.

