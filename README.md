# Kalman-Filter

Here will be presented Kalman Filter for IMU+HiveTracker.

There is three kind of files :
  - Simulations and Vizualization : which are Blender app. You kind open them and find a Python script inside. The simulation is a mini-game of lighthouse positionning system. Move the blue car car with Z, Q, S, D key (sorry it's for a french Azerty keyboard), and the scanning/calculated position will appear with a green dot.
  - The Kalman Filter : is C++ algorithm. It estimate the true position of the cube using IMU simulation and HiveTracker.
  - The localization : this code allows to find find position of the HIVE Tracker. It use only one lighthouse and assume diodes are on a solid.
  
  Libraries used on Python code are all on Anaconda package : https://www.anaconda.com/download/
