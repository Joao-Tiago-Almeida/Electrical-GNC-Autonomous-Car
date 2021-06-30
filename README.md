# Autonomous-Cars

The real-time simulation of an Autonomous Car performed, is formed by 3 main components: **_G_** uidance, **_N_** avigation and **_C_** ontrol. The software has a GUI where many real-life events can be drawn (e.g. speed limit zones or random pedestrian zones) and a route based on checkpoints which are used in the dynamical Dijkstra bearing in mind that the direct cost of moving to a specific position differs based on the previous one. Finally the smoothing technique bspline in transitions is performed in order to make a continuous path more polished with a fixed sample rate. In addition the sensor fusion used in the navigation turns all simulation more real by making the autonomous vehicle aware of traffic lights, signs or potential dangers caused by pedestrians or localisation errors, that are simulated with gaussian noise in the EKF predictions based on the GPS information and the car dynamic model. It is also considered possible GPS anomalies such as breakup zones or a kidnapped situation. The available energy of the car is crucial for the velocity planning, during the path-following based on a virtual robot (avatar). The controller also takes into account the future route in order to smooth the linear and angular velocity transitions of the vehicle, which also simulates brakes for different road conditions (e.g. normal and wet tar).


Report: [file](./Electrical_GNC_Autonomous_Car.pdf)

Software Grade: 18/20\
Reports  Grade: 15.8/20
