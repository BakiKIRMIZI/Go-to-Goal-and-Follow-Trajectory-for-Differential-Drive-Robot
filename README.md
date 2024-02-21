# Go to Goal and Follow Trajectory for Differential Drive Mobile Robot

In this project, a mobile robot with the differential drive feature is seen in the following figure. The wheel diameter is R=0.1m and the base L=0.3m are given.

The motion control of the robot is performed through the angular velocities of the right and left wheels (𝜔𝑟 and 𝜔𝑙).

<img src="https://github.com/BakiKIRMIZI/Go-to-Goal-and-Follow-Trajectory-for-Differential-Drive-Robot/assets/64962776/065544f1-8ae9-4fe1-a3a3-47593e7d617b" width=20% height=20%>

---

This project will use the obtained differential equations for a robot with a differential drive feature.

1. Implement the Go-to-Goal controller in Python or C/C++. Take the initial pose [x0,y0] = [0,0] and the goal pose [xg,yg] = [10,20].

2. Implement the Follow-Trajectory controller in Python or C/C++. Take the initial pose [x0,y0, 𝜃0] = [0,0,0], the trajectory to be followed [x(t), y(t)] = [t, cos(t)], and the following distance d*=1m.

3. Implement the Go-to-Goal and Follow-Trajectory controllers in Simulink.
