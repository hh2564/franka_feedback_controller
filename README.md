# Franka Effort Controller
# Package Description
This is a ROS PD feedback controller package to control Franka Emika Panda arm in Gazebo simulation environment.

# Getting Started 
This package was tested in simulation environment Gazebo with: 
* Ubuntu 20.04.4 LTS
* [Ros Noetic](http://wiki.ros.org/noetic)
* [Libfranka](https://frankaemika.github.io/docs/libfranka.html) 0.9.0
* [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) 0.9.0 


You can go to the their official website to find more instruction on installing these packages. 

# PD Feedback Controller 
For feedback controller approach, we implemented a PD controller: desired joint velocity is proportional to joint position error $\theta_e=\theta_d-\theta$ and joint velocity error $\dot\theta_e=\dot\theta_d-\dot\theta$, where $\theta_d$ and $\theta$ are the desired and measured joint position, and $\dot\theta_d$ and $\dot\theta$ are the desired and measured joint velocity respectively. 

We can obtain the desired joint toque $\tau_{feedback}$:


$$\begin{align}
\tau_{feedback} = K_p\theta_e+K_d\dot\theta_e+G(\theta)
\end{align}$$

Whereas $K_p$ and $K_d$ are the appropriate p-gain and d-gain, and $G(\theta)$ is gravity compensation. 
