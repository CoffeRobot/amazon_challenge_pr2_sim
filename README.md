# pr2_kiva_shelf_sim #

This package brings up a simulated PR2 in gazebo with the Kiva shelf for the amazon challenge
![pr2_gazebo_kiva.png](https://bitbucket.org/repo/GEndLK/images/1160564541-pr2_gazebo_kiva.png)

### Requirements ###

* Make sure you have **pr2_gazebo** and **gazebo** installed 

### Installation ###

* Clone the repository to your catkin workspace
  
```
#!python

git clone https://bitbucket.org/cvapgoestoseattle/pr2_kiva_shelf_sim
```

* Compile the workspace

```
#!python

cd ~/amazon_challenge_ws
catkin_make
```

### Launching the simulation ###


```
#!python

roslaunch pr2_kiva_shelf_sim pr2_sim_bringup.launch
```