# pr2_amazon_challenge_sim #

This package brings up a simulated PR2 in gazebo with the Kiva shelf for the amazon challenge
![pr2_gazebo_kiva.png](https://bitbucket.org/repo/GEndLK/images/1160564541-pr2_gazebo_kiva.png)

### Requirements ###

* Make sure you have **pr2_gazebo** and **gazebo** installed :


```
#!python

sudo apt-get install ros-hydro-pr2-* gazebo ros-hydro-gazebo*
```


### Installation ###

* Clone the repository to your catkin workspace
  
```
#!python

git clone https://bitbucket.org/cvapgoestoseattle/pr2_amazon_challenge_sim
```

* Compile the workspace

```
#!python

cd ~/amazon_challenge_ws
catkin_make
```

### Launching the simulation ###

To simulate a PR2 and empty Kiva shelf in Gazebo:

```
#!python

roslaunch pr2_amazon_challenge_sim pr2_empty_shelf.launch
```