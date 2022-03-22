# DRDO's UAV-Guided UGV Navigation Challenge


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
This package has been tested on Ubuntu 18.04 with ROS Melodic and Gazebo-9, installation instructions for ROS Melodic can be found [here](http://wiki.ros.org/melodic/Installation).

### 1.2. **Libraries** 
[Matplotlib](https://matplotlib.org/), [Numpy](https://numpy.org/), [Scipy](https://scipy.org/), [OpenCV](https://opencv.org/), [CasADi](https://web.casadi.org/get/)
```
    python -m pip install -U matplotlib
    pip install numpy
    python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
    sudo apt install python3-opencv
    pip install casadi
```

## 2. Setting up environment variables

## 3. Build the Repository
Clone the repository and catkin build:
```
    cd ~/catkin_ws/src
    git clone https://github.com/Cypre55/drdo-interiit22
    cd ../
    catkin build
    source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)


## 4. Running World-1: 
```
    cd catkin_ws/src
    cd drdo-interiit22/scripts
    ./launch_world1.sh
    rosservice call /mavros/set_stream_rate 0 20 1
```


## 5. Running World-2: 
```
    cd catkin_ws/src
    cd drdo-interiit22/scripts
    ./launch_world2.sh
````
