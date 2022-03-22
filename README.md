# DRDO's UAV-Guided UGV Navigation Challenge
## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
This package has been tested on Ubuntu 18.04 with ROS Melodic and Gazebo-9. Installation instructions for ROS Melodic can be found [here](http://wiki.ros.org/melodic/Installation).

### 1.2. **Libraries** 
We used the following libraries:  [Matplotlib](https://matplotlib.org/), [Numpy](https://numpy.org/), [Scipy](https://scipy.org/), [OpenCV](https://opencv.org/), [CasADi](https://web.casadi.org/get/)
```
    python -m pip install -U matplotlib
    pip install numpy
    python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
    sudo apt install python3-opencv
    pip install casadi
```
## 2. Setting up Environment Variables & Parameters
### 2.1 Environment Variables
Append the following lines at the end of your .bashrc file
```bash
    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    export PATH=/usr/lib/ccache:$PATH
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/drdo-interiit22/models
```
Set paths for models:
```
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc.
```
### 2.2 Parameters
Camera Parameters: Open the file drdo-interiit22/models/gimbal_small_2d/model.sdf and update following parameters of ```<sensor name="depth_camera" type="depth">``` :
```
    <update_rate>30</update_rate>
    <camera>
      <clip>
        <near>0.1</near>
        <far>70</far>
      </clip>
    </camera>
    <plugin name="depth_controller" filename="libgazebo_ros_openni_kinect.so">
      <updateRate>30.0</updateRate>
      <pointCloudCutoff>0.4</pointCloudCutoff>
      <pointCloudCutoffMax>70.0</pointCloudCutoffMax>
    </plugin>
```

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
    rosservice call /mavros/set_stream_rate 0 20 1
```

- System Architecture: Flow chart representing working of the package

    <img src = "img/Flow_chart_final.png" height = "400" width = "820">
- UGV Localization: Subpackage for detection of UGV using input from UAV

    <img src = "img/detector_rgb.png" height = "300" width = "380" > <img src = "img/detector_final.png" height = "300" width = "380" >
- Planning: Subpackage for exploring the world

    <img src = "img/lane_detection_graph.drawio.png" height = "400" width = "420" >
- Controls Flow Chart: Controls subpackage flow chart

    <img src = "img/controls-flow-chart.png" height = "400" width = "820" >
