# README

## In world_paths folder

### Record World Path(in gazebo frame):
1. Launch the desired world
2. Change the name of the file, where the path is changed, in take_odom2.py
3. run the control2 executable in your_catkin_ws/devel/lib/drdo_interiit22/. This opens a window named control2. While driving, focus on that window. No need to press enter after every keystroke lmao
4. run python script take_odom2.py .
5. drive car
6. after the path has been covered, close the running take_odom2.py script to save the file in the worlds_path dir.

### Transform (to local frame, dynamically)
1. launch world
2. run ardupilot sim_vehicle script
3. change input and output path names in transform.py
4. run transform.py
5. the script automatically saves and exits

### Transform (to local frame, given Tgl)
1. launch world
3. change input and output path names in transform.py
3. add the tgl matrix to the tgl variable in transform.py
4. run transform.py
5. the script automatically saves and exits

### visualize path
1. edit the cube.urdf file to change properties of the model that is spawned along the path (color, size, etc)
2. in spawn_path.py, add name of path_file you want to plot
3. change model_name in spawn_path.py if you want to plot multiple paths in the same gazebo iteration. also change cube.urdf to change appearance
4. launch world, pause the simulation in gazebo
5. run spawn.py


