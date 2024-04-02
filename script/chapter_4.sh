#!/bin/bash
cat <<- EOF > turtle_teleop.sh
#!/bin/bash
#安装tf2相应的功能包
#通过一个launch文件启动
#控制其中的一只小海龟，另外一只小海龟会自动跟随运动
sudo apt install ros-humble-turtle-tf2-py ros-humble-tf2-tools -y && \
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py && \
ros2 run turtlesim turtle_teleop_key && \
rm $0
EOF
chmod +x turtle_teleop.sh && gnome-terminal -t "安装tf2" -e bash -c "./turtle_teleop.sh;exec bash"

#查看TF树
gnome-terminal -t "查看TF树" -e "bash -c 'ros2 run tf2_tools view_frames; exec bash'"
# gnome-terminal -- bash -c "ros2 run tf2_tools view_frames;exec bash"

#查询坐标变换信息
gnome-terminal -t "查询坐标变换信息" -e bash -c "ros2 run tf2_ros tf2_echo turtle2 turtle1;exec bash"

#坐标系可视化
gnome-terminal -t "坐标系可视化" -e bash -c "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz;exec bash"


cat <<- EOF > install_Gazebo.sh
#!/bin/bash
#安装Gazebo仿真
sudo sh -c 'echo "deb  http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
sudo apt-get update && \
sudo apt install ros-humble-gazebo-* -y && \
rm $0
EOF
chmod +x install_Gazebo.sh && gnome-terminal -t "安装Gazebo仿真" -- bash -c "./install_Gazebo.sh;exec bash"
















