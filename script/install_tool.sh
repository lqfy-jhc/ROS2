#!/bin/bash
cd ~
##### #### ### ## #章节1# ### #### #####
# 安装依赖工具
echo "安装依赖工具"
sudo apt update && sudo apt install curl gnupg lsb-release -y

# 安装ros2
cat <<-EOF >install_ros2.sh
#!/bin/bash
# 添加源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
# 安装ros2
sudo apt update && sudo apt upgrade -y &&sudo apt install ros-humble-desktop -y
# 设置环境变量
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >>~/.bashrc
rm $0
EOF
chmod +x install_ros2.sh

# 检验安装是否成功
# 怎么持续检查文件a.sh是否运行成功，如果运行成功就运行b.sh，如果失败就继续检测，直到检测到b.sh运行成功为止。
# 这个脚本会持续检测a.txt中是否有特定文本内容abcdef，有就运行a.sh没有就继续检测直到找到并运行a.sh
cat <<EOF >check_ros2.sh
#!/bin/bash
while true; do
  if grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    gnome-terminal -t "检验方法1" -- bash -c "ros2 run turtlesim turtlesim_node && ros2 run turtlesim turtle_teleop_key;exec bash"
    gnome-terminal -t "检验方法2" -- bash -c "ros2 run demo_nodes_cpp talker && ros2 run demo_nodes_py listener;exec bash"
    # bash b.sh
    break
  fi
  sleep 1
done
rm $0
EOF
chmod +x check_ros2.sh

# 安装vscode
cat <<-EOF >install_vscode.sh
#!/bin/bash
curl -L https://vscode.download.prss.microsoft.com/dbazure/download/stable/019f4d1419fbc8219a181fab7892ebccf7ee29a2/code_1.87.0-1709078641_amd64.deb -o ~/vscode.deb
# 自动处理依赖问题
sudo dpkg -i ~/vscode.deb && sudo apt update && sudo apt-get install -f
rm $0
EOF
chmod +x install_vscode.sh
# err=$(sudo dpkg -i ~/vscode.deb 2>&1)
# if echo "$err" | grep -q 'Unmet dependencies'; then
#     sudo apt update && sudo apt-get install -f
# fi

##### #### ### ## #章节2# ### #### #####
# 安装colcon
cat <<-EOF >install_colcon.sh
#!/bin/bash
#安装colcon
sudo apt-get update && sudo apt-get install python3-colcon-common-extensions -y && \
rm $0
EOF
chmod +x install_colcon.sh

##### #### ### ## #章节4# ### #### #####
# 安装Gazebo仿真
cat <<-EOF >install_Gazebo.sh
#!/bin/bash
sudo sh -c 'echo "deb  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
sudo apt-get update && \
sudo apt install ros-humble-gazebo-* -y && \
rm $0
EOF
chmod +x install_Gazebo.sh

# 安装tf2相应的功能包
cat <<-EOF >install_tf2.sh
#!/bin/bash
sudo apt install ros-humble-turtle-tf2-py ros-humble-tf2-tools -y && \
rm rm $0
EOF
chmod +x install_tf2.sh

#---------------------------------------------------------------------#
echo "可安装列表如下:"
echo "1.安装ros2"
echo "2.安装vscode"
echo "3.安装colcon"
echo "4.安装Gazebo仿真"
echo "5.安装tf2"
function install_tool() {
    # 读取键盘输入，如果输入1则安装Gazebo仿真,如果输入2则安装vscode,如果输入3则安装ros2,如果直接回车，则安装全部，如果输入q则退出不安装
    read -p "请输入要安装的选项(安装全部:a，退出：q):" choice
    case "$choice" in
    1)
        gnome-terminal -t "安装ros2" -- bash -c "./install_ros2.sh;exec bash"
        gnome-terminal -t "检验是否安装成功" -- bash -c "./check_ros2.sh;exec bash"
        ;;
    2)
        gnome-terminal -t "安装vscode" -- bash -c "./install_vscode.sh;exec bash"
        ;;
    3)
        gnome-terminal -t "安装colcon" -- bash -c "./install_colcon.sh;exec bash"
        ;;
    4)
        gnome-terminal -t "安装Gazebo仿真" -- bash -c "./install_Gazebo.sh;exec bash"
        ;;
    5)
        gnome-terminal -t "安装tf2" -- bash -c "./install_tf2.sh;exec bash"
        ;;
    a | A)
        gnome-terminal -t "安装ros2" -- bash -c "./install_ros2.sh;exec bash"
        gnome-terminal -t "检验是否安装成功" -- bash -c "./check_ros2.sh;exec bash"
        gnome-terminal -t "安装vscode" -- bash -c "./install_vscode.sh;exec bash"
        gnome-terminal -t "安装colcon" -- bash -c "./install_colcon.sh;exec bash"
        gnome-terminal -t "安装Gazebo仿真" -- bash -c "./install_Gazebo.sh;exec bash"
        gnome-terminal -t "安装tf2" -- bash -c "./install_tf2.sh;exec bash"
        ;;
    q | Q)
        echo "退出安装"
        if [[ -e "~/install_tool.sh" ]];then
            mv ~/install_tool.sh ~/ros2_install_tool.sh
        fi
        echo "正在删除安装文件"
        rm ~/install_*.sh
        exit 0
        ;;
    *)
        echo "输入错误"
        if [[ -e "~/install_tool.sh" ]];then
            mv ~/install_tool.sh ~/ros2_install_tool.sh
        fi
        echo "正在删除安装文件"
        rm ~/install_*.sh
        exit 0
        ;;
    esac
}

install_tool
