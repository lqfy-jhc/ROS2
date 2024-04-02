# 脚本列表

## 一键启动

## 一键安装脚本

[ros2_tool.sh](https://github.com/lqfy-jhc/ROS2/blob/main/script/install_tool.sh)

- 方法 1

```sh
curl -sSL https://raw.githubusercontent.com/lqfy-jhc/ROS2/main/script/install_tool.sh | bash
```

- 方法 2

```sh
curl -L https://github.com/lqfy-jhc/ROS2/blob/main/script/install_tool.sh -o ~/install_tool.sh && chmod +x ~/install_tool.sh && gnome-terminal -t "运行脚本" -e bash -c "./install_tool.sh;exec bash"
```

- 方法 3

```sh
curl -L https://raw.githubusercontent.com/lqfy-jhc/ROS2/main/script/install_tool.sh -o ~/install_tool.sh && chmod +x ~/install_tool.sh && gnome-terminal -t "运行脚本" -e bash -c "./install_tool.sh;exec bash"
```
