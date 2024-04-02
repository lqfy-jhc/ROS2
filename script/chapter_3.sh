#!/bin/bash
#安装colcon
gnome-terminal -- bash -c "sudo apt-get update && sudo apt-get install python3-colcon-common-extensions -y;exec bash"

#创建节点并进入节点
cd ~ && mkdir -p ros2/mytest_ws/src && cd ~/ros2/mytest_ws/src

#
ros2 pkg create subscribe_and_publish --build-type ament_cmake --dependencies rclcpp

#创建publisher.cpp&subscribe1.cpp
touch subscribe_and_publish/src/publisher.cpp && touch subscribe_and_publish/src/subscribe1.cpp

#写入文本
cd ~/ros2/mytest_ws/src/subscribe_and_publish/src

cat > publisher.cpp << EOF
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class Publisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Publisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者
        subscribe_and_publish_publisher_ = this->create_publisher<std_msgs::msg::String>("subscribe_and_publish", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "1234";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        subscribe_and_publish_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr subscribe_and_publish_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个的节点*/
    auto node = std::make_shared<Publisher>("publisher");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
EOF

cat > subscribe1.cpp << EOF
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subscribe : public rclcpp::Node
{
public:
    Subscribe(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
          // 创建一个订阅者订阅话题
         subscribe_and_publish_subscribe_ = this->create_subscription<std_msgs::msg::String>("subscribe_and_publish", 10, std::bind(&Subscribe::command_callback, this, std::placeholders::_1));
    }

private:
     // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe_and_publish_subscribe_;
     // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // double speed = 0.0f;
        // if(msg->data == "1234")
        // {
        //     speed = 0.2f;
        // }
        // RCLCPP_INFO(this->get_logger(), "收到[%s]指令，发送速度 %f", msg->data.c_str(),speed);
        RCLCPP_INFO(this->get_logger(), "收到[%s]指令", msg->data.c_str());
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个的节点*/
    auto node = std::make_shared<Subscribe>("subscribe1");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
EOF

#修改CmakeLists.txt
cd ~/ros2/mytest_ws/src/subscribe_and_publish && \
sed -i '/"find_package(rclcpp REQUIRED)"/{N;s/\n/"find_package(std_msgs REQUIRED)"\n/}' CmakeLists.txt 

#在CmakeLists.txt最后面添加下面内容
cat >> CmakeLists.txt << EOF
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)
add_executable(subscribe1 src/subscribe1.cpp)
ament_target_dependencies(subscribe1 rclcpp std_msgs)

install(TARGETS
  publisher
  subscribe1
  DESTINATION lib/${PROJECT_NAME}
)
EOF

#打开package.xml文件，在中间添加
sed -i '/"<depend>rclcpp</depend>"/{N;s/\n/"<depend>std_msgs</depend>"\n/}' package.xml

#3.1重开一个终端，启动订阅节点
cat << EOF > subscribe.sh
#!/bin/bash
cd ~/ros2/mytest_ws && colcon build && source install/setup.bash
ros2 run subscribe_and_publish subscribe1
EOF
chmod +x subscribe.sh && gnome-terminal -t "启动订阅节点" -e bash -c "./subscribe.sh;exec bash"

#再开一个终端，启动发布节点
cat << EOF > publisher.sh
#!/bin/bash
cd ~/ros2/mytest_ws && source install/setup.bash
ros2 run subscribe_and_publish publisher
EOF
chmod +x publisher.sh && gnome-terminal -t "启动发布节点" -e bash -c "./publisher.sh;exec bash"

