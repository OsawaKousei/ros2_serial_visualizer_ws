#include <functional>
#include <memory>
#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "plotjuggler_msgs/msg/dictionary.hpp"
#include "plotjuggler_msgs/msg/data_points.hpp"

using namespace std::chrono_literals;

class PubsubNode : public rclcpp::Node
{
public:
    PubsubNode() : Node("pubsub_node")
    {

        data_pub = this->create_publisher<plotjuggler_msgs::msg::DataPoints>("plt_data", 10);

        auto callback = [this](const std_msgs::msg::UInt8MultiArray &msg) -> void
        {
            for (int i = 0; i < msg.data.size(); i++)
            {
                if (msg.data[i] == 'a') // 文字列の最後にはNULL文字が入っているのでそれを検出する
                {
                    double value = atof(buff);
                    printf("%s\n", buff);

                    data.dictionary_uuid = 1;

                    data.ctrl_val = CreateDataPoint(0, t, value);
                }
                else if (msg.data[i] == 'b')
                {
                    double value = atof(buff);
                    printf("%s\n", buff);

                    data.dictionary_uuid = 1;

                    data.cur_val = CreateDataPoint(1, t, value);
                }
                else if (msg.data[i] == 'c')
                {
                    double value = atof(buff);
                    printf("%s\n", buff);

                    data.dictionary_uuid = 1;

                    data.target = CreateDataPoint(2, t, value);
                }

                buff[buff_index] = msg.data[i];
                buff_index++;

                if (msg.data[i] == 0)
                {
                    buff[buff_index] = msg.data[i];

                    buff_index = 0; // バッファをリセット
                    // printf("%s\n", buff);
                    data_pub->publish(data);
                }
            }
        };

        serilal_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>("serial_read", 10, callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serilal_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<plotjuggler_msgs::msg::DataPoints>::SharedPtr data_pub;

    plotjuggler_msgs::msg::DataPoints data;

    char buff[256];
    int buff_index = 0;
    int value_index = 0;

    double t = 0;

    plotjuggler_msgs::msg::DataPoint CreateDataPoint(uint16_t name_index, double time, double value)
    {
        plotjuggler_msgs::msg::DataPoint point;
        point.stamp = time;
        point.name_index = name_index;
        point.value = value;
        return point;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubsubNode>());
    rclcpp::shutdown();
    return 0;
}

// ros2 run plotjuggler plotjuggler
// sudo chmod 777 /dev/ttyACM0
// ros2 launch serial_driver serial_driver_bridge_node.launch.py
