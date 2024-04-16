#include <chrono>
#include <memory>
#include <random>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial_visualizer/msg/feedback.hpp"
#include "plotjuggler_msgs/msg/dictionary.hpp"
#include "plotjuggler_msgs/msg/data_points.hpp"
#include "math.h"

using namespace std::chrono_literals; // 500msとか書けるようにするため

class DammyTalkerNode : public rclcpp::Node
{
public:
    DammyTalkerNode() : Node("dammy_talker_node")
    { // rclcpp::Nodeを継承してpublisher_nodeというNodeを作成

        // publisherの作成：第1引数の文字列はトピック名、第2引数の数値はQoS
        //<std_msgs::msg::String> でトピックのデータ型を指定
        dict_pub = this->create_publisher<plotjuggler_msgs::msg::Dictionary>("plt_dict", 10);
        data_pub = this->create_publisher<plotjuggler_msgs::msg::DataPoints>("plt_data", 10);

        // messageをpublish(送信)するcallback関数
        auto publish_msg_callback = [this]() -> void { // this->publisher_->publish(message); // publishする
            dict_publisher();

            plotjuggler_msgs::msg::DataPoints msg;

            msg.dictionary_uuid = UUID;
            msg.target = CreateDataPoint(0, t, 0);
            msg.cur_val = CreateDataPoint(0, t, sin(t));
            msg.ctrl_val = CreateDataPoint(0, t, cos(t));

            data_pub->publish(msg);

            t += 0.01;
        };
        timer_ = this->create_wall_timer(10ms, publish_msg_callback); // 500msに一度callbackが呼ばれる
    }

private:
    // 上記の動作に必要なprivateメンバ
    // publicで作成したメンバについて書く
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<plotjuggler_msgs::msg::Dictionary>::SharedPtr dict_pub;
    rclcpp::Publisher<plotjuggler_msgs::msg::DataPoints>::SharedPtr data_pub;

    const uint32_t UUID = 1;
    double t = 0;
    int dict_frag = 0;

    plotjuggler_msgs::msg::Dictionary dict;

    void dict_publisher()
    {
        if (dict_frag == 0)
        {
            dict.dictionary_uuid = UUID;
            dict.names.push_back("sensor_a"); // index 0
            dict.names.push_back("sensor_b"); // index 1
            dict.names.push_back("sensor_c"); // index 2

            dict_frag = 1;
        }

        dict_pub->publish(dict);
    }

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
    rclcpp::spin(std::make_shared<DammyTalkerNode>()); // クラスを実体化し、ノードを作る
    rclcpp::shutdown();
    return 0;
}
