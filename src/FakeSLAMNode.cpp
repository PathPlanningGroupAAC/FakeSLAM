#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "zed_msgs/msg/cones.hpp"
#include "zed_msgs/msg/cone.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "csv.h"

#include <filesystem>

using namespace std::chrono_literals;

struct Loc
{

    glm::vec2 pos;
    glm::vec2 dir;

};
class FakeSLAMNode : public rclcpp::Node
{
public:
    FakeSLAMNode() : Node("FakeSLAM"), count_(0)
    {

        loadTrack(m_in, "m_in.csv");
        loadTrack(m_out, "m_out.csv");
        loadLocs(m_loc, "loc.csv");
        loadInd(indices, "frames.csv");

        count_ = indices.size();

        i = 0;
        framep = 0;
        odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/ukf_update_pose", 10);
        zed_ = this->create_publisher<zed_msgs::msg::Cones>("/zed2i/topic_bbox_zed3d", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&FakeSLAMNode::pubblica_messaggio, this));
    }

private:
    void pubblica_messaggio()
    {
        framep = indices[i];
        if(i >= count_)
        {
            RCLCPP_INFO(this->get_logger(), "END");
            timer_->cancel();
            return;
        }
        auto msg = nav_msgs::msg::Odometry();

        msg.pose.pose.position.x = m_loc[i].pos.x;
        msg.pose.pose.position.y = m_loc[i].pos.y;

        msg.pose.pose.orientation.x = m_loc[i].dir.x;
        msg.pose.pose.orientation.y = m_loc[i].dir.y;
        msg.pose.pose.orientation.z = 0;
        msg.pose.pose.orientation.w = 0;

        odom_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "------------------ FRAME[%d] -------------------", i);
        // CONES
        {
            if(framep == indices[indices.size()-1])
            {
                indices.push_back(m_in.size());
            }

            auto msg = zed_msgs::msg::Cones();
            
            for(int j = framep; j < indices[i+1]; j++)
            {
                auto cone = zed_msgs::msg::Cone();
                cone.x = m_in[j].x;
                cone.y = m_in[j].y;
                msg.blue_cones.push_back(cone);
                RCLCPP_INFO(this->get_logger(), "Publishing cone: (%f %f)", msg.blue_cones[msg.blue_cones.size()-1].x, msg.blue_cones[msg.blue_cones.size()-1].y);
            }

            RCLCPP_INFO(this->get_logger(), "###");

            for(int j = framep; j < indices[i+1]; j++)
            {
                auto cone = zed_msgs::msg::Cone();
                cone.x = m_out[j].x;
                cone.y = m_out[j].y;
                msg.yellow_cones.push_back(cone);
                RCLCPP_INFO(this->get_logger(), "Publishing cone: (%f %f)", msg.yellow_cones[msg.yellow_cones.size()-1].x, msg.yellow_cones[msg.yellow_cones.size()-1].y);
            }

            zed_->publish(msg);
        }

        i++;
    }

    // blu in
    // giallo out
    void loadTrack(std::vector<glm::vec2>& points, const std::string& path)
    {
        char separator = std::filesystem::path::preferred_separator;
        std::string absolute = std::filesystem::current_path().string() + separator + path;
        try {
            io::CSVReader<2> in(absolute);

            double x, y;
            while (in.read_row(x, y))
            {
                points.push_back(glm::vec2((float)x, (float)y));
            }
        }
        catch (...)
        {
            std::cout << "File error\n";
        }
    }

    void loadLocs(std::vector<Loc>& points, const std::string& path)
    {
        char separator = std::filesystem::path::preferred_separator;
        std::string absolute = std::filesystem::current_path().string() + separator + path;
        try {
            io::CSVReader<4> in(absolute);

            double x, y, dx, dy;
            while (in.read_row(x, y, dx, dy))
            {
                points.push_back({glm::vec2((float)x, (float)y), glm::vec2((float)dx, (float)dy)});
            }
        }
        catch (...)
        {
            std::cout << "File error\n";
        }
    }

    void loadInd(std::vector<int>& points, const std::string& path)
    {
        char separator = std::filesystem::path::preferred_separator;
        std::string absolute = std::filesystem::current_path().string() + separator + path;
        try {
            io::CSVReader<1> in(absolute);

            int x;
            while (in.read_row(x))
            {
                points.push_back(x);
            }
        }
        catch (...)
        {
            std::cout << "File error\n";
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
    rclcpp::Publisher<zed_msgs::msg::Cones>::SharedPtr zed_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<glm::vec2> m_in;
    std::vector<glm::vec2> m_out;
    std::vector<Loc> m_loc;
    std::vector<int> indices;
    size_t count_;

    int i;
    int framep;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeSLAMNode>());
    rclcpp::shutdown();
    return 0;
}
