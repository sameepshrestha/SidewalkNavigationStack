#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
using namespace grid_map;

class SidewalkNavNode : public rclcpp::Node {
public:
    SidewalkNavNode() : Node("sidewalk_nav_node") {
        this->declare_parameter("grid_size", 10.0);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("map_frame", "base_link");

        map_.setFrameId(this->get_parameter("map_frame").as_string());
        map_.setGeometry(Length(this->get_parameter("grid_size").as_double(), 
                               this->get_parameter("grid_size").as_double()), 
                        this->get_parameter("resolution").as_double());
        
        map_.add("elevation");
        map_.add("semantics");
        map_.add("color"); 
        map_.add("nav2_occupancy"); // This will be sent to Nav2

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/da3/points", sensor_qos,
            std::bind(&SidewalkNavNode::cloudCallback, this, std::placeholders::_1));

        // Output 1: Full GridMap (for RViz debugging)
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/gridmap/semantic_map", 10);
        
        // Output 2: Occupancy Grid (Direct feed for Nav2)
        nav_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap/published_costmap", 10);
        
        RCLCPP_INFO(this->get_logger(), "Nav2 Semantic Node Started.");
    }

private:
    int rgbToClass(uint8_t r, uint8_t g, uint8_t b) {
        if (r < 50 && g < 50 && b > 200)   return 0;  // Road
        if (r < 50 && g > 200 && b < 50)   return 1;  // Sidewalk
        if (r > 200 && g < 50 && b < 50)   return 2;  // Building
        if (r > 100 && r < 150 && g > 100 && g < 150 && b > 100 && b < 150) return 3; // Wall
        if (r < 50 && g > 200 && b > 200)  return 5;  // Pole
        if (r < 50 && g > 100 && b < 50)   return 8;  // Vegetation
        if (r > 100 && g > 50 && b < 50)   return 9;  // Terrain
        if (r > 200 && g < 50 && b > 100)  return 11; // Person
        if (r < 50 && g < 100 && b > 100)  return 13; // Car
        return 255; 
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            auto tf = tf_buffer_->lookupTransform(map_.getFrameId(), msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, transformed_cloud, tf);
        } catch (tf2::TransformException &ex) { return; }

        // Reset layers for new robot-centric frame
        map_.setPosition(Position(4.0, 0.0));
        map_["elevation"].setConstant(NAN);
        map_["semantics"].setConstant(NAN);
        map_["nav2_occupancy"].setConstant(-1); // -1 is "Unknown" in Nav2

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(transformed_cloud, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(transformed_cloud, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(transformed_cloud, "b");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            if (*iter_z < -1.5 || *iter_z > 1.5) continue;

            Position position(*iter_x, *iter_y);
            if (!map_.isInside(position)) continue;

            int class_id = rgbToClass(*iter_r, *iter_g, *iter_b);
            if (class_id == 255) continue;

            // --- NAV2 COST MAPPING ---
            float cost = -1; 
            if (class_id == 1)      cost = 0;   // Sidewalk is perfectly FREE
            else if (class_id == 0) cost = 30;  // Road is driveable but not preferred
            else if (class_id == 9) cost = 70;  // Terrain is driveable but bumpy
            else cost = 100;                    // Everything else (Building, Pole, Person) is LETHAL

            // --- ELEVATION CHECK (Physical Safety) ---
            // If the ground is physically too high (>15cm), override semantic logic and make it lethal
            //if (*iter_z > 0.15) cost = 100;

            // Update Map layers
            map_.atPosition("elevation", position) = *iter_z;
            map_.atPosition("semantics", position) = (float)class_id;
            
            // Keep the "worst case" cost for this cell
            float& current_nav_cost = map_.atPosition("nav2_occupancy", position);
            if (cost > current_nav_cost) current_nav_cost = cost;
        }

        // Publish the GridMap for RViz
        std::vector<std::string> layers = {"color", "semantics", "elevation", "nav2_occupancy"};
        auto grid_msg = grid_map::GridMapRosConverter::toMessage(map_, layers);
        grid_map_pub_->publish(*grid_msg);


        // Publish the OccupancyGrid for Nav2
        auto occupancy_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        GridMapRosConverter::toOccupancyGrid(map_, "nav2_occupancy", -1.0, 100.0, *occupancy_msg);
        nav_grid_pub_->publish(std::move(occupancy_msg));
    }

    GridMap map_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_grid_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SidewalkNavNode>());
    rclcpp::shutdown();
    return 0;
}