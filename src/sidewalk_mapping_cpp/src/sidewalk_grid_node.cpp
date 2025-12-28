#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <filters/filter_chain.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace grid_map;

class SidewalkNavNode : public rclcpp::Node {
public:
    SidewalkNavNode() : Node("sidewalk_grid_node"), filter_chain_("grid_map::GridMap") {
        
        this->declare_parameter("grid_size", 10.0);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("map_frame", "base_link");
        this->declare_parameter("filter_yaml", "filters.yaml");

        map_.setFrameId(this->get_parameter("map_frame").as_string());
        map_.setGeometry(Length(this->get_parameter("grid_size").as_double(), 
                               this->get_parameter("grid_size").as_double()), 
                        this->get_parameter("resolution").as_double());
        
        map_.add("elevation");
        map_.add("semantics");
        map_.add("color"); 
        map_.add("nav2_occupancy"); 

        // Configure Filter Chain
// Inside your Constructor:
        std::string yaml_file = this->get_parameter("filter_yaml").as_string();
        std::string pkg_share = ament_index_cpp::get_package_share_directory("sidewalk_mapping_cpp");
        std::string yaml_path = pkg_share + "/config/" + yaml_file;

        //this->declare_parameter("grid_map_filters.filter_chain", rclcpp::PARAMETER_NOT_SET);
        // Configure Filter Chain
        RCLCPP_INFO(this->get_logger(), "Parameters available at startup:");
        auto params = this->list_parameters({}, 100);
        for (auto &p : params.names) {
            RCLCPP_INFO(this->get_logger(), "  %s", p.c_str());
        }

        bool ok = filter_chain_.configure("grid_map_filters", 
                                        this->get_node_logging_interface(),
                                        this->get_node_parameters_interface());
        RCLCPP_INFO(this->get_logger(), "Filter chain configure returned: %s", ok ? "true" : "false");

        if (!ok) {
            RCLCPP_ERROR(this->get_logger(), "Filter Chain: Failed to find 'grid_map_filters' in parameters!");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/da3/points", sensor_qos,
            std::bind(&SidewalkNavNode::cloudCallback, this, std::placeholders::_1));

        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/gridmap/semantic_map", 10);
        nav_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap/published_costmap", 10);
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

        map_.setPosition(Position(4.0, 0.0));
        map_["elevation"].setConstant(NAN);
        map_["semantics"].setConstant(NAN);
        map_["nav2_occupancy"].setConstant(-1); 

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(transformed_cloud, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(transformed_cloud, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(transformed_cloud, "b");

        bool points_added = false;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            if (*iter_z < -1.5 || *iter_z > 1.5) continue;

            Position position(*iter_x, *iter_y);
            if (!map_.isInside(position)) continue;

            int class_id = rgbToClass(*iter_r, *iter_g, *iter_b);
            if (class_id == 255) continue;

            float cost = -1; 
            if (class_id == 1)      cost = 0;   
            else if (class_id == 0) cost = 30;  
            else if (class_id == 9) cost = 70;  
            else cost = 100;                    

            map_.atPosition("elevation", position) = *iter_z;
            map_.atPosition("semantics", position) = (float)class_id;
            
            float& current_nav_cost = map_.atPosition("nav2_occupancy", position);
            if (cost > current_nav_cost) current_nav_cost = cost;
            points_added = true;
        }

        //debug code added
        auto layers = map_.getLayers();
        std::string layers_str;
        for (const auto &l : layers) {
            layers_str += l + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Before update, layers: %s", layers_str.c_str());
        layers_str.clear();
        for (const auto &l : layers) {
            layers_str += l + " ";
        }
        RCLCPP_INFO(this->get_logger(), "After update, layers: %s", layers_str.c_str());

        if (!points_added) return;

        // --- FILTER CHAIN UPDATE ---
        if (!filter_chain_.update(map_, map_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Filter update failed.");
        }
        // After: if (!filter_chain_.update(map_, map_)) { ... }

        // Add this:
        // === LAYER STATISTICS (finite values only) ===
        RCLCPP_INFO(this->get_logger(), "=== LAYER STATISTICS (finite values only) ===");
        std::vector<std::string> layers_to_check = {
            "elevation", "elevation_inpainted", "elevation_smooth",
            "normal_x", "normal_y", "normal_z",
            "slope", "roughness", "edges", "nav2_occupancy", "semantics"
        };

        for (const auto& layer : layers_to_check) {
            if (map_.exists(layer)) {
                float min_val = map_[layer].minCoeffOfFinites();
                float max_val = map_[layer].maxCoeffOfFinites();
                float mean_val = map_[layer].meanOfFinites();
                int finite_cells = map_[layer].count();

                RCLCPP_INFO(this->get_logger(), "%-20s min=%.4f  max=%.4f  mean=%.4f  cells=%d",
                            layer.c_str(), min_val, max_val, mean_val, finite_cells);
            } else {
                RCLCPP_INFO(this->get_logger(), "%-20s LAYER NOT FOUND", layer.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "================================================");


        //added part of the codede 
        // --- PUBLISH GRID MAP ---
        // Correct ROS 2 Syntax: toMessage returns a unique_ptr
        auto output_grid_msg = GridMapRosConverter::toMessage(map_);
        grid_map_pub_->publish(std::move(output_grid_msg));

        // --- PUBLISH OCCUPANCY GRID ---
        auto occupancy_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        GridMapRosConverter::toOccupancyGrid(map_, "nav2_occupancy", -1.0, 100.0, *occupancy_msg);
        nav_grid_pub_->publish(std::move(occupancy_msg));
    }

    GridMap map_;
    filters::FilterChain<grid_map::GridMap> filter_chain_;
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