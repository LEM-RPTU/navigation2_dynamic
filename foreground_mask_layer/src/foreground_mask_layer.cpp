#include "foreground_mask_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace foreground_mask
{

    void ForegroundMaskLayer::onInitialize()
    {
        ObstacleLayer::onInitialize();
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node. Ensure node_ is initialized properly."};
        }

        // Declare plugin parameters
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("map_topic", rclcpp::ParameterValue(std::string("/map")));
        declareParameter("inflation_radius", rclcpp::ParameterValue(8)); // Default inflation radius

        node->get_parameter(name_ + "." + "enabled", enabled_);
        node->get_parameter(name_ + "." + "map_topic", map_topic_);
        node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);

        RCLCPP_INFO(node->get_logger(), "ForegroundMaskLayer initialized with map_topic=%s, inflation_radius=%d",
                    map_topic_.c_str(), inflation_radius_);

        // Subscribe to the static map
        map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&ForegroundMaskLayer::incomingStaticMap, this, std::placeholders::_1));

        matchSize();
        current_ = true;
        enabled_ = true;
    }

    void ForegroundMaskLayer::incomingStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        static_map_ = msg;
        RCLCPP_INFO(logger_, "Static map received (%ux%u)", msg->info.width, msg->info.height);

        // Resize the inflated static map buffer to hold one cell per original map cell.
        inflated_static_.resize(static_map_->info.width * static_map_->info.height, 0);

        // Inflate the static map using the parameter inflation_radius_.
        for (unsigned int y = 0; y < static_map_->info.height; ++y)
        {
            for (unsigned int x = 0; x < static_map_->info.width; ++x)
            {
                unsigned int idx = x + y * static_map_->info.width;
                if (static_map_->data[idx] == 100) // Obstacle cell in static map
                {
                    for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
                    {
                        for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && ny >= 0 &&
                                nx < static_cast<int>(static_map_->info.width) &&
                                ny < static_cast<int>(static_map_->info.height))
                            {
                                unsigned int inflated_idx = nx + ny * static_map_->info.width;
                                inflated_static_[inflated_idx] = 100; // Mark as inflated obstacle value
                            }
                        }
                    }
                }
            }
        }
        RCLCPP_INFO(logger_, "Static map inflated.");
    }

    void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                          int min_i, int min_j, int max_i, int max_j)
    {
        ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
        
        if (!enabled_ || !static_map_)
        {
            return;  // No warning needed on every update
        }

        // Cache static map parameters to avoid repeated member access
        const double static_origin_x = static_map_->info.origin.position.x;
        const double static_origin_y = static_map_->info.origin.position.y;
        const double static_resolution = static_map_->info.resolution;
        const unsigned int static_width = static_map_->info.width;
        const unsigned int static_height = static_map_->info.height;
        const double inv_resolution = 1.0 / static_resolution;  // Multiply is faster than divide

        for (int j = min_j; j < max_j; ++j)
        {
            for (int i = min_i; i < max_i; ++i)
            {
                // Get world coordinates from costmap
                double wx, wy;
                master_grid.mapToWorld(i, j, wx, wy);

                // Convert to static map cell coordinates (use multiplication instead of division)
                int mx = static_cast<int>((wx - static_origin_x) * inv_resolution);
                int my = static_cast<int>((wy - static_origin_y) * inv_resolution);

                // Bounds check with signed integers (catches negative values)
                if (mx >= 0 && my >= 0 && 
                    mx < static_cast<int>(static_width) && 
                    my < static_cast<int>(static_height))
                {
                    unsigned int inflated_idx = mx + my * static_width;
                    unsigned char inflated_val = inflated_static_[inflated_idx];
                    unsigned char master_val = master_grid.getCost(i, j);  // Direct access, no need for getIndex

                    // Foreground detection: obstacle in costmap but NOT in inflated static map
                    if (master_val >= 90 && inflated_val < 50)
                    {
                        // This is a DYNAMIC obstacle (foreground)
                        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                    }
                    else
                    {
                        // Either no obstacle, or it's static (background) - clear it
                        master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                    }
                }
                else
                {
                    // Outside static map bounds - clear it (could also mark as unknown)
                    master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE);
                }
            }
        }

        current_ = true;
    }

} // namespace foreground_mask

PLUGINLIB_EXPORT_CLASS(foreground_mask::ForegroundMaskLayer, nav2_costmap_2d::Layer)