#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <odpp_msgs/msg/obstacle_array.hpp>
#include <odpp_msgs/msg/obstacle.hpp>
// #include <unique_id/unique_id.h>
#include <unique_identifier_msgs/msg/uuid.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <random>
#include <limits>
#include <map>
#include <thread>
#include <ctime>

// Updated BlobCluster includes boundary.
struct BlobCluster
{
    std::vector<geometry_msgs::msg::Point> points;   // raw blob points
    geometry_msgs::msg::Point centroid;              // average of raw points
    std::vector<geometry_msgs::msg::Point> boundary; // convex hull of blob points
};

//
// Structure to hold prediction results (placeholder VAR-like prediction).
// Each MSE is a flattened 2x2 matrix: [var_x, cov_xy, cov_xy, var_y] (zeros here).
//
struct PredictionResult
{
    std::vector<geometry_msgs::msg::Point> prediction;
    std::vector<std::vector<double>> prediction_mse;
};

//
// Structure for tracking obstacles.
//
struct TrackedObstacle
{
    int id;
    geometry_msgs::msg::Point last_centroid;
    std::vector<geometry_msgs::msg::Point> history;
    std::vector<geometry_msgs::msg::Point> prediction;
    std::vector<std::vector<double>> prediction_mse;
    int missed_frames = 0;
};

// -- Helper: generate a placeholder UUID with random bytes --
unique_identifier_msgs::msg::UUID generateRealUUID()
{
    unique_identifier_msgs::msg::UUID uuid;
    static const char *charset = "0123456789abcdef";
    // Seed the generator once. In production, prefer a better method.
    static bool seeded = false;
    if (!seeded)
    {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        seeded = true;
    }
    for (int i = 0; i < 16; i++)
    {
        int index = std::rand() % 16;                        // get an index 0..15
        uuid.uuid[i] = static_cast<uint8_t>(charset[index]); // store ascii code of chosen character
    }
    // Set the version (slot 6) to '4'
    uuid.uuid[6] = '4';
    // Set the variant (slot 8) to one of "8", "9", "a", or "b"
    const char *varset = "89ab";
    uuid.uuid[8] = varset[std::rand() % 4];
    return uuid;
}

class DynamicObstacleNode : public rclcpp::Node
{
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node"),
          next_obstacle_id_(0)
    {
        // Declare prediction-related parameters.
        this->declare_parameter("length_future_time_series", 7);
        this->declare_parameter("min_samples_for_prediction", 10);
        this->declare_parameter("var_time_steps", 0.5);
        this->declare_parameter("static_threshold", 0.05);

        length_future_time_series_ = this->get_parameter("length_future_time_series").as_int();
        min_samples_for_prediction_ = this->get_parameter("min_samples_for_prediction").as_int();
        var_time_steps_ = this->get_parameter("var_time_steps").as_double();
        static_threshold_ = this->get_parameter("static_threshold").as_double();

        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "dynamic_obstacle_costmap", std::string{get_namespace()}, "dynamic_obstacle_costmap");

        costmap_thread_ = std::make_unique<std::thread>(
            [](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
            {
                rclcpp::spin(node->get_node_base_interface());
            },
            costmap_ros_);

        rclcpp_lifecycle::State state;
        costmap_ros_->on_configure(state);
        costmap_ros_->on_activate(state);

        // Timer to process costmap (blob detection + tracking)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        // Publisher for custom obstacles.
        obstacle_pub_ = this->create_publisher<odpp_msgs::msg::ObstacleArray>("obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode started.");
    }

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<odpp_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;

    // Tracking variables.
    std::map<int, TrackedObstacle> tracked_obstacles_;
    int next_obstacle_id_;

    // Prediction parameters.
    int length_future_time_series_;
    int min_samples_for_prediction_;
    double var_time_steps_;
    double static_threshold_;

    // Compute centroid from points.
    geometry_msgs::msg::Point computeCentroid(const std::vector<geometry_msgs::msg::Point> &points)
    {
        geometry_msgs::msg::Point centroid;
        centroid.x = 0;
        centroid.y = 0;
        centroid.z = 0;
        for (const auto &p : points)
        {
            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= points.size();
        centroid.y /= points.size();
        return centroid;
    }

    // Helper: cross product of OA x OB.
    double cross(const geometry_msgs::msg::Point &O, const geometry_msgs::msg::Point &A, const geometry_msgs::msg::Point &B)
    {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    // Extract convex hull using Gift Wrapping.
    std::vector<geometry_msgs::msg::Point> computeConvexHull(const std::vector<geometry_msgs::msg::Point> &points)
    {
        std::vector<geometry_msgs::msg::Point> hull;
        if (points.size() < 3)
            return points;
        size_t leftmost = 0;
        for (size_t i = 1; i < points.size(); ++i)
        {
            if (points[i].x < points[leftmost].x ||
                (points[i].x == points[leftmost].x && points[i].y < points[leftmost].y))
                leftmost = i;
        }
        size_t p = leftmost, q;
        do
        {
            hull.push_back(points[p]);
            q = (p + 1) % points.size();
            for (size_t i = 0; i < points.size(); i++)
            {
                if (cross(points[p], points[i], points[q]) > 0)
                    q = i;
            }
            p = q;
        } while (p != leftmost);
        return hull;
    }

    // Hungarian assignment (Kuhn-Munkres) for data association.
    std::vector<int> hungarianAssignment(const std::vector<std::vector<double>> &costMatrix)
    {
        size_t n = costMatrix.size();
        std::vector<double> u(n + 1, 0), v(n + 1, 0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);
        for (size_t i = 1; i <= n; i++)
        {
            p[0] = i;
            std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
            std::vector<bool> used(n + 1, false);
            int j0 = 0;
            do
            {
                used[j0] = true;
                int i0 = p[j0];
                double delta = std::numeric_limits<double>::max();
                int j1 = 0;
                for (size_t j = 1; j <= n; j++)
                {
                    if (!used[j])
                    {
                        double cur = costMatrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j])
                        {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta)
                        {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for (size_t j = 0; j <= n; j++)
                {
                    if (used[j])
                        u[p[j]] += delta;
                    else
                        minv[j] -= delta;
                }
                j0 = j1;
            } while (p[j0] != 0);
            do
            {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }
        std::vector<int> assignment(n, -1);
        for (size_t j = 1; j <= n; j++)
        {
            assignment[p[j] - 1] = j - 1;
        }
        return assignment;
    }

    //
    // Predict future states using a placeholder constant velocity model.
    // If not enough history, returns repeated current centroid.
    //
    PredictionResult predictFutureStates(const TrackedObstacle &obs)
    {
        PredictionResult result;
        if (static_cast<int>(obs.history.size()) < min_samples_for_prediction_)
        {
            for (int i = 0; i < length_future_time_series_; i++)
            {
                result.prediction.push_back(obs.last_centroid);
                result.prediction_mse.push_back({0.0, 0.0, 0.0, 0.0});
            }
            return result;
        }
        geometry_msgs::msg::Point last = obs.last_centroid;
        geometry_msgs::msg::Point second_last = obs.history[obs.history.size() - 2];
        double vx = last.x - second_last.x;
        double vy = last.y - second_last.y;
        for (int i = 0; i < length_future_time_series_; i++)
        {
            geometry_msgs::msg::Point pred;
            pred.x = last.x + (i + 1) * vx * var_time_steps_;
            pred.y = last.y + (i + 1) * vy * var_time_steps_;
            pred.z = last.z;
            result.prediction.push_back(pred);
            result.prediction_mse.push_back({0.0, 0.0, 0.0, 0.0});
        }
        return result;
    }

    //
    // Tracking update via data association.
    // All obstacles are treated as dynamic.
    //
    void trackObstacles(const std::vector<BlobCluster> &clusters)
    {
        if (tracked_obstacles_.empty())
        {
            for (const auto &cluster : clusters)
            {
                TrackedObstacle obs;
                obs.id = next_obstacle_id_++;
                obs.last_centroid = cluster.centroid;
                obs.history.push_back(cluster.centroid);
                obs.prediction = predictFutureStates(obs).prediction;
                obs.prediction_mse = predictFutureStates(obs).prediction_mse;
                tracked_obstacles_[obs.id] = obs;
            }
            return;
        }

        std::vector<geometry_msgs::msg::Point> predicted;
        std::vector<int> tracked_ids;
        for (const auto &pair : tracked_obstacles_)
        {
            const TrackedObstacle &obs = pair.second;
            predicted.push_back(!obs.prediction.empty() ? obs.prediction.front() : obs.last_centroid);
            tracked_ids.push_back(obs.id);
        }
        size_t n = predicted.size();
        size_t m = clusters.size();
        size_t size = std::max(n, m);
        std::vector<std::vector<double>> costMatrix(size, std::vector<double>(size, 1e6));
        for (size_t i = 0; i < n; i++)
        {
            for (size_t j = 0; j < m; j++)
            {
                double dx = predicted[i].x - clusters[j].centroid.x;
                double dy = predicted[i].y - clusters[j].centroid.y;
                costMatrix[i][j] = std::hypot(dx, dy);
            }
        }
        std::vector<int> assignment = hungarianAssignment(costMatrix);
        std::vector<bool> matchedCluster(m, false);
        std::set<int> matched_ids;
        for (size_t i = 0; i < n; i++)
        {
            int cluster_idx = assignment[i];
            if (cluster_idx < 0 || cluster_idx >= static_cast<int>(m))
                continue;
            if (costMatrix[i][cluster_idx] < 1.0) // threshold
            {
                int tid = tracked_ids[i];
                tracked_obstacles_[tid].last_centroid = clusters[cluster_idx].centroid;
                tracked_obstacles_[tid].history.push_back(clusters[cluster_idx].centroid);
                PredictionResult pr = predictFutureStates(tracked_obstacles_[tid]);
                tracked_obstacles_[tid].prediction = pr.prediction;
                tracked_obstacles_[tid].prediction_mse = pr.prediction_mse;
                tracked_obstacles_[tid].missed_frames = 0;
                matched_ids.insert(tid);
                matchedCluster[cluster_idx] = true;
            }
        }
        for (size_t j = 0; j < m; j++)
        {
            if (!matchedCluster[j])
            {
                TrackedObstacle obs;
                obs.id = next_obstacle_id_++;
                obs.last_centroid = clusters[j].centroid;
                obs.history.push_back(clusters[j].centroid);
                PredictionResult pr = predictFutureStates(obs);
                obs.prediction = pr.prediction;
                obs.prediction_mse = pr.prediction_mse;
                tracked_obstacles_[obs.id] = obs;
            }
        }
        for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end();)
        {
            if (matched_ids.find(it->first) == matched_ids.end())
            {
                it->second.missed_frames++;
                if (it->second.missed_frames > 3)
                {
                    it = tracked_obstacles_.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

    //
    // Publish an ObstacleArray message.
    // Fills each obstacle with a random placeholder UUID and computes velocity/heading.
    //
    void publishObstacles(const std::vector<BlobCluster> &clusters)
    {
        odpp_msgs::msg::ObstacleArray msg;
        msg.header.stamp = now();
        msg.header.frame_id = costmap_ros_->getGlobalFrameID();

        for (const auto &pair : tracked_obstacles_)
        {
            const TrackedObstacle &obs = pair.second;
            odpp_msgs::msg::Obstacle obst;
            obst.uuid = generateRealUUID();
            obst.score = 1.0f;
            obst.id = obs.id;

            // Set positions: first is current centroid, others are predictions.
            obst.position.clear();
            obst.position.push_back(obs.last_centroid);
            for (const auto &pred : obs.prediction)
                obst.position.push_back(pred);

            // Compute velocity based on last two history entries.
            geometry_msgs::msg::Vector3 vel;
            if (obs.history.size() >= 2)
            {
                const auto &p_last = obs.history.back();
                const auto &p_prev = obs.history[obs.history.size() - 2];
                vel.x = p_last.x - p_prev.x;
                vel.y = p_last.y - p_prev.y;
                vel.z = 0.0;
            }
            else
            {
                vel.x = 0.0;
                vel.y = 0.0;
                vel.z = 0.0;
            }
            obst.velocity = vel;

            // Compute heading as normalized velocity.
            geometry_msgs::msg::Vector3 heading;
            double norm = std::hypot(vel.x, vel.y);
            if (norm > 0.0)
            {
                heading.x = vel.x / norm;
                heading.y = vel.y / norm;
                heading.z = 0.0;
            }
            else
            {
                heading.x = 0.0;
                heading.y = 0.0;
                heading.z = 0.0;
            }
            obst.heading = heading;

            // Use convex hull from the nearest blob cluster.
            geometry_msgs::msg::Polygon poly;
            for (const auto &cluster : clusters)
            {
                if (std::hypot(cluster.centroid.x - obs.last_centroid.x,
                               cluster.centroid.y - obs.last_centroid.y) < 1.0)
                {
                    for (const auto &pt : cluster.boundary)
                    {
                        geometry_msgs::msg::Point32 pt32;
                        pt32.x = pt.x;
                        pt32.y = pt.y;
                        pt32.z = pt.z;
                        poly.points.push_back(pt32);
                    }
                    break;
                }
            }
            obst.polygon = poly;

            // Fill covariances (placeholder).
            obst.position_covariance.fill(0.0);
            obst.velocity_covariance.fill(0.0);
            if (!obs.prediction_mse.empty())
            {
                obst.position_covariance[0] = obs.prediction_mse[0][0];
                obst.position_covariance[1] = obs.prediction_mse[0][1];
                obst.position_covariance[3] = obs.prediction_mse[0][1];
                obst.position_covariance[4] = obs.prediction_mse[0][3];
            }
            msg.obstacles.push_back(obst);
        }
        obstacle_pub_->publish(msg);
    }

    // Main processing function (blob detection via flood fill).
    void processCostmap()
    {
        auto costmap = costmap_ros_->getCostmap();
        unsigned char cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        std::vector<std::vector<bool>> visited(size_x, std::vector<bool>(size_y, false));
        using Cell = std::pair<unsigned int, unsigned int>;
        std::vector<BlobCluster> clusters;
        auto floodFill = [&](unsigned int i, unsigned int j, std::vector<geometry_msgs::msg::Point> &cluster)
        {
            std::vector<Cell> stack = {{i, j}};
            while (!stack.empty())
            {
                auto [ci, cj] = stack.back();
                stack.pop_back();
                if (visited[ci][cj])
                    continue;
                visited[ci][cj] = true;
                if (costmap->getCost(ci, cj) >= cost_threshold)
                {
                    double wx, wy;
                    costmap->mapToWorld(ci, cj, wx, wy);
                    geometry_msgs::msg::Point p;
                    p.x = wx;
                    p.y = wy;
                    p.z = 0.0;
                    cluster.push_back(p);
                    for (int di = -1; di <= 1; ++di)
                    {
                        for (int dj = -1; dj <= 1; ++dj)
                        {
                            int ni = ci + di;
                            int nj = cj + dj;
                            if (ni >= 0 && nj >= 0 &&
                                ni < static_cast<int>(size_x) &&
                                nj < static_cast<int>(size_y) &&
                                !visited[ni][nj])
                            {
                                stack.push_back({ni, nj});
                            }
                        }
                    }
                }
            }
        };

        for (unsigned int i = 0; i < size_x; ++i)
        {
            for (unsigned int j = 0; j < size_y; ++j)
            {
                if (!visited[i][j] && costmap->getCost(i, j) >= cost_threshold)
                {
                    std::vector<geometry_msgs::msg::Point> cluster_points;
                    floodFill(i, j, cluster_points);
                    if (!cluster_points.empty())
                    {
                        BlobCluster cluster;
                        cluster.points = cluster_points;
                        cluster.centroid = computeCentroid(cluster_points);
                        cluster.boundary = computeConvexHull(cluster_points);
                        clusters.push_back(cluster);
                    }
                }
            }
        }
        trackObstacles(clusters);
        publishObstacles(clusters);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}