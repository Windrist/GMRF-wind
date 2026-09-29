//========================================================================================
//	GMRF_node - Wind Estimation
//	Description: Implements the Gaussian Markov random field mapping algorithm for the
//               estimation of the windflow from a set of sparse 2D wind measurements.
//
//	topics subscribed:
//  topics published:
//	services:
//
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//	Revision log:
//	version: 1.0	23/02/2017
//========================================================================================

#include "gmrf_node.h"
#include "Utils.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace std::placeholders;

Cgmrf::Cgmrf()
    : Node("GMRF_wind")
{
    printf("\n=================================================================");
    printf("\n=	             GMRF Wind-Distribution Mapping Node              =");
    printf("\n=================================================================\n");

    //------------------
    // Load Parameters
    //------------------
    frame_id = declare_parameter<std::string>("frame_id", "map");
    sensor_topic = declare_parameter<std::string>("sensor_topic", "/anemometer");
    exec_freq = declare_parameter<double>("exec_freq", 2.0);
    cell_size = declare_parameter<double>("cell_size", 0.5);

    GMRF_lambdaPrior_reg =
        declare_parameter<double>("GMRF_lambdaPrior_reg", 1); // Weight for regularization prior -> neighbour cells have similar wind vectors
    GMRF_lambdaPrior_mass_conservation =
        declare_parameter<double>("GMRF_lambdaPrior_mass_conservation", 10000); // Weight for mass conservation law prior
    GMRF_lambdaPrior_obstacles = declare_parameter<double>(
        "GMRF_lambdaPrior_obstacles", 10);                              // Weight for wind close to obstacles prior -->cells close to obstacles has only tangencial wind
    GMRF_lambdaObs = declare_parameter<double>("GMRF_lambdaObs", 10.0); // [GMRF model] The initial weight (Lambda) of each observation
    GMRF_lambdaObsLoss = declare_parameter<double>(
        "GMRF_lambdaObsLoss", 0.0); // [GMRF model] The loss of information (Lambda) of the observations with each iteration (see AppTick)

    colormap = declare_parameter<std::string>("colormap", "jet");

    //----------------------------------
    // Subscriptions
    //----------------------------------
    sub_sensor = create_subscription<olfaction_msgs::msg::Anemometer>(
        sensor_topic, rclcpp::SensorDataQoS(), std::bind(&Cgmrf::sensorCallback, this, _1));
    ocupancyMap_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("map_topic", "map"), rclcpp::QoS(1).transient_local().reliable(), std::bind(&Cgmrf::mapCallback, this, _1));
    //----------------------------------
    // Publishers
    //----------------------------------
    wind_array_pub = create_publisher<visualization_msgs::msg::MarkerArray>("wind_array_pub", 1);
    //----------------------------------
    // Services
    //----------------------------------
    // rclcpp::ServiceServer service = param_n.advertiseService("suggestNextObservationLocation", suggestNextObservationLocation);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::Duration(std::chrono::seconds(30)));
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    verbose = declare_parameter<bool>("verbose", false);

    // Dynamic map update parameters
    map_update_cooldown_ = declare_parameter<double>("map_update_cooldown", 5.0); // seconds
    last_map_update_time_ = this->now();

    // Filter unexplored regions parameter
    filter_unexplored_ = declare_parameter<bool>("filter_unexplored_regions", true);

    module_init = false;
}

Cgmrf::~Cgmrf()
{
}

//--------------------------
// CALLBACK - OCCUPANCY MAP
//--------------------------
void Cgmrf::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Handle dynamic map updates
    if (module_init)
    {
        // Check if the map has changed significantly
        if (my_map->hasMapChanged(*msg))
        {
            // Apply rate limiting for map updates
            auto now = this->now();
            if ((now - last_map_update_time_).seconds() < map_update_cooldown_)
            {
                if (verbose)
                    RCLCPP_DEBUG(get_logger(), "[GMRF] Map change detected but cooldown active (%.1fs remaining)",
                                 map_update_cooldown_ - (now - last_map_update_time_).seconds());
                return;
            }

            RCLCPP_INFO(get_logger(), "[GMRF] Map dimensions changed, updating GMRF grid...");

            // Update the occupancy map (handles grid expansion and factor rebuilding)
            if (my_map->updateOccupancyMap(*msg))
            {
                last_map_update_time_ = now;
                RCLCPP_INFO(get_logger(), "[GMRF] Grid update completed successfully");
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "[GMRF] Failed to update grid with new map");
            }
        }
        else
        {
            // Map bounds haven't changed, but occupancy data might have
            // Update the occupancy grid reference for obstacle checking
            // This is a lightweight update that doesn't require grid expansion
            my_map->updateOccupancyMap(*msg);
        }
        return;
    }

    // First-time initialization
    // we can choose to read a map file directly from disk, if we don't want to use the one published by map_server
    // this is often useful when we need to alter the occupancy map to include outlets, which should be empty for GMRF but which may not be navigable (think windows)
    std::string mapFilePath = declare_parameter<std::string>("map_file", "");
    if (mapFilePath != "")
    {
        RCLCPP_INFO(get_logger(), "Reading map from file '%s'", mapFilePath.c_str());
        occupancyMap = Utils::parseMapImage(mapFilePath);
        occupancyMap.header = msg->header;
        occupancyMap.info = msg->info;
    }
    else
    {
        occupancyMap = *msg;
        RCLCPP_INFO(get_logger(), "Using map from topic '%s'", ocupancyMap_sub->get_topic_name());
    }

    // publish the actual map we are using, in case it is different from the one published by map_server
    static rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_republisher =
        create_publisher<nav_msgs::msg::OccupancyGrid>("gmrf_occupancy", rclcpp::QoS(1).transient_local());

    map_republisher->publish(occupancyMap);

    initialize();
}

void Cgmrf::initialize()
{
    // Set GasMap dimensions as the OccupancyMap
    // Note: map bounds are calculated but only used for logging/debugging

    // Create GMRF-Map and init
    my_map = std::make_unique<CGMRF_map>(this, occupancyMap, cell_size, GMRF_lambdaPrior_reg, GMRF_lambdaPrior_mass_conservation,
                                         GMRF_lambdaPrior_obstacles, colormap, verbose, filter_unexplored_);
    RCLCPP_INFO(get_logger(), "[GMRF-node] GMRF GridMap initialized (filter_unexplored=%s)", filter_unexplored_ ? "true" : "false");

    module_init = true;
}

//----------------------------------
// CALLBACK - NEW WIND OBSERVATION
//----------------------------------
void Cgmrf::sensorCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
{
    if (!module_init)
        return;

    // Use the same timestamped transform for sensor position and flow direction.
    // The TF listener runs on its own thread; wait briefly when TF trails the sample.
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer->lookupTransform(
            frame_id, msg->header.frame_id, msg->header.stamp,
            tf2::Duration(std::chrono::milliseconds(100)));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "[GMRF] Exception reading observation: %s", ex.what());
        return;
    }

    const double speed = msg->wind_speed;
    double downwind_direction_map = 0.0;
    if (speed != 0.0)
    {
        geometry_msgs::msg::PoseStamped upwind_sensor, upwind_map;
        upwind_sensor.header = msg->header;
        upwind_sensor.pose.orientation = Utils::createQuaternionMsgFromYaw(
            0.5 * M_PI - msg->wind_direction);
        tf2::doTransform(upwind_sensor, upwind_map, transform);
        downwind_direction_map = angles::normalize_angle(
            Utils::getYaw(upwind_map.pose.orientation) + M_PI);
    }
    my_map->insertObservation_GMRF(
        speed, downwind_direction_map, transform.transform.translation.x,
        transform.transform.translation.y, GMRF_lambdaObs);
}

void Cgmrf::publishMaps()
{
    visualization_msgs::msg::MarkerArray wind_array;
    my_map->get_as_markerArray(wind_array, frame_id);
    wind_array_pub->publish(wind_array);
}

bool Cgmrf::get_wind_value_srv(WindEstimation::Request::SharedPtr req, WindEstimation::Response::SharedPtr res)
{
    if (!module_init)
    {
        RCLCPP_ERROR(get_logger(), "Trying to query GMRF wind, but it is not initialized yet (probably has not received the occupancy map)");
        return false;
    }

    Eigen::Vector2i dimensions = my_map->map_size();
    res->map_width = dimensions.x();
    // an empty request means get all the points
    if (req->x.empty())
    {
        size_t size = dimensions.x() * dimensions.y();

        res->u.reserve(size);
        res->v.reserve(size);
        res->stdev_angle.reserve(size);

        for (size_t i = 0; i < size; i++)
        {
            WindVector r = my_map->getEstimation(static_cast<int>(i));
            Eigen::Vector2d vec = r.asEigen();
            res->u.push_back(vec.x());
            res->v.push_back(vec.y());
            res->stdev_angle.push_back(r.stdDev);
        }

        return true;
    }

    // Since the wind fields are identical among different instances, return just the information from instance[0]
    for (size_t i = 0; i < req->x.size(); i++)
    {
        WindVector r = my_map->getEstimation(req->x[i], req->y[i]);
        Eigen::Vector2d vec = r.asEigen();
        res->u.push_back(vec.x());
        res->v.push_back(vec.y());
        res->stdev_angle.push_back(r.stdDev);
    }
    return true;
}

//-----------------------------------------------------------------------------
//                                    MAIN
//----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto my_gmrf_map = std::make_shared<Cgmrf>();

    auto service = my_gmrf_map->create_service<WindEstimation>("WindEstimation", std::bind(&Cgmrf::get_wind_value_srv, my_gmrf_map.get(), _1, _2));
    RCLCPP_INFO(my_gmrf_map->get_logger(), "[gmrf] LOOP....");
    rclcpp::Time last_publication_time = my_gmrf_map->now();
    const double frequency = my_gmrf_map->exec_freq;
    const double period_ns = 1e9 / frequency;
    if (!std::isfinite(frequency) || frequency <= 0.0 || !std::isfinite(period_ns) ||
        period_ns < 1.0 || period_ns >= static_cast<double>(std::numeric_limits<int64_t>::max()))
    {
        RCLCPP_ERROR(my_gmrf_map->get_logger(), "exec_freq must define a finite positive timer period");
        rclcpp::shutdown();
        return 1;
    }
    // Drain subscriptions continuously; only the solve is limited to exec_freq.
    auto timer = my_gmrf_map->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(period_ns)), [&]()
        {

        if (my_gmrf_map->module_init)
        {
            // Update and Publish maps
            my_gmrf_map->my_map->updateMapEstimation_GMRF(my_gmrf_map->GMRF_lambdaObsLoss);
            my_gmrf_map->publishMaps();

            // Info about actual rate
            if (my_gmrf_map->verbose)
                RCLCPP_INFO(my_gmrf_map->get_logger(), "[gmrf] Updating every %f seconds. Intended preiod was %f",
                            (my_gmrf_map->now() - last_publication_time).seconds(), 1.0 / my_gmrf_map->exec_freq);
            last_publication_time = my_gmrf_map->now();
        }
        else
        {
            if (my_gmrf_map->verbose)
                RCLCPP_INFO(my_gmrf_map->get_logger(), "[gmrf] Waiting for initialization (Map of environment).");
        } });
    rclcpp::spin(my_gmrf_map);
    rclcpp::shutdown();
}
