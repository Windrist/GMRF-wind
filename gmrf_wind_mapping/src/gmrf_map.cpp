#include "gmrf_map.h"
#include "Utils.h"
#include <algorithm>
#include <cstdlib>
#include <limits>

/*---------------------------------------------------------------
                        Constructor
  ---------------------------------------------------------------*/
CGMRF_map::CGMRF_map(rclcpp::Node *_node, const nav_msgs::msg::OccupancyGrid &oc_map, float cell_size, double m_lambdaPrior_reg,
                     double m_lambdaPrior_mass_conservation, double m_lambdaPrior_obstacles, std::string /* m_colormap */,
                     bool verbose, bool filter_unexplored)
    : node(_node), verbose(verbose), filter_unexplored_(filter_unexplored)
{
    try
    {
        // Copy params to internal variables
        m_Ocgridmap = oc_map;
        m_resolution = cell_size;
        lambdaPrior_reg = m_lambdaPrior_reg;
        lambdaPrior_mass_conservation = m_lambdaPrior_mass_conservation;
        lambdaPrior_obstacles = m_lambdaPrior_obstacles;

        // Set initial GMRF dimensions as the OccupancyMap (in meters)
        double x_min = oc_map.info.origin.position.x;
        double x_max = oc_map.info.origin.position.x + oc_map.info.width * oc_map.info.resolution;
        double y_min = oc_map.info.origin.position.y;
        double y_max = oc_map.info.origin.position.y + oc_map.info.height * oc_map.info.resolution;

        // Adjust size to comply with the desired resolution
        m_x_min = m_resolution * round(x_min / m_resolution);
        m_y_min = m_resolution * round(y_min / m_resolution);
        m_x_max = m_resolution * round(x_max / m_resolution);
        m_y_max = m_resolution * round(y_max / m_resolution);

        m_size_x = round((m_x_max - m_x_min) / m_resolution);
        m_size_y = round((m_y_max - m_y_min) / m_resolution);
        N = m_size_x * m_size_y;

        // Initialize visualization markers
        initializeVisualizationMarkers();

        RCLCPP_INFO(node->get_logger(), "[CGMRF] Generating GMRF for WIND estimation");

        // Initialize the map container (Wx: [0,N-1], Wy: [N,2N-1])
        TRandomFieldCell init_cell{0.0, 0.0};
        m_map.assign(2 * N, init_cell);

        if (verbose)
        {
            RCLCPP_INFO(node->get_logger(), "[CGMRF] GMRF created: (%lu,%lu) cells, resolution %.2fm, N=%lu nodes",
                        m_size_x, m_size_y, m_resolution, 2 * N);
        }

        // Initialize observation tracking
        nObsFactors = 0;
        activeObs.clear();

        // Build prior factors with visualization markers
        buildPriorFactors(true);

        init_colormaps("jet");
        RCLCPP_INFO(node->get_logger(), "[CGMRF] Initialization complete: %lu factors for %lu nodes", nFactors, 2 * N);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[GMRF-Constructor] Exception: %s", e.what());
    }
}

void CGMRF_map::initializeVisualizationMarkers()
{
    auto setupMarker = [this](visualization_msgs::msg::Marker &marker, const std::string &ns, int id,
                              float r, float g, float b)
    {
        marker.header.stamp = node->now();
        marker.ns = ns;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id;
        marker.points.clear();
        marker.scale.x = 0.02;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
    };

    setupMarker(line_list, "factors_reg", 0, 0.0, 0.0, 1.0);     // Blue for regularization
    setupMarker(line_list_obs, "factors_obs", 1, 1.0, 0.0, 0.0); // Red for obstacles
}

/*---------------------------------------------------------------
                        Destructor
  ---------------------------------------------------------------*/
CGMRF_map::~CGMRF_map()
{
}

/*---------------------------------------------------------------
                Dynamic Map Update Methods
  ---------------------------------------------------------------*/

bool CGMRF_map::hasMapChanged(const nav_msgs::msg::OccupancyGrid &new_map) const
{
    // Calculate new map bounds
    double new_x_min = new_map.info.origin.position.x;
    double new_x_max = new_map.info.origin.position.x + new_map.info.width * new_map.info.resolution;
    double new_y_min = new_map.info.origin.position.y;
    double new_y_max = new_map.info.origin.position.y + new_map.info.height * new_map.info.resolution;

    // Adjust to GMRF resolution
    float adj_x_min = m_resolution * round(new_x_min / m_resolution);
    float adj_x_max = m_resolution * round(new_x_max / m_resolution);
    float adj_y_min = m_resolution * round(new_y_min / m_resolution);
    float adj_y_max = m_resolution * round(new_y_max / m_resolution);

    // Check if bounds have changed significantly (more than half a cell)
    const float tolerance = m_resolution * 0.5;
    bool bounds_changed = (std::abs(adj_x_min - m_x_min) > tolerance) ||
                          (std::abs(adj_x_max - m_x_max) > tolerance) ||
                          (std::abs(adj_y_min - m_y_min) > tolerance) ||
                          (std::abs(adj_y_max - m_y_max) > tolerance);

    return bounds_changed;
}

std::vector<SavedObservation> CGMRF_map::getActiveObservations() const
{
    std::vector<SavedObservation> saved;
    saved.reserve(activeObs.size());

    for (const auto &obs : activeObs)
    {
        SavedObservation s;
        // Convert cell index back to world coordinates
        size_t cell_x = obs.cell_idx % m_size_x;
        size_t cell_y = obs.cell_idx / m_size_x;
        s.x_pos = m_x_min + (cell_x * m_resolution) + (m_resolution / 2);
        s.y_pos = m_y_min + (cell_y * m_resolution) + (m_resolution / 2);
        s.wind_speed = std::hypot(obs.windX, obs.windY);
        s.wind_direction = atan2(obs.windY, obs.windX);
        s.lambda = obs.lambda;
        s.time_invariant = obs.time_invariant;
        saved.push_back(s);
    }

    return saved;
}

void CGMRF_map::restoreObservations(const std::vector<SavedObservation> &observations)
{
    for (const auto &obs : observations)
    {
        // Check if the observation is within the new map bounds
        if (obs.x_pos >= m_x_min && obs.x_pos < m_x_max &&
            obs.y_pos >= m_y_min && obs.y_pos < m_y_max)
        {
            const int cellIdx = xy2idx(obs.x_pos, obs.y_pos);
            if (cellIdx >= 0 && static_cast<size_t>(cellIdx) < N && is_cell_free(cellIdx))
            {
                TobservationGMRF new_obs;
                new_obs.cell_idx = cellIdx;
                new_obs.windX = obs.wind_speed * cos(obs.wind_direction);
                new_obs.windY = obs.wind_speed * sin(obs.wind_direction);
                new_obs.lambda = obs.lambda;
                new_obs.time_invariant = obs.time_invariant;
                activeObs.push_back(new_obs);
                nObsFactors += 2;
            }
        }
    }

    if (verbose)
        RCLCPP_INFO(node->get_logger(), "[GMRF] Restored %lu observations after grid update", observations.size());
}

int CGMRF_map::mapOldIdxToNewIdx(size_t old_idx, size_t old_size_x, float old_x_min, float old_y_min) const
{
    // Convert old index to world coordinates
    size_t old_cell_x = old_idx % old_size_x;
    size_t old_cell_y = old_idx / old_size_x;
    double world_x = old_x_min + (old_cell_x * m_resolution) + (m_resolution / 2);
    double world_y = old_y_min + (old_cell_y * m_resolution) + (m_resolution / 2);

    // Convert world coordinates to new index
    if (world_x < m_x_min || world_x >= m_x_max || world_y < m_y_min || world_y >= m_y_max)
        return -1;

    return xy2idx(world_x, world_y);
}

bool CGMRF_map::expandGrid(float new_x_min, float new_x_max, float new_y_min, float new_y_max)
{
    try
    {
        // Store old dimensions
        size_t old_size_x = m_size_x;
        size_t old_size_y = m_size_y;
        size_t old_N = N;
        float old_x_min = m_x_min;
        float old_y_min = m_y_min;

        // Store old wind field values
        std::vector<TRandomFieldCell> old_map = m_map;

        // Update dimensions
        m_x_min = new_x_min;
        m_x_max = new_x_max;
        m_y_min = new_y_min;
        m_y_max = new_y_max;
        m_size_x = round((m_x_max - m_x_min) / m_resolution);
        m_size_y = round((m_y_max - m_y_min) / m_resolution);
        N = m_size_x * m_size_y;

        if (verbose)
        {
            RCLCPP_INFO(node->get_logger(), "[GMRF] Expanding grid from (%lu,%lu) to (%lu,%lu) cells",
                        old_size_x, old_size_y, m_size_x, m_size_y);
            RCLCPP_INFO(node->get_logger(), "[GMRF] New bounds: x=(%.2f,%.2f) y=(%.2f,%.2f)",
                        m_x_min, m_x_max, m_y_min, m_y_max);
        }

        // Initialize new map with zeros
        TRandomFieldCell init_cell;
        init_cell.mean = 0.0;
        init_cell.std = 0.0;
        m_map.assign(2 * N, init_cell);

        // Copy old values to new positions
        for (size_t old_idx = 0; old_idx < old_N; old_idx++)
        {
            int new_idx = mapOldIdxToNewIdx(old_idx, old_size_x, old_x_min, old_y_min);
            if (new_idx >= 0 && static_cast<size_t>(new_idx) < N)
            {
                // Copy Wx component
                m_map[new_idx].mean = old_map[old_idx].mean;
                m_map[new_idx].std = old_map[old_idx].std;
                // Copy Wy component
                m_map[new_idx + N].mean = old_map[old_idx + old_N].mean;
                m_map[new_idx + N].std = old_map[old_idx + old_N].std;
            }
        }

        // Update observation cell indices
        std::vector<TobservationGMRF> old_obs = activeObs;
        activeObs.clear();
        nObsFactors = 0;

        for (const auto &obs : old_obs)
        {
            int new_idx = mapOldIdxToNewIdx(obs.cell_idx, old_size_x, old_x_min, old_y_min);
            if (new_idx >= 0 && static_cast<size_t>(new_idx) < N)
            {
                TobservationGMRF new_obs = obs;
                new_obs.cell_idx = new_idx;
                activeObs.push_back(new_obs);
                nObsFactors += 2;
            }
        }

        if (verbose)
            RCLCPP_INFO(node->get_logger(), "[GMRF] Preserved %lu observations after grid expansion", activeObs.size());

        return true;
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[GMRF] Exception during grid expansion: %s", e.what());
        return false;
    }
}

bool CGMRF_map::updateOccupancyMap(const nav_msgs::msg::OccupancyGrid &new_map)
{
    try
    {
        // Calculate new bounds
        double new_x_min_raw = new_map.info.origin.position.x;
        double new_x_max_raw = new_map.info.origin.position.x + new_map.info.width * new_map.info.resolution;
        double new_y_min_raw = new_map.info.origin.position.y;
        double new_y_max_raw = new_map.info.origin.position.y + new_map.info.height * new_map.info.resolution;

        // Adjust to GMRF resolution
        float new_x_min = m_resolution * round(new_x_min_raw / m_resolution);
        float new_x_max = m_resolution * round(new_x_max_raw / m_resolution);
        float new_y_min = m_resolution * round(new_y_min_raw / m_resolution);
        float new_y_max = m_resolution * round(new_y_max_raw / m_resolution);

        // Check if grid needs expansion
        bool needs_expansion = (new_x_min < m_x_min) || (new_x_max > m_x_max) ||
                               (new_y_min < m_y_min) || (new_y_max > m_y_max);

        if (needs_expansion)
        {
            // Expand to encompass both old and new bounds
            float expanded_x_min = std::min(m_x_min, new_x_min);
            float expanded_x_max = std::max(m_x_max, new_x_max);
            float expanded_y_min = std::min(m_y_min, new_y_min);
            float expanded_y_max = std::max(m_y_max, new_y_max);

            if (!expandGrid(expanded_x_min, expanded_x_max, expanded_y_min, expanded_y_max))
            {
                RCLCPP_ERROR(node->get_logger(), "[GMRF] Failed to expand grid");
                return false;
            }
        }

        // Update occupancy grid reference
        m_Ocgridmap = new_map;

        // Rebuild prior factors with new occupancy information
        buildPriorFactors();

        RCLCPP_INFO(node->get_logger(), "[GMRF] Occupancy map updated successfully. Grid: (%lu,%lu), %lu factors",
                    m_size_x, m_size_y, nPriorFactors);

        return true;
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[GMRF] Exception updating occupancy map: %s", e.what());
        return false;
    }
}

void CGMRF_map::buildPriorFactors(bool include_visualization)
{
    // Clear existing prior factors
    J.clear();
    Lambda.clear();
    if (include_visualization)
    {
        line_list.points.clear();
        line_list_obs.points.clear();
    }

    // Estimate number of factors for memory reservation
    nPriorFactors = 2 * ((m_size_x - 1) * m_size_y + m_size_x * (m_size_y - 1));
    J.reserve(5 * nPriorFactors);
    Lambda.reserve(nPriorFactors);

    geometry_msgs::msg::Point p;
    p.z = 0;

    size_t count = 0;
    for (size_t j = 0; j < N; j++)
    {
        size_t jx, jy;
        id2cellxy(j, jx, jy);

        if (!is_cell_free(j))
        {
            // Force occupied cell to 0 value
            Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
            Eigen::Triplet<double> J_entry(count, j, 1);
            Lambda.push_back(lambda_entry);
            J.push_back(J_entry);
            count++;
            Eigen::Triplet<double> lambda_entry2(count, count, lambdaPrior_obstacles);
            Eigen::Triplet<double> J_entry2(count, j + N, 1);
            Lambda.push_back(lambda_entry2);
            J.push_back(J_entry2);
            count++;
        }

        // Factor with the right node: (j <--> j+1)
        if (jx < (m_size_x - 1))
        {
            if (is_cell_free(j) && is_cell_free(j + 1))
            {
                if (check_connectivity_between2cells(j, j + 1))
                {
                    // Regularization factor for Wx
                    Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_reg);
                    Eigen::Triplet<double> J_entry1(count, j, 1);
                    Eigen::Triplet<double> J_entry2(count, j + 1, -1);
                    Lambda.push_back(lambda_entry);
                    J.push_back(J_entry1);
                    J.push_back(J_entry2);
                    count++;

                    // Regularization factor for Wy
                    Eigen::Triplet<double> lambda_entry2(count, count, lambdaPrior_reg);
                    Eigen::Triplet<double> J_entry3(count, j + N, 1);
                    Eigen::Triplet<double> J_entry4(count, j + N + 1, -1);
                    Lambda.push_back(lambda_entry2);
                    J.push_back(J_entry3);
                    J.push_back(J_entry4);
                    count++;

                    if (include_visualization)
                    {
                        id2xy(j, p.x, p.y);
                        line_list.points.push_back(p);
                        id2xy(j + 1, p.x, p.y);
                        line_list.points.push_back(p);
                    }
                }
                else
                {
                    // Obstacle between cells - force Wx=0
                    Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                    Eigen::Triplet<double> J_entry(count, j, 1);
                    Lambda.push_back(lambda_entry);
                    J.push_back(J_entry);
                    count++;

                    Eigen::Triplet<double> lambda_entry2(count, count, lambdaPrior_obstacles);
                    Eigen::Triplet<double> J_entry2(count, j + 1, 1);
                    Lambda.push_back(lambda_entry2);
                    J.push_back(J_entry2);
                    count++;
                }
            }
            else if (is_cell_free(j))
            {
                Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                Eigen::Triplet<double> J_entry(count, j, 1);
                Lambda.push_back(lambda_entry);
                J.push_back(J_entry);
                count++;
            }
            else if (is_cell_free(j + 1))
            {
                Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                Eigen::Triplet<double> J_entry(count, j + 1, 1);
                Lambda.push_back(lambda_entry);
                J.push_back(J_entry);
                count++;
            }
        }

        // Factor with the upper node: (j <--> j+m_size_x)
        if (jy < (m_size_y - 1))
        {
            if (is_cell_free(j) && is_cell_free(j + m_size_x))
            {
                if (check_connectivity_between2cells(j, j + m_size_x))
                {
                    // Regularization factor for Wx
                    Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_reg);
                    Eigen::Triplet<double> J_entry1(count, j, 1);
                    Eigen::Triplet<double> J_entry2(count, j + m_size_x, -1);
                    Lambda.push_back(lambda_entry);
                    J.push_back(J_entry1);
                    J.push_back(J_entry2);
                    count++;

                    // Regularization factor for Wy
                    Eigen::Triplet<double> lambda_entry2(count, count, lambdaPrior_reg);
                    Eigen::Triplet<double> J_entry3(count, j + N, 1);
                    Eigen::Triplet<double> J_entry4(count, j + N + m_size_x, -1);
                    Lambda.push_back(lambda_entry2);
                    J.push_back(J_entry3);
                    J.push_back(J_entry4);
                    count++;

                    if (include_visualization)
                    {
                        id2xy(j, p.x, p.y);
                        line_list.points.push_back(p);
                        id2xy(j + m_size_x, p.x, p.y);
                        line_list.points.push_back(p);
                    }
                }
                else
                {
                    // Obstacle between cells - force Wy=0
                    Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                    Eigen::Triplet<double> J_entry(count, j + N, 1);
                    Lambda.push_back(lambda_entry);
                    J.push_back(J_entry);
                    count++;

                    Eigen::Triplet<double> lambda_entry2(count, count, lambdaPrior_obstacles);
                    Eigen::Triplet<double> J_entry2(count, j + N + m_size_x, 1);
                    Lambda.push_back(lambda_entry2);
                    J.push_back(J_entry2);
                    count++;
                }
            }
            else if (is_cell_free(j))
            {
                Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                Eigen::Triplet<double> J_entry(count, j + N, 1);
                Lambda.push_back(lambda_entry);
                J.push_back(J_entry);
                count++;
            }
            else if (is_cell_free(j + m_size_x))
            {
                Eigen::Triplet<double> lambda_entry(count, count, lambdaPrior_obstacles);
                Eigen::Triplet<double> J_entry(count, j + N + m_size_x, 1);
                Lambda.push_back(lambda_entry);
                J.push_back(J_entry);
                count++;
            }
        }

        // Mass conservation factors
        if (is_cell_free(j) && jx > 0 && jx < m_size_x - 1 && jy > 0 && jy < m_size_y - 1)
        {
            bool set = false;
            if (is_cell_free(j - 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j - 1, -1));
                set = true;
            }
            if (is_cell_free(j + 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j + 1, 1));
                set = true;
            }
            if (is_cell_free(j - m_size_x))
            {
                J.push_back(Eigen::Triplet<double>(count, j + N - m_size_x, -1));
                set = true;
            }
            if (is_cell_free(j + m_size_x))
            {
                J.push_back(Eigen::Triplet<double>(count, j + N + m_size_x, 1));
                set = true;
            }

            // Diagonals
            if (is_cell_free(j + m_size_x - 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j + m_size_x - 1, -0.5));
                J.push_back(Eigen::Triplet<double>(count, j + m_size_x - 1 + N, 0.5));
                set = true;
            }
            if (is_cell_free(j + m_size_x + 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j + m_size_x + 1, 0.5));
                J.push_back(Eigen::Triplet<double>(count, j + m_size_x + 1 + N, 0.5));
                set = true;
            }
            if (is_cell_free(j - m_size_x + 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j - m_size_x + 1, 0.5));
                J.push_back(Eigen::Triplet<double>(count, j - m_size_x + 1 + N, -0.5));
                set = true;
            }
            if (is_cell_free(j - m_size_x - 1))
            {
                J.push_back(Eigen::Triplet<double>(count, j - m_size_x - 1, -0.5));
                J.push_back(Eigen::Triplet<double>(count, j - m_size_x - 1 + N, -0.5));
                set = true;
            }

            if (set)
            {
                Lambda.push_back(Eigen::Triplet<double>(count, count, lambdaPrior_mass_conservation));
                count++;
            }
        }
    }

    nPriorFactors = count;
    nFactors = nPriorFactors + nObsFactors;

    if (verbose)
        RCLCPP_INFO(node->get_logger(), "[GMRF] Built %lu prior factors for %lu cells", nPriorFactors, N);
}

/*---------------------------------------------------------------
                        Cell index transformations
  ---------------------------------------------------------------*/
// Get x,y in cells (in the GMRF representation) from the general index in the array
void CGMRF_map::id2cellxy(size_t id, size_t &cell_x, size_t &cell_y)
{
    cell_x = id % m_size_x;
    cell_y = (size_t)floor(id / m_size_x);
}

// Get pose x,y (in meters) (in the GMRF representation) from the general index in the array
void CGMRF_map::id2xy(size_t id, double &x, double &y)
{
    size_t cell_x, cell_y;
    id2cellxy(id, cell_x, cell_y);

    x = m_x_min + (cell_x * m_resolution) + (m_resolution / 2);
    y = m_y_min + (cell_y * m_resolution) + (m_resolution / 2);
}

/*---------------------------------------------------------------
             Check if a cell is free of obstacles
  ---------------------------------------------------------------*/
// Occupancy threshold constants
static constexpr int8_t OCCUPANCY_FREE_THRESHOLD = 50;
static constexpr int8_t OCCUPANCY_OBSTACLE_THRESHOLD_H = 60;
static constexpr int8_t OCCUPANCY_OBSTACLE_THRESHOLD_V = 50;

// Check at OccupancyMap level, if a cell is free of obstacles (checking the cell center at GMRF resolution)
bool CGMRF_map::is_cell_free(size_t id_gmrf)
{
    // The pose x,y (meters) of cell center
    double cell_1_x, cell_1_y;
    id2xy(id_gmrf, cell_1_x, cell_1_y);

    // Get corresponding cell_idx in the Occupancy Gridmap
    // IMPORTANT --> Use the resolution and size of Occupancy Gridmap (not the GMRF)
    int id_oc;
    id_oc = static_cast<int>((cell_1_x - m_Ocgridmap.info.origin.position.x) / m_Ocgridmap.info.resolution);                           // x component
    id_oc += static_cast<int>((cell_1_y - m_Ocgridmap.info.origin.position.y) / m_Ocgridmap.info.resolution) * m_Ocgridmap.info.width; // y component

    // Bounds check to prevent undefined behavior
    if (id_oc < 0 || static_cast<size_t>(id_oc) >= m_Ocgridmap.data.size())
    {
        return false; // Out of bounds = not free
    }

    return m_Ocgridmap.data[id_oc] < OCCUPANCY_FREE_THRESHOLD;
}

/*---------------------------------------------------------------
             Check if a cell is explored (not unknown)
  ---------------------------------------------------------------*/
// Check at OccupancyMap level, if a cell has been explored (not unknown/unobserved)
// Unknown cells typically have value -1 in OccupancyGrid
bool CGMRF_map::is_cell_explored(size_t id_gmrf)
{
    // The pose x,y (meters) of cell center
    double cell_1_x, cell_1_y;
    id2xy(id_gmrf, cell_1_x, cell_1_y);

    // Get corresponding cell_idx in the Occupancy Gridmap
    int id_oc;
    id_oc = static_cast<int>((cell_1_x - m_Ocgridmap.info.origin.position.x) / m_Ocgridmap.info.resolution);
    id_oc += static_cast<int>((cell_1_y - m_Ocgridmap.info.origin.position.y) / m_Ocgridmap.info.resolution) * m_Ocgridmap.info.width;

    // Check if index is valid
    if (id_oc < 0 || static_cast<size_t>(id_oc) >= m_Ocgridmap.data.size())
    {
        return false; // Out of bounds = unexplored
    }

    // Check if cell is unknown (-1 indicates unexplored in OccupancyGrid)
    // Some implementations use values < 0 for unknown
    return m_Ocgridmap.data[id_oc] >= 0;
}

/*---------------------------------------------------------------
             Check cell interconnectivity
  ---------------------------------------------------------------*/
// Check at OccupancyMap level, if two cells are interconnected, that is no obstacles in between them.
// If ture, we will set a regularization factor.
bool CGMRF_map::check_connectivity_between2cells(size_t idx_1_gmrf, size_t idx_2_gmrf)
{
    // Get poses (x,y) of the cell centers in GMRF map
    double cell_1_x, cell_1_y, cell_2_x, cell_2_y;
    id2xy(idx_1_gmrf, cell_1_x, cell_1_y);
    id2xy(idx_2_gmrf, cell_2_x, cell_2_y);

    // Get corresponding cell_idx in the Occupancy Gridmap
    // IMPORTANT --> Use the resolution and size of Occupancy Gridmap (not the GMRF)
    int idx_1_oc, idx_2_oc;
    idx_1_oc = static_cast<int>((cell_1_x - m_Ocgridmap.info.origin.position.x) / m_Ocgridmap.info.resolution); // x component
    idx_1_oc +=
        static_cast<int>((cell_1_y - m_Ocgridmap.info.origin.position.y) / m_Ocgridmap.info.resolution) * m_Ocgridmap.info.width; // y component
    idx_2_oc = static_cast<int>((cell_2_x - m_Ocgridmap.info.origin.position.x) / m_Ocgridmap.info.resolution);                   // x component
    idx_2_oc +=
        static_cast<int>((cell_2_y - m_Ocgridmap.info.origin.position.y) / m_Ocgridmap.info.resolution) * m_Ocgridmap.info.width; // y component

    // Bounds validation
    const size_t map_size = m_Ocgridmap.data.size();
    if (idx_1_oc < 0 || idx_2_oc < 0 ||
        static_cast<size_t>(idx_1_oc) >= map_size || static_cast<size_t>(idx_2_oc) >= map_size)
    {
        return false; // Out of bounds = not connected
    }

    // check if cells are in the same row of the GMRF map
    const bool horizontal = (idx_2_gmrf == idx_1_gmrf + 1);

    // Check that a straight line between both cells centers is free of obstacles
    if (horizontal)
    {
        for (size_t p = idx_1_oc; p < static_cast<size_t>(idx_2_oc); p++)
        {
            if (p >= map_size) return false;
            if (m_Ocgridmap.data[p] >= OCCUPANCY_OBSTACLE_THRESHOLD_H)
            {
                return false;
            }
        }
    }
    else
    {
        for (size_t p = idx_1_oc; p < static_cast<size_t>(idx_2_oc); p += m_Ocgridmap.info.width)
        {
            if (p >= map_size) return false;
            if (m_Ocgridmap.data[p] >= OCCUPANCY_OBSTACLE_THRESHOLD_V)
            {
                return false;
            }
        }
    }

    return true;
}

/*---------------------------------------------------------------
             Insert New Wind Observation
---------------------------------------------------------------*/
void CGMRF_map::insertObservation_GMRF(double wind_speed, double wind_direction, double x_pos, double y_pos, double lambdaObs)
{
    try
    {
        auto add_obs = [this](const TobservationGMRF &observation)
        {
            if (observation.cell_idx >= N)
            {
                RCLCPP_ERROR(node->get_logger(), "Observation is outside of the map!");
                return;
            }
            activeObs.push_back(observation);
        };
        const int cellIdx = xy2idx(x_pos, y_pos);
        // Fill new Observation
        // The wind vector provided is already the DownWind direction in the map reference system
        if (x_pos <= m_x_min || x_pos >= m_x_max || y_pos <= m_y_min || y_pos >= m_y_max || !is_cell_free(cellIdx))
            return;

        // Filter observations in unexplored regions if filtering is enabled
        if (filter_unexplored_ && !is_cell_explored(cellIdx))
        {
            if (verbose)
                RCLCPP_DEBUG(node->get_logger(), "[GMRF] Rejecting observation at unexplored cell (%.2f, %.2f)", x_pos, y_pos);
            return;
        }

        TobservationGMRF new_obs;
        new_obs.cell_idx = cellIdx;
        new_obs.windX = wind_speed * cos(wind_direction);
        new_obs.windY = wind_speed * sin(wind_direction);
        new_obs.lambda = lambdaObs;
        new_obs.time_invariant = false; // Default behaviour, the obs will lose weight with time.
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "[GMRF] New obs: Wx = %.2f m/s Wy = %.2f m/s", new_obs.windX, new_obs.windY);

        // Add Observation to GMRF
        add_obs(new_obs);
        nObsFactors += 2; // we add 2 factors foe each observation to account for Wx and Wy components

        // NOTE --> We create 4 observations to expand a bit the measurement impact, replicating the content to neighbour cells
        // Also check that neighbor cells are explored when filtering is enabled
        auto is_valid_neighbor = [this](int idx)
        {
            if (!is_cell_free(idx))
                return false;
            if (filter_unexplored_ && !is_cell_explored(idx))
                return false;
            return true;
        };

        if (is_valid_neighbor(cellIdx - 1))
        {
            new_obs.cell_idx = cellIdx - 1;
            add_obs(new_obs);
            nObsFactors += 2;
        }
        if (is_valid_neighbor(cellIdx - m_size_x))
        {
            new_obs.cell_idx = cellIdx - m_size_x;
            add_obs(new_obs);
            nObsFactors += 2;
        }
        if (is_valid_neighbor(cellIdx - m_size_x - 1))
        {
            new_obs.cell_idx = cellIdx - m_size_x - 1;
            add_obs(new_obs);
            nObsFactors += 2;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[GMRF] Exception inserting observation: %s", e.what());
    }
}

/*---------------------------------------------------------------
                    updateMapEstimation_GMRF
  ---------------------------------------------------------------*/
void CGMRF_map::updateMapEstimation_GMRF(float lambdaObsLoss)
{
    try
    {
        /*
         * J (Jacobian) The J matrix contains the dr/dm for every factor in the graph
         *              J is size (nFactors x NumNodes)
         *
         * Lambda (weights) Is the Diagonal information matrix (contains the weights for each factor)
         *              Lambda is size (nFactors x nFactors)
         *
         * Y (vector of observations) contains the values of observations, 0 for prior factors
         *              y is size (nFactors x 1)
         *
         * R (Residuals) Since our system is deterministic, the residuals do not
         *              need to be re-evaluated on each iteration (we only perform 1 iteration).
         *              Therefore, R = -y, since we ALWAYS start from a all 0 map state.
         *
         * H (Hessian) = J' * Lambda * J
         *               H is size (NumNodes x NumNodes)
         *
         * G (gradient) = J' * Lambda * R
         *               g is size (NumNodes x 1)
         */

        // 1. Get current number of factors (nPriorFactors is constant, but nObsFactors is dynamic)
        nFactors = nPriorFactors + nObsFactors;

        // 2. Copy The prior part of Jacobian and Lambda matrices
        std::vector<Eigen::Triplet<double>> J_temp;
        J_temp.reserve(J.size() + nObsFactors);
        std::copy(J.begin(), J.end(), back_inserter(J_temp));

        std::vector<Eigen::Triplet<double>> Lambda_temp;
        Lambda_temp.reserve(Lambda.size() + nObsFactors);
        std::copy(Lambda.begin(), Lambda.end(), back_inserter(Lambda_temp));

        Eigen::VectorXd y_temp;
        y_temp.resize(nFactors);
        y_temp.fill(0.0);

        // 3. Include Active Observations into Jacobian and Lambda
        size_t count = nPriorFactors; // start after the already introduced prior factors
        for (std::vector<TobservationGMRF>::iterator ito = activeObs.begin(); ito != activeObs.end(); ++ito)
        {
            // Each observation translates to 2 factors (Wx,Wy)
            //  Wx range [1,N]
            Eigen::Triplet<double> lambda_entry(count, count, ito->lambda);
            Eigen::Triplet<double> J_entry(count, ito->cell_idx, 1);
            Lambda_temp.push_back(lambda_entry);
            J_temp.push_back(J_entry);
            y_temp[count] = ito->windX;
            count++;

            // Wy range [N+1,2N]
            Eigen::Triplet<double> lambda_entry2(count, count, ito->lambda);
            Eigen::Triplet<double> J_entry2(count, ito->cell_idx + N, 1);
            Lambda_temp.push_back(lambda_entry2);
            J_temp.push_back(J_entry2);
            y_temp[count] = ito->windY;
            count++;
        }

        // 3. Build Matrices (J, J', A, H, G)
        Eigen::SparseMatrix<double> Jsparse(nFactors, 2 * N); // declares a column-major sparse matrix type of float
        Jsparse.setFromTriplets(J_temp.begin(), J_temp.end());
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "          [GMRF] Jsparse is (%ld,%ld)", Jsparse.rows(), Jsparse.cols());

        Eigen::SparseMatrix<double> JsparseT; //(2*N,nFactors);				// declares a column-major sparse matrix type of float
        JsparseT = Eigen::SparseMatrix<double>(Jsparse.transpose());
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "          [GMRF] JsparseT is (%ld,%ld)", JsparseT.rows(), JsparseT.cols());

        Eigen::SparseMatrix<double> Asparse(nFactors, nFactors); // declares a column-major sparse matrix type of float
        Asparse.setFromTriplets(Lambda_temp.begin(), Lambda_temp.end());
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "          [GMRF] Asparse is (%ld,%ld)", Asparse.rows(), Asparse.cols());

        Eigen::SparseMatrix<double> Hsparse; //(2*N,2*N);   				// declares a column-major sparse matrix type of float
        Hsparse = JsparseT * Asparse * Jsparse;
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "          [GMRF] Hsparse is (%ld,%ld)", Hsparse.rows(), Hsparse.cols());

        Eigen::VectorXd G = JsparseT * Asparse * y_temp;
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "          [GMRF] G is (%lu,%lu)", G.rows(), G.cols());

        // 4. SOLVE
        // We need to solve: H * inc_m = -G
        // In an iterative scenario: m = m + inc_m;
        // In our case, we do not need to consider the previous state, so m = inc_m
        // We use a Cholesky Factorization of Hessian --> chol( P * H * inv(P) )
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(Hsparse);
        if (solver.info() != Eigen::Success)
        {
            RCLCPP_ERROR(node->get_logger(), "[GMRF] Cholesky decomposition failed!");
            return;
        }

        Eigen::VectorXd m_inc = solver.solve(G);
        if (solver.info() != Eigen::Success)
        {
            RCLCPP_ERROR(node->get_logger(), "[GMRF] Linear system solve failed!");
            return;
        }

        if (verbose)
            RCLCPP_INFO(node->get_logger(), "[GMRF] System solved with solution size (%lu,%lu)", m_inc.rows(), m_inc.cols());

        // 5. Update GMRF values from current solution
        for (size_t j = 0; j < m_map.size(); j++)
        {
            m_map[j].mean = m_inc(j);
            m_map[j].std = 0.0; // Not used currently
        }

        // 6. Update Information/Strength of Active Observations
        auto ito = activeObs.begin();
        while (ito != activeObs.end())
        {
            if (!ito->time_invariant)
            {
                ito->lambda -= lambdaObsLoss;
                if (ito->lambda <= 0.0)
                {
                    ito = activeObs.erase(ito);
                    nObsFactors -= 2;
                }
                else
                    ++ito;
            }
            else
                ++ito;
        }
        if (verbose)
            RCLCPP_INFO(node->get_logger(), "[GMRF] %lu ObservationFactors are active", nObsFactors);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "[GMRF] Exception updating maps: %s", e.what());
    }
}

WindVector CGMRF_map::getEstimation(int index)
{
    // If filtering is enabled and cell is unexplored, return zero wind
    if (filter_unexplored_ && !is_cell_explored(index))
    {
        return {0.0, 0.0, std::numeric_limits<double>::max()}; // Return invalid/zero wind with max uncertainty
    }

    double module = std::hypot(m_map[index].mean, m_map[index + N].mean);
    double direction = atan2(m_map[index + N].mean, m_map[index].mean);
    double stdev = std::max(0.1, std::hypot(m_map[index].std, m_map[index + N].std));

    return {module, direction, stdev};
}

WindVector CGMRF_map::getEstimation(double x, double y)
{
    int i = xy2idx(x, y);

    // Check bounds before querying
    if (i < 0 || static_cast<size_t>(i) >= N)
    {
        return {0.0, 0.0, std::numeric_limits<double>::max()};
    }

    return getEstimation(i);
}

void CGMRF_map::get_as_markerArray(visualization_msgs::msg::MarkerArray &ma, std::string frame_id)
{
    ma.markers.clear();
    // options (debug)
    bool plot_cell_centers = false;
    bool plot_factors = false;
    bool plot_wind_vectors = true;

    if (plot_cell_centers)
    {
        // marker-points at all cells
        visualization_msgs::msg::Marker marker_free;
        visualization_msgs::msg::Marker marker_occ;
        marker_free.header.frame_id = marker_occ.header.frame_id = frame_id.c_str();
        marker_free.header.stamp = marker_occ.header.stamp = node->now();
        marker_free.ns = marker_occ.ns = "cell_centers";
        marker_free.type = marker_occ.type = visualization_msgs::msg::Marker::POINTS;
        marker_free.action = marker_occ.action = visualization_msgs::msg::Marker::ADD;
        marker_free.id = 2;
        marker_occ.id = 3;
        // POINTS markers use x and y scale for width/height respectively
        marker_free.scale.x = 0.1;
        marker_free.scale.y = 0.1;
        marker_free.color.r = 0.0;
        marker_free.color.g = 1.0;
        marker_free.color.b = 0.0;
        marker_free.color.a = 1.0;

        marker_occ.scale.x = 0.1;
        marker_occ.scale.y = 0.1;
        marker_occ.color.r = 1.0;
        marker_occ.color.g = 0.0;
        marker_occ.color.b = 0.0;
        marker_occ.color.a = 1.0;

        // fill points
        marker_free.points.clear();
        marker_occ.points.clear();
        for (size_t i = 0; i < N; i++)
        {
            double cell_center_x, cell_center_y;
            geometry_msgs::msg::Point p;
            id2xy(i, cell_center_x, cell_center_y);
            p.x = cell_center_x;
            p.y = cell_center_y;

            if (is_cell_free(i))
                marker_free.points.push_back(p);
            else
                marker_occ.points.push_back(p);
        }
        // Push PointMarker to array
        ma.markers.push_back(marker_occ);
        ma.markers.push_back(marker_free);
    }

    if (plot_factors)
    {
        // Push Line_list marker to array
        line_list.header.frame_id = frame_id.c_str();
        line_list_obs.header.frame_id = frame_id.c_str();
        ma.markers.push_back(line_list);
        ma.markers.push_back(line_list_obs);
    }

    if (plot_wind_vectors)
    {
        // Add an ARROW marker for each node
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id.c_str();
        marker.header.stamp = node->now();
        marker.ns = "WindVector";
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Get max wind vector in the map (to normalize the plot)
        // Only consider explored cells when filtering is enabled
        // Pre-compute exploration status to avoid repeated expensive lookups
        std::vector<bool> cell_explored(N, true);
        if (filter_unexplored_)
        {
            for (size_t i = 0; i < N; i++)
            {
                cell_explored[i] = is_cell_explored(i);
            }
        }

        double max_module = 0.0;
        for (size_t i = 0; i < N; i++)
        {
            // Skip unexplored cells when filtering is enabled
            if (filter_unexplored_ && !cell_explored[i])
                continue;

            const double module = std::hypot(m_map[i].mean, m_map[i + N].mean);
            if (module > max_module)
                max_module = module;
        }

        // Prevent division by zero
        static constexpr double MIN_WIND_MODULE = 0.001;
        if (max_module < MIN_WIND_MODULE)
            max_module = MIN_WIND_MODULE;

        for (size_t i = 0; i < N; i++)
        {
            // Skip unexplored cells when filtering is enabled
            if (filter_unexplored_ && !cell_explored[i])
                continue;

            // if (is_cell_free(i))
            {
                const double module = std::hypot(m_map[i].mean, m_map[i + N].mean);
                // RCLCPP_INFO(node->get_logger(), "[GMRF] wind(%lu)=(%.2f,%.2f)m/s",i,m_map[i].mean,m_map[i+N].mean );
                if (module > MIN_WIND_MODULE)
                {
                    // Set the pose of the marker.
                    marker.id = i + 10;
                    double cell_center_x, cell_center_y;
                    id2xy(i, cell_center_x, cell_center_y);
                    marker.pose.position.x = cell_center_x;
                    marker.pose.position.y = cell_center_y;
                    marker.pose.orientation = Utils::createQuaternionMsgFromYaw(atan2(m_map[i + N].mean, m_map[i].mean));
                    // shape
                    marker.scale.x = m_resolution * (module / max_module); // arrow length,
                    marker.scale.y = 0.03;                                 // arrow width
                    marker.scale.z = 0.05;                                 // arrow height
                    // color -> must normalize to [0-199]
                    size_t idx_color = 199 * (module / max_module);
                    marker.color.r = color_r[idx_color];
                    marker.color.g = color_g[idx_color];
                    marker.color.b = color_b[idx_color];
                    marker.color.a = 1.0;

                    // Push Arrow to array
                    ma.markers.push_back(marker);
                }
            }
        } // end for
    }
}

void CGMRF_map::save_grmf_factor_graph(std::vector<Eigen::Triplet<double>> &Jout, std::vector<Eigen::Triplet<double>> &Aout, Eigen::VectorXd &yout)
{
    // Get output directory from environment or use /tmp as fallback
    const char* home_dir = std::getenv("HOME");
    std::string output_dir = home_dir ? std::string(home_dir) : "/tmp";

    bool save_dense = true;
    bool save_sparse = true;
    if (save_dense)
    {
        // 1. Jacobian
        Eigen::SparseMatrix<double> Jsparse(nFactors, 2 * N); // declares a column-major sparse matrix type of float
        Jsparse.setFromTriplets(Jout.begin(), Jout.end());
        Eigen::MatrixXd Jdense = Jsparse.toDense();

        // define the format you want, you only need one instance of this...
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        // RCLCPP_INFO(node->get_logger(), "[GMRF] Saving Factor-Graph to file...");
        std::ofstream file(output_dir + "/gmrf_jacobian_dense.txt");
        if (file.is_open())
        {
            file << Jdense.format(CSVFormat) << '\n';
        }
        file.close();

        // 2. Information Matrix
        Eigen::SparseMatrix<double> Asparse(nFactors, nFactors); // declares a column-major sparse matrix type of float
        Asparse.setFromTriplets(Aout.begin(), Aout.end());
        Eigen::MatrixXd Adense = Asparse.toDense();
        std::ofstream file2(output_dir + "/gmrf_lambda_dense.txt");
        if (file2.is_open())
        {
            file2 << Adense.format(CSVFormat) << '\n';
        }
        file2.close();
    }

    if (save_sparse)
    {
        RCLCPP_INFO(node->get_logger(), "[GMRF] Saving Factor-Graph (list of triplets) to file...");
        RCLCPP_INFO(node->get_logger(), "[GMRF] Jtriplets(%lu,3), Atriplets(%lu,3), numFactors(%lu)", Jout.size(), Aout.size(), yout.rows());

        // 1. Jacobian
        std::ofstream file(output_dir + "/gmrf_Jacobian.txt");
        if (file.is_open())
        {
            file << "# Jacobian of the GMRF: row col value"
                 << "\n";
            file << "# Num_cells_x = " << m_size_x << "\n";
            file << "# Num_cells_y = " << m_size_y << "\n";
            file << "# cell_size = " << m_resolution << "\n";
            for (std::vector<Eigen::Triplet<double>>::iterator it = Jout.begin(); it != Jout.end(); it++)
            {
                file << it->row() << " " << it->col() << " " << it->value() << "\n";
            }
        }
        file.close();

        // 2. Information Matrix
        std::ofstream file2(output_dir + "/gmrf_Lambda.txt");
        if (file2.is_open())
        {
            for (std::vector<Eigen::Triplet<double>>::iterator it = Aout.begin(); it != Aout.end(); it++)
            {
                file2 << it->row() << " " << it->col() << " " << it->value() << "\n";
            }
        }
        file2.close();

        // 3. Save vector of observations
        std::ofstream file3(output_dir + "/gmrf_observations.txt");
        if (file3.is_open())
        {
            file3 << yout;
        }
        file3.close();
    }
}

// Save Sparse matrices to file (for debug)
void CGMRF_map::save_grmf_factor_graph(Eigen::SparseMatrix<double> &H, Eigen::VectorXd &G)
{
    // Get output directory from environment or use /tmp as fallback
    const char* home_dir = std::getenv("HOME");
    std::string output_dir = home_dir ? std::string(home_dir) : "/tmp";

    // 1. Hessian
    std::ofstream file(output_dir + "/gmrf_hessian.txt");
    if (file.is_open())
    {
        file << "# Hessian of the GMRF: row col value"
             << "\n";
        for (int k = 0; k < H.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(H, k); it; ++it)
            {
                file << it.row() << " "; // row index
                file << it.col() << " "; // col index (here it is equal to k)
                file << it.value() << "\n";
            }
        }
    }
    file.close();

    // Hdense
    Eigen::MatrixXd Hdense = H.toDense();
    // define the format you want, you only need one instance of this...
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::ofstream file3(output_dir + "/gmrf_hessian_dense.txt");
    if (file3.is_open())
    {
        file3 << Hdense.format(CSVFormat) << '\n';
    }
    file3.close();

    // 2. Gradient
    std::ofstream file2(output_dir + "/gmrf_gradient.txt");
    if (file2.is_open())
    {
        file2 << G;
    }
    file2.close();
}

//------------------------------------------
// Build colormaps for visualization
//------------------------------------------
void CGMRF_map::init_colormaps(std::string colormap)
{
    // Static constexpr colormap arrays to avoid stack allocation
    static constexpr float jet_color_r[200] = {
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34,
        0.36, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52, 0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68, 0.70, 0.72, 0.74, 0.76, 0.78, 0.80,
        0.82, 0.84, 0.86, 0.88, 0.90, 0.92, 0.94, 0.96, 0.98, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.98, 0.96, 0.94, 0.92, 0.90, 0.88, 0.86, 0.84, 0.82,
        0.80, 0.78, 0.76, 0.74, 0.72, 0.70, 0.68, 0.66, 0.64, 0.62, 0.60, 0.58, 0.56, 0.54, 0.52, 0.50};
    static constexpr float jet_color_g[200] = {
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42,
        0.44, 0.46, 0.48, 0.50, 0.52, 0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68, 0.70, 0.72, 0.74, 0.76, 0.78, 0.80, 0.82, 0.84, 0.86, 0.88,
        0.90, 0.92, 0.94, 0.96, 0.98, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.98, 0.96, 0.94, 0.92, 0.90, 0.88, 0.86, 0.84, 0.82, 0.80, 0.78, 0.76, 0.74,
        0.72, 0.70, 0.68, 0.66, 0.64, 0.62, 0.60, 0.58, 0.56, 0.54, 0.52, 0.50, 0.48, 0.46, 0.44, 0.42, 0.40, 0.38, 0.36, 0.34, 0.32, 0.30, 0.28,
        0.26, 0.24, 0.22, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.04, 0.02, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
    static constexpr float jet_color_b[200] = {
        0.52, 0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68, 0.70, 0.72, 0.74, 0.76, 0.78, 0.80, 0.82, 0.84, 0.86, 0.88, 0.90, 0.92, 0.94, 0.96,
        0.98, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.98, 0.96, 0.94, 0.92, 0.90, 0.88, 0.86, 0.84, 0.82, 0.80, 0.78, 0.76, 0.74, 0.72, 0.70, 0.68, 0.66,
        0.64, 0.62, 0.60, 0.58, 0.56, 0.54, 0.52, 0.50, 0.48, 0.46, 0.44, 0.42, 0.40, 0.38, 0.36, 0.34, 0.32, 0.30, 0.28, 0.26, 0.24, 0.22, 0.20,
        0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.04, 0.02, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

    static constexpr float hot_color_r[200] = {
        0.01, 0.03, 0.04, 0.05, 0.07, 0.08, 0.09, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.19, 0.20, 0.21, 0.23, 0.24, 0.25, 0.27, 0.28, 0.29, 0.31,
        0.32, 0.33, 0.35, 0.36, 0.37, 0.39, 0.40, 0.41, 0.43, 0.44, 0.45, 0.47, 0.48, 0.49, 0.51, 0.52, 0.53, 0.55, 0.56, 0.57, 0.59, 0.60, 0.61,
        0.63, 0.64, 0.65, 0.67, 0.68, 0.69, 0.71, 0.72, 0.73, 0.75, 0.76, 0.77, 0.79, 0.80, 0.81, 0.83, 0.84, 0.85, 0.87, 0.88, 0.89, 0.91, 0.92,
        0.93, 0.95, 0.96, 0.97, 0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00};
    static constexpr float hot_color_g[200] = {
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.03, 0.04, 0.05, 0.07, 0.08, 0.09, 0.11, 0.12, 0.13, 0.15, 0.16, 0.17, 0.19, 0.20, 0.21, 0.23,
        0.24, 0.25, 0.27, 0.28, 0.29, 0.31, 0.32, 0.33, 0.35, 0.36, 0.37, 0.39, 0.40, 0.41, 0.43, 0.44, 0.45, 0.47, 0.48, 0.49, 0.51, 0.52, 0.53,
        0.55, 0.56, 0.57, 0.59, 0.60, 0.61, 0.63, 0.64, 0.65, 0.67, 0.68, 0.69, 0.71, 0.72, 0.73, 0.75, 0.76, 0.77, 0.79, 0.80, 0.81, 0.83, 0.84,
        0.85, 0.87, 0.88, 0.89, 0.91, 0.92, 0.93, 0.95, 0.96, 0.97, 0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
        1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00};
    static constexpr float hot_color_b[200] = {
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22,
        0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52, 0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.66, 0.68,
        0.70, 0.72, 0.74, 0.76, 0.78, 0.80, 0.82, 0.84, 0.86, 0.88, 0.90, 0.92, 0.94, 0.96, 0.98, 1.00};

    const float *src_r = jet_color_r;
    const float *src_g = jet_color_g;
    const float *src_b = jet_color_b;

    if (colormap == "hot")
    {
        src_r = hot_color_r;
        src_g = hot_color_g;
        src_b = hot_color_b;
    }

    std::copy(src_r, src_r + 200, color_r);
    std::copy(src_g, src_g + 200, color_g);
    std::copy(src_b, src_b + 200, color_b);
}

int CGMRF_map::xy2idx(float x, float y) const
{
    int x_idx = static_cast<int>((x - m_x_min) / m_resolution);
    int y_idx = static_cast<int>((y - m_y_min) / m_resolution);
    return x_idx + y_idx * m_size_x;
}