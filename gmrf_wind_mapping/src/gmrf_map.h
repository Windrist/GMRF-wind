#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <eigen3/Eigen/Sparse>
#include <fstream>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>

struct TRandomFieldCell
{
    double mean;
    double std;
};

struct WindVector
{
    double module;
    double direction;
    double stdDev;

    Eigen::Vector2d asEigen()
    {
        double x = module * std::cos(direction);
        double y = module * std::sin(direction);
        return Eigen::Vector2d(x, y);
    }
};

/**
 * Structure to save observation data for backup/restore during grid expansion
 */
struct SavedObservation
{
    double x_pos;
    double y_pos;
    double wind_speed;
    double wind_direction;
    double lambda;
    bool time_invariant;
};

/** GMRF class implementing the probability map and methods for insterting new observations and update the map */
class CGMRF_map
{
public:
    CGMRF_map(rclcpp::Node *_node, const nav_msgs::msg::OccupancyGrid &oc_map, float cell_size, double m_lambdaPrior_reg,
              double m_lambdaPrior_mass_conservation, double m_lambdaPrior_obstacles, std::string m_colormap,
              bool verbose, bool filter_unexplored = true);
    ~CGMRF_map();

    // insert new observation
    void insertObservation_GMRF(double wind_speed, double wind_direction, double x_pos, double y_pos, double lambdaObs);

    // solves the minimum quadratic system to determine the new concentration of each cell
    void updateMapEstimation_GMRF(float lambdaObsLoss);

    // Visualization
    void get_as_markerArray(visualization_msgs::msg::MarkerArray &ma, std::string frame_id);

    WindVector getEstimation(int index);
    WindVector getEstimation(double x, double y);

    Eigen::Vector2i map_size()
    {
        return {m_size_x, m_size_y};
    }

    /**
     * Check if the occupancy map has changed dimensions or origin
     * @param new_map The new occupancy grid map
     * @return true if the map has changed and requires grid expansion/update
     */
    bool hasMapChanged(const nav_msgs::msg::OccupancyGrid &new_map) const;

    /**
     * Update the GMRF grid to accommodate a new occupancy map
     * This method handles dynamic map expansion during robot exploration
     * @param new_map The new occupancy grid map with potentially different dimensions
     * @return true if the update was successful
     */
    bool updateOccupancyMap(const nav_msgs::msg::OccupancyGrid &new_map);

    /**
     * Get all active observations for backup purposes
     * @return Vector of saved observations with world coordinates
     */
    std::vector<SavedObservation> getActiveObservations() const;

    /**
     * Restore observations from a saved backup
     * @param observations Vector of saved observations
     */
    void restoreObservations(const std::vector<SavedObservation> &observations);

    /**
     * Get the current map bounds
     */
    void getMapBounds(float &x_min, float &x_max, float &y_min, float &y_max) const
    {
        x_min = m_x_min;
        x_max = m_x_max;
        y_min = m_y_min;
        y_max = m_y_max;
    }

protected:
    rclcpp::Node *node;
    std::vector<TRandomFieldCell> m_map;      // GMRF container of nodes
    nav_msgs::msg::OccupancyGrid m_Ocgridmap; // Occupancy gridmap of the environment
    float m_x_min, m_x_max, m_y_min, m_y_max; // dimensions (m)
    float m_resolution;                       // cell_size (m)
    size_t m_size_x, m_size_y;                // dimensions in CellNumber
    size_t N;                                 // number of cells in the GMRF (we have 2N nodes)
    bool verbose;

    // GMRF
    size_t nPriorFactors;                 // Static/fixed factors
    size_t nObsFactors;                   // Dynamic factors due to observations
    size_t nFactors;                      // Total num of factors
    double lambdaPrior_reg;               // Weight for regularization prior -> neighbour cells have similar wind vectors
    double lambdaPrior_mass_conservation; // Weight for mass conservation law prior
    double lambdaPrior_obstacles;         // Weight for wind close to obstacles prior -->cells close to obstacles has only tangencial wind

    struct TobservationGMRF
    {
        size_t cell_idx;
        double windX;
        double windY;
        double lambda;
        bool time_invariant; // if the observation will lose weight (lambda) as time goes on (default false)
    };

    // GMRF structures
    std::vector<Eigen::Triplet<double>> J;      // the Jacobian
    std::vector<Eigen::Triplet<double>> Lambda; // the information matrix (weights)
    std::vector<TobservationGMRF> activeObs;    // Vector with the active observations and their respective Information

    // functions
    bool is_cell_free(size_t id_gmrf);
    bool is_cell_explored(size_t id_gmrf);
    bool check_connectivity_between2cells(size_t idx_1_gmrf, size_t idx_2_gmrf);

    // Filter unexplored regions flag
    bool filter_unexplored_;

    int xy2idx(float x, float y) const;

    void id2cellxy(size_t id, size_t &cell_x, size_t &cell_y);
    void id2xy(size_t id, double &x, double &y);

    /**
     * Build prior factors for the GMRF
     * This is called during initialization and after grid expansion
     * @param include_visualization If true, build visualization markers (line_list, line_list_obs)
     */
    void buildPriorFactors(bool include_visualization = false);

    /**
     * Initialize visualization markers for the factor graph
     */
    void initializeVisualizationMarkers();

    /**
     * Expand the GMRF grid to accommodate new map bounds
     * Preserves existing wind field estimates in overlapping regions
     * @param new_x_min New minimum x bound
     * @param new_x_max New maximum x bound
     * @param new_y_min New minimum y bound
     * @param new_y_max New maximum y bound
     * @return true if expansion was successful
     */
    bool expandGrid(float new_x_min, float new_x_max, float new_y_min, float new_y_max);

    /**
     * Map old cell index to new cell index after grid expansion
     * @param old_idx Index in the old grid
     * @param old_size_x Old grid width
     * @param old_x_min Old minimum x bound
     * @param old_y_min Old minimum y bound
     * @return New cell index, or -1 if mapping failed
     */
    int mapOldIdxToNewIdx(size_t old_idx, size_t old_size_x, float old_x_min, float old_y_min) const;

    // Visualization
    void save_grmf_factor_graph(std::vector<Eigen::Triplet<double>> &Jout, std::vector<Eigen::Triplet<double>> &Aout, Eigen::VectorXd &yout);
    void save_grmf_factor_graph(Eigen::SparseMatrix<double> &H, Eigen::VectorXd &G);
    visualization_msgs::msg::Marker line_list, line_list_obs;
    void init_colormaps(std::string colormap);
    float color_r[200];
    float color_g[200];
    float color_b[200];
};
