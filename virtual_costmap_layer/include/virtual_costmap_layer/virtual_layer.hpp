#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tetra_msgs/msg/obstacles.hpp"
#include "tetra_msgs/msg/zone.hpp"

namespace virtual_costmap_layer {

struct PointInt {
    int x;
    int y;
};

class VirtualLayer : public nav2_costmap_2d::CostmapLayer {
public:
    using Polygon = std::vector<geometry_msgs::msg::Point>;
    VirtualLayer();
    virtual ~VirtualLayer();

    //// \brief function which get called at initialization of the costmap
    ///        it defines the reconfigure callback, gets params from server and subscribes to topics
    virtual void onInitialize();

    //// \brief this is called by the LayeredCostmap to poll this plugin as to how much of the costmap it needs to update.
    ///        each layer can increase the size of this bounds.
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double *min_x, double *min_y, double *max_x, double *max_y);

    //// \brief  function which get called at every cost updating procdure of the overlayed costmap. The before readed costs will get filled
    virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void reset();
protected:
    ////  \brief dynamic reconfiguration callback
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    //// \brief computes bounds in world coordinates for the current set of points and polygons.
    ///        the result is stored in class members _min_x, _min_y, _max_x and _max_y.
    void computeMapBounds();

    /// \brief                set cost in a Costmap2D for a polygon (polygon may be located outside bounds)
    /// \param master_grid    reference to the Costmap2D object
    /// \param polygon        polygon defined by a vector of points (in world coordinates)
    /// \param cost           the cost value to be set (0,255)
    /// \param min_i          minimum bound on the horizontal map index/coordinate
    /// \param min_j          minimum bound on the vertical map index/coordinate
    /// \param max_i          maximum bound on the horizontal map index/coordinate
    /// \param max_j          maximum bound on the vertical map index/coordinate
    /// \param fill_polygon   if true, tue cost for the interior of the polygon will be set as well
    void setPolygonCost(nav2_costmap_2d::Costmap2D &master_grid, const Polygon &polygon,
                        unsigned char cost, int min_i, int min_j, int max_i, int max_j, bool fill_polygon);

    /// \brief                     converts polygon (in map coordinates) to a set of cells in the map
    /// \note                      This method is mainly based on Costmap2D::convexFillCells() but accounts for a self - implemented polygonOutlineCells() method and allows negative map coordinates
    /// \param polygon             Polygon defined  by a vector of map coordinates
    /// \param fill                If true, the interior of the polygon will be considered as well
    /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
    void rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill);

    /// \brief                     extracts the boundary of a polygon in terms of map cells
    /// \note                      this method is based on Costmap2D::polygonOutlineCells() but accounts for a self - implemented raytrace algorithm and allows negative map coordinates
    /// \param polygon             polygon defined  by a vector of map coordinates
    /// \param[out] polygon_cells  new cells in map coordinates are pushed back on this container
    void polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells);

    /// \brief             rasterizes line between two map coordinates into a set of cells
    /// \note              since Costmap2D::raytraceLine() is based on the size_x and since we want to rasterize polygons that might also be located outside map bounds we provide a modified raytrace
    ///                    implementation(also Bresenham) based on the integer version presented here : http : //playtechs.blogspot.de/2007/03/raytracing-on-grid.html
    /// \param x0          line start x-coordinate (map frame)
    /// \param y0          line start y-coordinate (map frame)
    /// \param x1          line end x-coordinate (map frame)
    /// \param y1          line end y-coordinate (map frame)
    /// \param[out] cells  new cells in map coordinates are pushed back on this container
    void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells);

    bool parseFormListFromString(const std::string & form_string);

    /// \brief zone callback function
    void zoneCallback(const tetra_msgs::msg::Zone::SharedPtr msg);

    /// \brief obstacle callback function
    void obstaclesCallback(const tetra_msgs::msg::Obstacles::SharedPtr obstacles_msg);

    /// \brief                checks if the robot point is in the polygon defining the zone
    /// \note                 works only for one zone otherwise returns true
    /// \param zone           polygon zone
    /// \return bool true     if the robot is in the zone
    bool robotInZone(const Polygon &zone);

    /// \brief gets a current geometry_msgs::msg::Point of the robot
    /// \return geometry_msgs::msg::Point current pose
    geometry_msgs::msg::Point getRobotPoint();

    rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_; // dynamic_reconfigure server for the costmap
    // std::mutex _data_mutex;                                                 // mutex for the accessing forms
    double _costmap_resolution;                                             // resolution of the overlayed costmap to create the thinnest line out of two points
    bool _enabled, _one_zone_mode, _clear_obstacles;                                  // put in memory previous zones and obstacles if false
    std::string _base_frame;                                                // base frame of the robot by default "base_link"
    std::string _map_frame;                                                 // map frame by default "map"
    std::vector<geometry_msgs::msg::Point> _obstacle_points;                     // vector to save the obstacle points in source coordinates
    std::vector<Polygon> _zone_polygons;                                    // vector to save the zone polygons (more than 3 edges) in source coordinates
    std::vector<Polygon> _obstacle_polygons;                                // vector to save the obstacle polygons (including lines) in source coordinates
    std::vector<Polygon> _form_polygons;                                    // vector to save the form polygons (including lines) in source coordinates
    std::vector<geometry_msgs::msg::Point> _form_points;                         // vector to save the form points in source coordinates
    rclcpp::Subscription<tetra_msgs::msg::Obstacles>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<tetra_msgs::msg::Zone>::SharedPtr zone_sub_;

    double _min_x, _min_y, _max_x, _max_y; // cached map bounds

};

} // namespace virtual_costmap_layer
