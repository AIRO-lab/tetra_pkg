#include "virtual_costmap_layer/virtual_layer.hpp"

#include "nav2_costmap_2d/array_parser.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(virtual_costmap_layer::VirtualLayer, nav2_costmap_2d::Layer)

static const std::string tag {"[VIRTUAL-LAYER] "};

namespace virtual_costmap_layer {

// ---------------------------------------------------------------------

VirtualLayer::VirtualLayer(){}

// ---------------------------------------------------------------------

VirtualLayer::~VirtualLayer()
{
  dyn_params_handler_.reset();
}

// ---------------------------------------------------------------------

void VirtualLayer::onInitialize()
{
  current_ = true;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declareParameter("map_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("one_zone", rclcpp::ParameterValue(false));
  declareParameter("clear_obstacles", rclcpp::ParameterValue(true));
  
  node_->get_parameter("enabled", enabled_);
  node_->get_parameter("base_frame", _base_frame);
  node_->get_parameter("map_frame", _map_frame);
  node_->get_parameter("one_zone", _one_zone_mode);
  node_->get_parameter("clear_obstacles", _clear_obstacles);

  dyn_params_handler_ = node_->add_on_set_parameters_callback(std::bind(&VirtualLayer::dynamicParametersCallback, this, std::placeholders::_1));

  // save resolution
  _costmap_resolution = layered_costmap_->getCostmap()->getResolution();

  // set initial bounds
  _min_x = _min_y = _max_x = _max_y = 0;
  //TODO
  // reading the defined topics out of the namespace of this plugin!
  std::string zone_topics_name, obstacle_topics_name;

  declareParameter("zone_topics", rclcpp::ParameterValue(std::string("virtual_costamp_layer/zone")));
  declareParameter("obstacle_topics", rclcpp::ParameterValue(std::string("virtual_costamp_layer/obsctacles")));

  node_->get_parameter("zone_topics", zone_topics_name);
  node_->get_parameter("obstacle_topics", obstacle_topics_name);

  zone_sub_ = node_->create_subscription<tetra_msgs::msg::Zone>(
    zone_topics_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&VirtualLayer::zoneCallback, this, std::placeholders::_1));
  obstacles_sub_ = node_->create_subscription<tetra_msgs::msg::Obstacles>(
    obstacle_topics_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&VirtualLayer::obstaclesCallback, this, std::placeholders::_1));

  // reading the defined forms out of the namespace of this plugin!
  std::vector<double> forms_1;
  std::string forms_2, forms_3, forms_4;

  declareParameter("forms1", rclcpp::ParameterValue(std::vector<double>()));
  declareParameter("forms2", rclcpp::ParameterValue(std::string("[]")));
  declareParameter("forms3", rclcpp::ParameterValue(std::string("[]")));
  declareParameter("forms4", rclcpp::ParameterValue(std::string("[]")));

  node_->get_parameter("forms1", forms_1);
  node_->get_parameter("forms2", forms_2);
  node_->get_parameter("forms3", forms_3);
  node_->get_parameter("forms4", forms_4);

  geometry_msgs::msg::Point point;

  if(!forms_1.empty()){
    point.x = forms_1[0];
    point.y = forms_1[1];
    point.z = 0.0;
    _form_points.push_back(point);
  }

  if(forms_2 != "" && forms_2 != "[]"){
    parseFormListFromString(forms_2);
  }
  if(forms_3 != "" && forms_3 != "[]"){
    parseFormListFromString(forms_3);
  }
  if(forms_4 != "" && forms_4 != "[]"){
    parseFormListFromString(forms_4);
  }
  // compute map bounds for the current set of areas and obstacles.
  computeMapBounds();

  RCLCPP_INFO_STREAM(logger_, tag << "layer is initialized: [points: " << _form_points.size() << "] [polygons: " << _form_polygons.size() << "]");
}

bool VirtualLayer::parseFormListFromString(const std::string & form_string){
  std::string error;
  std::vector<std::vector<float>> vvf = nav2_costmap_2d::parseVVF(form_string, error);
  if (error != "") {
    RCLCPP_ERROR(logger_, "Error parsing footprint parameter: '%s'", error.c_str());
    RCLCPP_ERROR(logger_, "forms string was '%s'.", form_string.c_str());
    return false;
  }
  if(vvf.size() == 1){
    for (unsigned int i = 0; i < vvf.size(); i++) {
      if (vvf[i].size() == 2) {
        geometry_msgs::msg::Point point;
        point.x = vvf[i][0];
        point.y = vvf[i][1];
        point.z = 0;
        _form_points.push_back(point);
      } else {
        RCLCPP_ERROR(
          logger_,
          "Points in the form specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
          static_cast<int>(vvf[i].size()));
        return false;
      }
    }
  } else if(vvf.size() == 2) {
    Polygon vector_to_add;
    vector_to_add.reserve(3);
    geometry_msgs::msg::Point a;
    geometry_msgs::msg::Point b;
    if (vvf[0].size() == 2) {
      a.x = vvf[0][0];
      a.y = vvf[0][1];
      a.z = 0;
      vector_to_add.push_back(a);
    } else {
      RCLCPP_ERROR(
        logger_,
        "Points in the form specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
        static_cast<int>(vvf[0].size()));
      return false;
    }
    if (vvf[1].size() == 2) {
      b.x = vvf[1][0];
      b.y = vvf[1][1];
      b.z = 0;
      vector_to_add.push_back(b);
    } else {
      RCLCPP_ERROR(
        logger_,
        "Points in the form specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
        static_cast<int>(vvf[1].size()));
      return false;
    }
    // calculate the normal vector for AB
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point n;
    n.x = b.y - a.y;
    n.y = a.x - b.x;
    // get the absolute value of N to normalize it and to set the length of the costmap resolution
    double abs_n = sqrt(pow(n.x, 2) + pow(n.y, 2));
    n.x = n.x / abs_n * _costmap_resolution;
    n.y = n.y / abs_n * _costmap_resolution;
    // calculate the new points to get a polygon which can be filled
    point.x = a.x + n.x;
    point.y = a.y + n.y;
    vector_to_add.push_back(point);

    point.x = b.x + n.x;
    point.y = b.y + n.y;
    vector_to_add.push_back(point);

    _form_polygons.push_back(vector_to_add);
  } else {
    Polygon vector_to_add;
    vector_to_add.reserve(vvf.size());
    for (unsigned int i = 0; i < vvf.size(); i++) {
      if (vvf[i].size() == 2) {
        geometry_msgs::msg::Point point;
        point.x = vvf[i][0];
        point.y = vvf[i][1];
        point.z = 0;
        vector_to_add.push_back(point);
      } else {
        RCLCPP_ERROR(
          logger_,
          "Points in the form specification must be pairs of numbers. Found a point with %d numbers.", //NOLINT
          static_cast<int>(vvf[i].size()));
        return false;
      }
    }
    if (!vector_to_add.empty()) {
      _form_polygons.push_back(vector_to_add);
    }
  }
  return true;
}

// ---------------------------------------------------------------------

bool VirtualLayer::robotInZone(const Polygon &zone)
{
    if (!_one_zone_mode) {
        RCLCPP_WARN_STREAM(logger_, tag << "could be applied only for one_zone_mode");
        return true;
    }

    geometry_msgs::msg::Point point = getRobotPoint();
    std::size_t i, j;
    std::size_t size = zone.size();
    bool result = false;

    for (i = 0, j = size - 1; i < size; j = ++i) {
        if (((zone[i].y > point.y) != (zone[j].y > point.y)) &&
            (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y) / (zone[j].y - zone[i].y) + zone[i].x)) {
            result = !result;
        }
    }

    return result;
}

// ---------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult VirtualLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name == "zone_topics" ||
      param_name == "obstacle_topics" ||
      param_name == "forms1" ||
      param_name == "forms2" ||
      param_name == "forms3" ||
      param_name == "forms4")
    {
      RCLCPP_WARN(
        logger_, "%s is not a dynamic parameter "
        "cannot be changed while running. Rejecting parameter update.", param_name.c_str());
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      if (param_name == "map_frame") {
        _map_frame = parameter.as_string();
      } else if (param_name == "base_frame") {
        _base_frame = parameter.as_string();
      }
    } else if (param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (param_name == "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
      } else if (param_name == "one_zone" && _one_zone_mode != parameter.as_bool()) {
        _one_zone_mode = parameter.as_bool();
      } else if (param_name == "clear_obstacles" && _clear_obstacles != parameter.as_bool()) {
        _clear_obstacles = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

void VirtualLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  if (_obstacle_points.empty() && _zone_polygons.empty() && _obstacle_polygons.empty()) {
    return;
  }

  *min_x = std::min(*min_x, _min_x);
  *min_y = std::min(*min_y, _min_y);
  *max_x = std::max(*max_x, _max_x);
  *max_y = std::max(*max_y, _max_y);
}

void VirtualLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // set costs of zone polygons
  /*
  for (int i = 0; i < _zone_polygons.size(); ++i) {
      setPolygonCost(master_grid, _zone_polygons[i], nav2_costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, false);
  }
  */

  // set costs of obstacle polygons
  for (int i = 0; i < _obstacle_polygons.size(); ++i) {
    setPolygonCost(master_grid, _obstacle_polygons[i], nav2_costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
  }

  // set cost of obstacle points
  for (int i = 0; i < _obstacle_points.size(); ++i) {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(_obstacle_points[i].x, _obstacle_points[i].y, mx, my)) {
      master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void VirtualLayer::computeMapBounds()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  // reset bounds
  _min_x = _min_y = _max_x = _max_y = 0;

  // iterate zone polygons
  for (int i = 0; i < _zone_polygons.size(); ++i) {
    for (int j = 0; j < _zone_polygons.at(i).size(); ++j) {
      double px = _zone_polygons.at(i).at(j).x;
      double py = _zone_polygons.at(i).at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate obstacle polygons
  for (int i = 0; i < _obstacle_polygons.size(); ++i) {
    for (int j = 0; j < _obstacle_polygons.at(i).size(); ++j) {
      double px = _obstacle_polygons.at(i).at(j).x;
      double py = _obstacle_polygons.at(i).at(j).y;
      _min_x = std::min(px, _min_x);
      _min_y = std::min(py, _min_y);
      _max_x = std::max(px, _max_x);
      _max_y = std::max(py, _max_y);
    }
  }

  // iterate obstacle points
  for (int i = 0; i < _obstacle_points.size(); ++i) {
    double px = _obstacle_points.at(i).x;
    double py = _obstacle_points.at(i).y;
    _min_x = std::min(px, _min_x);
    _min_y = std::min(py, _min_y);
    _max_x = std::max(px, _max_x);
    _max_y = std::max(py, _max_y);
  }
}

void VirtualLayer::setPolygonCost(nav2_costmap_2d::Costmap2D &master_grid, const Polygon &polygon, unsigned char cost,
                                  int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
{
  std::vector<PointInt> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    PointInt loc;
    master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
    map_polygon.push_back(loc);
  }

  std::vector<PointInt> polygon_cells;

  // get the cells that fill the polygon
  rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    int mx = polygon_cells[i].x;
    int my = polygon_cells[i].y;
    // check if point is outside bounds
    if (mx < min_i || mx >= max_i)
      continue;
    if (my < min_j || my >= max_j)
      continue;
    master_grid.setCost(mx, my, cost);
  }
}

void VirtualLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
{
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
  }
  if (!polygon.empty()) {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
  }
}

void VirtualLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
{
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  PointInt pt;
  pt.x = x0;
  pt.y = y0;
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    cells.push_back(pt);

    if (error > 0) {
      pt.x += x_inc;
      error -= dy;
    } else {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void VirtualLayer::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
{
  // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

  //we need a minimum polygon of a traingle
  if (polygon.size() < 3)
    return;

  //first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  if (!fill)
    return;

  //quick bubble sort to sort points by x
  PointInt swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    } else
      ++i;
  }

  i = 0;
  PointInt min_pt;
  PointInt max_pt;
  int min_x = polygon_cells[0].x;
  int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

  //walk through each column and mark cells inside the polygon
  for (int x = min_x; x <= max_x; ++x) {
    if (i >= (int)polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    PointInt pt;
    //loop though cells in the column
    for (int y = min_pt.y; y < max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

void VirtualLayer::zoneCallback(const tetra_msgs::msg::Zone::SharedPtr zone_msg)
{
  if (zone_msg->area.form.size() > 2) {
    Polygon vector_to_add;
    for (int i = 0; i < zone_msg->area.form.size(); ++i) {
      vector_to_add.push_back(zone_msg->area.form[i]);
    }

    if (!robotInZone(vector_to_add)) {
      RCLCPP_WARN_STREAM(logger_, tag << "Robot point is not the navigation zone");
      return;
    }

    if (_one_zone_mode) {
      _zone_polygons.clear();
    }
    _zone_polygons.push_back(vector_to_add);

    computeMapBounds();
  } else {
    RCLCPP_ERROR_STREAM(logger_, tag << "A zone Layer needs to be a polygon with minimun 3 edges");
  }
}

void VirtualLayer::obstaclesCallback(const tetra_msgs::msg::Obstacles::SharedPtr obstacles_msg)
{
  if (_clear_obstacles) {
    _obstacle_polygons.clear();
    _obstacle_points.clear();
  }

  for (int i = 0; i < obstacles_msg->list.size(); ++i) {
    Polygon vector_to_add;
    if (obstacles_msg->list[i].form.size() == 1) {
      if (obstacles_msg->list[i].form[0].z == 0.0) {
        // ROS_INFO_STREAM(tag << "Adding a Point");
        _obstacle_points.push_back(obstacles_msg->list[i].form[0]);
      } else if (obstacles_msg->list[i].form[0].z > 0.0) {
        // ROS_INFO_STREAM(tag << "Adding a Circle");
        // Loop over 36 angles around a circle making a point each time
        int N = 36;
        geometry_msgs::msg::Point pt;
        for (int j = 0; j < N; ++j) {
          double angle = j * 2 * M_PI / N;
          pt.x = obstacles_msg->list[i].form[0].x + cos(angle) * obstacles_msg->list[i].form[0].z;
          pt.y = obstacles_msg->list[i].form[0].y + sin(angle) * obstacles_msg->list[i].form[0].z;
          vector_to_add.push_back(pt);
        }
        _obstacle_polygons.push_back(vector_to_add);
      }
    } else if (obstacles_msg->list[i].form.size() == 2) {
      // ROS_INFO_STREAM(tag << "Adding a Line");

      geometry_msgs::msg::Point point_A = obstacles_msg->list[i].form[0];
      geometry_msgs::msg::Point point_B = obstacles_msg->list[i].form[1];
      vector_to_add.push_back(point_A);
      vector_to_add.push_back(point_B);

      // calculate the normal vector for AB
      geometry_msgs::msg::Point point_N;
      point_N.x = point_B.y - point_A.y;
      point_N.y = point_A.x - point_B.x;

      // get the absolute value of N to normalize and get
      // it to the length of the costmap resolution
      double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
      point_N.x = point_N.x / abs_N * _costmap_resolution;
      point_N.y = point_N.y / abs_N * _costmap_resolution;

      // calculate the new points to get a polygon which can be filled
      geometry_msgs::msg::Point point;
      point.x = point_A.x + point_N.x;
      point.y = point_A.y + point_N.y;
      vector_to_add.push_back(point);

      point.x = point_B.x + point_N.x;
      point.y = point_B.y + point_N.y;
      vector_to_add.push_back(point);

      _obstacle_polygons.push_back(vector_to_add);
    } else {
      // ROS_INFO_STREAM(tag << "Adding a Polygon");
      for (int j = 0; j < obstacles_msg->list[i].form.size(); ++j) {
        vector_to_add.push_back(obstacles_msg->list[i].form[j]);
      }
      _obstacle_polygons.push_back(vector_to_add);
    }
  }
  computeMapBounds();
}

geometry_msgs::msg::Point VirtualLayer::getRobotPoint()
{
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::PoseStamped current_robot_pose, current_robot_pose_base;
  geometry_msgs::msg::Point robot_point;
  geometry_msgs::msg::TransformStamped current_transform_msg;
  try {
    rclcpp::Time now = node_->get_clock()->now();
    current_robot_pose_base.header.stamp = now;
    current_robot_pose_base.header.frame_id = _base_frame;
    current_robot_pose_base.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());

    current_transform_msg = tf_buffer_->lookupTransform(_map_frame, current_robot_pose_base.header.frame_id, current_robot_pose_base.header.stamp);
    tf2::Transform t;
    tf2::fromMsg(current_transform_msg.transform, t);
    tf2::Vector3 markerOrigin(0, 0, 0);
    tf2::Transform m(tf2::Quaternion::getIdentity(), markerOrigin);
    tf2::Transform pose_transform = t * m;
    current_robot_pose.pose.position.x = pose_transform.getOrigin().getX();
    current_robot_pose.pose.position.y = pose_transform.getOrigin().getY();
    robot_point.x = current_robot_pose.pose.position.x;
    robot_point.y = current_robot_pose.pose.position.y;
    robot_point.z = 0.0;
  } catch (tf2::TransformException &ex) {
    RCLCPP_DEBUG_STREAM(logger_, tag << "Can't get robot pose: " << ex.what());
  }
  return robot_point;
}

} // namespace virtual_costmap_layer