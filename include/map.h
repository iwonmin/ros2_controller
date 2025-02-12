#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/sensor_msgs/msg/point_cloud2.h>
#include <optional>

typedef std::shared_ptr<octomap::OcTree> Octree_ptr;
typedef DynamicEDTOctomapBase<octomap::OcTree> DynamicEDTOctomap;
typedef std::shared_ptr<DynamicEDTOctomap> DynamicEDTMapPtr;
typedef octomap_msgs::msg::Octomap OctomapMsg;

class Map {
public:
  Map() = default;

  void update(const OctomapMsg& octomap_msg,
              const octomap::point3d& world_min,
              const octomap::point3d& world_max);

  [[nodiscard]] DynamicEDTMapPtr get_dist_map_ptr() const;

  void get_distance_and_closest_obstacle(const octomap::point3d& search_point,
                                                       float& distance,
                                                       octomap::point3d& closest_obstacle) const;

  [[nodiscard]] bool is_updated() const { return updated; }

  [[nodiscard]] double get_world_maxdist() const { return world_maxdist; }

  bool is_free(float x, float y) const;

private:
  Octree_ptr octree_ptr = nullptr;
  bool updated = false;
  double world_maxdist = 1.0;
  DynamicEDTMapPtr dynamic_edt_map_ptr = nullptr;
};