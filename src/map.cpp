#include "map.h"


void Map::update(const OctomapMsg& octomap_msg,
                 const octomap::point3d& world_min,
                 const octomap::point3d& world_max) {
  // Octomap Msg to Octree conversion
  octree_ptr.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));
  if (!octree_ptr) {
    throw std::runtime_error("Failed to convert OctomapMsg to OcTree.");
  }

  // Octree to DynamicEDTOctomap conversion
  dynamic_edt_map_ptr = std::make_shared<DynamicEDTOctomap>(
      world_maxdist, octree_ptr.get(), world_min, world_max, false);
  dynamic_edt_map_ptr->update();
  
  updated = true;
}

DynamicEDTMapPtr Map::get_dist_map_ptr() const {
  return dynamic_edt_map_ptr;
}

void Map::get_distance_and_closest_obstacle(const octomap::point3d& search_point,
                                            float& distance,
                                            octomap::point3d& closest_obstacle) const {
  if (!dynamic_edt_map_ptr) {
    distance = std::numeric_limits<float>::infinity();
    closest_obstacle = octomap::point3d(0,0,0);
    return;
  }
  dynamic_edt_map_ptr->getDistanceAndClosestObstacle(search_point, distance, closest_obstacle);
}

bool Map::is_free(double x, double y) const {
  if (!dynamic_edt_map_ptr) return false;

  octomap::point3d query_point(x, y, 0.0); 
  float distance;
  octomap::point3d closest_obstacle;

  dynamic_edt_map_ptr->getDistanceAndClosestObstacle(query_point, distance, closest_obstacle);

  return distance > 0.23f;
}

// bool Map::is_free(double wx, double wy) const {
//   if (!octree_ptr) {
//     return false;
//   }

//   double wz = 0.0;
//   octomap::OcTreeNode* node = octree_ptr->search(wx, wy, wz);
//   if (node == nullptr) {
//     return false;
//   }

//   double occupancy = node->getOccupancy();
//   return (occupancy < 0.2f);
// }
