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



bool Map::is_free(float x, float y) const {
  if (!dynamic_edt_map_ptr) return false;

  octomap::point3d query_point(x, y,0.15); 
  float distance;
  octomap::point3d closest_obstacle;

  dynamic_edt_map_ptr->getDistanceAndClosestObstacle(query_point, distance, closest_obstacle);

  return distance > 0.3f; 
}
// bool Map::cast_ray(const octomap::point3d& origin, 
//                    const octomap::point3d& direction, 
//                    octomap::point3d& hit_point) 
// {
//     if (!octree_ptr) {
//         return false;
//     }

//     octomap::point3d normalized_dir = direction - origin;
//     normalized_dir.normalize();

//     bool hit = octree_ptr->castRay(origin, normalized_dir, hit_point);
//     return hit; 
// }
