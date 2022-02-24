/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/viam_draw_trajectories.h"

#include "cartographer/io/image.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {

cartographer::io::UniqueCairoSurfacePtr DrawTrajectoryNodes(const cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNode>& trajectory_nodes,//const cartographer::mapping::MapById<cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNodePose>& trajectory_node_poses,
                    float resolution, cartographer::transform::Rigid3d slice_pose,
                    const cartographer::io::FloatColor& color, cartographer::mapping::SubmapId submap_id,
                    cairo_surface_t* surface) {

  double kTrajectoryWidth = .75;
  double kTrajectoryEndMarkers = 4;
  constexpr double kAlpha = 1.;

  //auto cr = cartographer::io::MakeUniqueCairoPtr(cairo_create(surface));
  //auto cr = cartographer::io::MakeUniqueCairoSurfacePtr(cairo_create(surface));
  auto cr = cairo_create(surface);

  if (trajectory_nodes.size() == 0) {
    //return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr.get()));
    return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr));
    //return cr;
  }

  cairo_set_source_rgba(cr, color[0], color[1], color[2], kAlpha);
  cairo_set_line_width(cr, kTrajectoryWidth);

  // Draw trajectory path
  for (const auto& node : trajectory_nodes) {
    //if (node.id.trajectory_id != 0) {
    const auto t_global_pose = node.data.global_pose;
    const Eigen::Vector3d pixel = t_global_pose.translation();
    //std::cout << node.id << " | " << submap_id << "\n";
    
    
    if (submap_id.trajectory_id == node.id.trajectory_id && submap_id.submap_index == 0) {
     // std::cout << "MATCH!\n";

    double px =  (slice_pose.translation().y() - pixel.y())/resolution;
    double py = (slice_pose.translation().x() - pixel.x())/resolution;

    cairo_line_to(cr, px, py);
    }
    //}
  }
  cairo_stroke(cr);

  // Draw origin point
  double origin_x =  slice_pose.translation().y()/resolution;
  double origin_y = slice_pose.translation().x()/resolution;

  cairo_arc(cr, origin_x, origin_y, kTrajectoryEndMarkers, 0, 2 * M_PI);
  cairo_fill(cr);

  return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr));
  //return cartographer::io::MakeUniqueCairoSurfacePtr(cairo_get_target(cr.get()));
}

}  // namespace io
}  // namespace cartographer
