// Generated from the stage9 GUI-recorded R2 command; no YAML reimplementation.
path_manager::MissionPlanRequest recordedRequest(double unit_m) {
  path_manager::MissionPlanRequest r;
  r.head.src = path_manager::StartStateSource::STATED_SPEED;
  r.start_altitude = path_manager::StartAltitude::COMMAND_AGL;
  r.head.pos_u = Eigen::Vector3d(14724.6468,18218.68,2);
  r.waypoints = {Eigen::Vector3d(15165.8971,17382.9601,2)};
  Eigen::Vector3d first_leg = r.waypoints.front() - r.head.pos_u;
  first_leg.z()=0.0;
  first_leg.normalize();
  r.head.vel_u = first_leg * (200/unit_m);
  r.formation_type = "none";
  r.formation_pattern = {Eigen::Vector3d(0,0,0)};
  r.formation_size = 1;
  return r;
}
