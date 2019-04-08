/*
 ~~ NOTES ~~
2019-03-10: It looks like the output is, in fact, in meters
*/

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"
#include "racecar_simulator/distance_transform.hpp"

using namespace racecar_simulator;

ScanSimulator2D::ScanSimulator2D(
    int    num_beams_           , 
    double field_of_view_       , 
    double offset_angle_        ,
    double scan_std_dev         , 
    double dist_min_            ,
    double dist_max_            ,
    double ray_tracing_epsilon_
) : num_beams( num_beams_ ) ,
    field_of_view( field_of_view_ ) ,
    offset_angle( offset_angle_ ) ,
    dist_min( dist_min_ ) ,
    dist_max( dist_max_ ) ,
    ray_tracing_epsilon( ray_tracing_epsilon_ ){
        
  // Initialize laser settings
  angle_increment = field_of_view / ( num_beams - 1 );

  // Initialize the output
  scan_output = std::vector<double>( num_beams );

  // Initialize the noise
  noise_generator = std::mt19937( std::random_device{}() );
  noise_dist = std::normal_distribution<double>( 0.0 , scan_std_dev );
}

double RS_saturation( double input , double loBound , double hiBound ){
    // Return 0 if the 'input' is out of bounds, Otherwise return the 'input'
    if( input < loBound )  return 0.0;
    // if( input > hiBound )  return 0.0;
    if( input > hiBound )  return hiBound;
    return input;
}

const std::vector<double> ScanSimulator2D::scan( const Pose2D& pose ){
    
  // Construct a pose for each beam
  Pose2D beam_pose = pose;
  //~ beam_pose.theta -= ( field_of_view/2.0 + offset_angle );
  beam_pose.theta -= ( field_of_view/2.0 );
  
  beam_pose.theta += offset_angle;

  // Sweep through each beam
  for (int i = 0; i < num_beams; i++) {
      
    // Compute the distance to the nearest point
    double distance = trace_ray(beam_pose);

    // Add Gaussian noise to the ray trace
    distance += noise_dist(noise_generator);

    // Add the distance to the output
    scan_output[i] = RS_saturation( distance , dist_min , dist_max );

    // Increment the scan
    beam_pose.theta += angle_increment;
  }

  return scan_output;
}

double ScanSimulator2D::distance_transform(const Pose2D & pose) const {
  // Convert the pose to a grid cell
  
  Pose2D beam_pose{ pose };
  
  //~ beam_pose.theta -= offset_angle;

  // Translate the state by the origin
  double x_trans = beam_pose.x - origin.x;
  double y_trans = beam_pose.y - origin.y;

  // Rotate the state into the map
  //~ double x_rot = x_trans * cos(-origin.theta + offset_angle ) - y_trans * sin(-origin.theta + offset_angle);
  //~ double y_rot = x_trans * sin(-origin.theta + offset_angle ) + y_trans * cos(-origin.theta + offset_angle);
  double x_rot = x_trans * cos( -origin.theta ) - y_trans * sin( -origin.theta );
  double y_rot = x_trans * sin( -origin.theta ) + y_trans * cos( -origin.theta );

  // Clip the state to be a cell
  if (x_rot < 0 or x_rot >= width * resolution) return 0;
  if (y_rot < 0 or y_rot >= height * resolution) return 0;

  // Discretize the state into row and column
  size_t col = std::floor(x_rot/resolution);
  size_t row = std::floor(y_rot/resolution);

  // Convert into a cell
  size_t cell = row * width + col;

  return dt[cell];
}

double ScanSimulator2D::trace_ray( const Pose2D& pose ) const {
  // Perform ray marching
  Pose2D p = pose;
  double distance_to_nearest = distance_transform(p);
  double total_distance = distance_to_nearest;

  while( distance_to_nearest > ray_tracing_epsilon ){
    // Move in the direction of the ray
    // by distanceToObstacle
    p.x += distance_to_nearest * std::cos( p.theta );
    p.y += distance_to_nearest * std::sin( p.theta );
    
    // Compute the nearest distance at that point
    distance_to_nearest = distance_transform(p);
    total_distance += distance_to_nearest;
  }

  return total_distance;
}

void ScanSimulator2D::set_map(
    const std::vector<double> & map, 
    size_t height_, 
    size_t width_, 
    double resolution_,
    const Pose2D & origin_,
    double free_threshold) {

  // Assign parameters
  height = height_;
  width = width_;
  resolution = resolution_;
  origin = origin_;

  // Threshold the map
  dt = std::vector<double>(map.size());
  for (size_t i = 0; i < map.size(); i++) {
    if (0 <= map[i] and map[i] <= free_threshold) {
      dt[i] = 99999; // Free
    } else {
      dt[i] = 0; // Occupied
    }
  }
  DistanceTransform::distance_2d(dt, width, height, resolution);
}
