/** @mainpage ProAut OctoMap
 *
 * @section intro_sec Introduction
 *
 * This package was designed to automatically remove outdated voxels from the <a href="http://wiki.ros.org/octomap">octomap</a>.
 *
 * We described our motivation and concept on our <a href="https://www.tu-chemnitz.de/etit/proaut/octo">octomap-website</a> - here you will also find two supportive videos.
 * For further explanations, you may want to have a look at this <a href="http://nbn-resolving.de/urn:nbn:de:bsz:ch1-qucosa-226576">workshop abstract</a>.
 *
 * @section node_sec Node
 *
 * Our implementation:
 * @verbatim
rosrun octomap_pa octree_stamped_pa_node
@endverbatim
 *
 * The native implementation of the <a href="https://octomap.github.io">original octomap package</a>:
 * @verbatim
rosrun octomap_pa octree_stamped_native_node
@endverbatim
 *
 *
 * <b>Input and Output Topics:</b>
 *
 * Topic Name             | Type                                                                                                | Description
 * -----------------------|-----------------------------------------------------------------------------------------------------|---------------------------------
 * "~/in_cloud"           | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html">sensor_msgs/PointCloud2</a> | Input as <em>new</em> pointcloud type.
 * "~/in_cloud_old"       | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html">sensor_msgs/PointCloud</a>   | Input as <em>old</em> pointloud type. Will be converted to new pointcloud type.
 * "~/in_laser"           | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html">sensor_msgs/LaserScan</a>     | Input as single laser scan. Will be converted to new pointcloud type by package "laser_geometry".
 * "~/out_octomap"        | <a href="http://docs.ros.org/api/octomap_msgs/html/msg/Octomap.html">octomap_msgs/Octomap</a>       | Output of binary octomap - voxels are either free or occupied (smaller in size).
 * "~/out_octomap_full"   | <a href="http://docs.ros.org/api/octomap_msgs/html/msg/Octomap.html">octomap_msgs/Octomap</a>       | Output of octomap (full size).
 * "~/out_cloud_free"     | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html">sensor_msgs/PointCloud2</a> | Output of all free voxels as pointcloud.
 * "~/out_cloud_occupied" | <a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html">sensor_msgs/PointCloud2</a> | Output of all occupied voxels as pointcloud.
 *
 * All topics can be remapped using parameters (see below).
 *
 *
 * <b>Services:</b>
 *
 * Service Name       | Type                                                                                                                | Description
 * -------------------|---------------------------------------------------------------------------------------------------------------------|---------------------------------
 * "~/clear"          | std_srvs/Empty                                                                                                      | Deletes internal octomap.
 * "~/getsize"        | <a href="https://github.com/TUC-ProAut/ros_octomap/blob/master/srv/OctomapPaGetSize.srv">OctomapPaGetSize.srv</a>   | Returning number of nodes, total size in bytes and number of inserted measurments.
 * "~/save"           | <a href="https://github.com/TUC-ProAut/ros_octomap/blob/master/srv/OctomapPaFileName.srv">OctomapPaFileName.srv</a> | Storing the current octomap as file - timestamps are not saved.
 * "~/load"           | <a href="https://github.com/TUC-ProAut/ros_octomap/blob/master/srv/OctomapPaFileName.srv">OctomapPaFileName.srv</a> | Loading a octomap from file - timestamps are ignored.
 *
 *
 * <b>Parameters:</b>
 *
 * degrading of voxels
 * Parameter Name               | Type                 | Description
 * -----------------------------|----------------------|-------------------------------------
 * "~/degrading_time"           | double               | Duration how long the outdated nodes will be kept.
 * "~/auto_degrading"           | bool                 | Turns on automatic degrading.
 * "~/auto_degrading_intervall" | double               | Intervall for automatic degrading.
 *
 * pointcloud insertion
 * Parameter Name               | Type                 | Description
 * -----------------------------|----------------------|-------------------------------------
 * "~/map_prob_hit"             | double               | Probability that a positive measurement relates to a occupied voxel.
 * "~/map_prob_miss"            | double               | Probability that a negative measurement relates to a occupied voxel.
 * "~/pcd_voxel_active"         | bool                 | Use voxel-filter for pointcloud insertion.
 * "~/pcd_voxel_explicit"       | bool                 | Use pcl-filter instead of octomap-filter.
 * "~/pcd_voxel_explicit_relative_resolution" | double | Relative resolution of pcl-filter.
 *
 * octomap in general
 * Parameter Name               | Type                 | Description
 * -----------------------------|----------------------|-------------------------------------
 * "~/output_frame"             | string               | Coordinate system for insertion and output.
 * "~/map_resolution"           | double               | Side length of one voxel (in meters).
 * "~/map_prob_threshold"       | double               | Threshold for binary evaluation of single voxels.
 * "~/map_clamp_min"            | double               | Lower clamping value of occupancy probability.
 * "~/map_clamp_max"            | double               | Upper clamping value of occupancy probability.
 *
 * topics and services
 * Parameter Name               | Type                 | Description
 * -----------------------------|----------------------|-------------------------------------
 * "~/topic_in_cloud"           | string               | Name of input topic for new pointclouds.
 * "~/topic_in_cloud_old"       | string               | Name of input topic for old pointclouds.
 * "~/topic_in_laser"           | string               | Name of input topic for laser scans.
 * "~/topic_out_octomap"        | string               | Name of output topic for binary octomap.
 * "~/topic_out_octomap_full"   | string               | Name of output topic for full octomap.
 * "~/topic_out_cload_free"     | string               | Name of output topic for free voxels.
 * "~/topic_out_cloud_occupied" | string               | Name of output topic for occupied voxels.
 *
 *
 * See also
 * <a href="https://github.com/TUC-ProAut/ros_octomap/blob/master/config/parameter.yaml">this config file</a>.
 * It contains all parameters and their default value.
 *
 *
 * @section links_sec Links
 *
 * Source code at github:
 *  + https://github.com/TUC-ProAut/ros_octomap
 *
 * Related packages:
 *  + https://github.com/TUC-ProAut/ros_parameter
 *
 **/
