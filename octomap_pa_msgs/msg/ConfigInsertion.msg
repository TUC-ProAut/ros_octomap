## used for OctomapPaConfig.msg, OctomapPaGetConfig.srv
## & OctomapPaSetConfigInsertion.srv


## pointcloud insertion
# use pcl-transform instead of octomap-transform (default true)
bool pcd_explicit_transform
# use voxel-filter (default true)
bool pcd_voxel_active
# use pcl-filter instead of octomap-filter (default false)
bool pcd_voxel_explicit
# relative resolution of pcl-filter, if activated (default 0.5)
float32 pcd_voxel_explicit_relative_resolution

# probalitity that a positive measurement relates to a occupied voxel (default 0.7)
float32 map_prob_hit
# probalitity that a negative measurement relates to a occupied voxel (default 0.4)
float32 map_prob_miss

## octomap in general
# threshold for binary evaluation of single voxels (default 0.5)
float32 map_prob_threshold

# clamping values (default 0.12 & 0.97)
float32 map_clamp_min
float32 map_clamp_max
