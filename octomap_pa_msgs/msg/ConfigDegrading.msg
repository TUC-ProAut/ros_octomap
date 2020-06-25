## used for OctomapPaConfig.msg, OctomapPaGetConfig.srv
## & OctomapPaSetConfigDegrading.srv


## degrading of voxels (only valid for degrading nodes)
# duration how long the outdated voxels will be kept (default 60 seconds)
float32 degrading_time
# turns on automatic degrading (default true)
bool auto_degrading
# intervall for automatic degrading (default 2.0 seconds)
float32 auto_degrading_intervall
