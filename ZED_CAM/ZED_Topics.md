/zed2/zed_node/imu/data                     sensor_msgs/Imu             

/zed2/zed_node/right_raw/camera_info        sensor_msgs/CameraInfo

/zed2/zed_node/rgb_raw/image_raw_color      sensor_msgs/Image

/zed2/zed_node/disparity/disparity_image    stereo_msgs/DisparityImage              DisparityImage for camera, color means the distance to the camera

/zed2/zed_node/depth/depth_registered       sensor_msgs/Image                       Image for depth graph 

/zed2/zed_node/point_cloud/cloud_registered sensor_msgs/PointCloud2                 Point cloud for camera

For object detectation, we need to enable some paramater. See section 4.8 in the following website
http://wiki.ros.org/zed-ros-wrapper For future improvement(if we need more topices), here is the reference page.