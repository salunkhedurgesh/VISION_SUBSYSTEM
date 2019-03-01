#############################################
# This message is use to describe a support and it is created by the support segmentation service through the message "SupportSegmentatio.srv"
# 
# Particularly, it describes a support through the index of the points that are belonging to the support with respect to the
# original point cloud given to the SupportSegmentation.srv. Moreover, it contains also a cloud with only the points that are 
# related to the support itself as well as the cloud the contains the points of the objects that are on top of the support
# (for more information on how this is computed see the SupportSegmentation service).
# Finally, it returns also the parametric estimation of the supports given by RANSAC through 4 coefficients (a,b,c,d) that 
# describes the plane as: (aX + bY + cZ + d = 0).  
#############################################

int32[] inliers								# the index with respect of the input cloud to the service of the points that are belonging to a support

sensor_msgs/PointCloud2 support_cloud 		# the point cloud containing only the points that are belonging to one support (extrapolation of the inliers of the original cloud)

sensor_msgs/PointCloud2 on_support_cloud 	# the point of the area above the cloud 

float32 support_coefficient_a
float32 support_coefficient_b
float32 support_coefficient_c
float32 support_coefficient_d