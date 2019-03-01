#Header header

int64 object_id

float32 x_pc_centroid # center of mass of the primitive point cloud
float32 y_pc_centroid
float32 z_pc_centroid

string shape_tag		
float32[] coefficients	# depends on the primitive shape class

float32 x_est_centroid  # center of mass of the primitive as estimated by ransac
float32 y_est_centroid
float32 z_est_centroid
std_msgs/String color

