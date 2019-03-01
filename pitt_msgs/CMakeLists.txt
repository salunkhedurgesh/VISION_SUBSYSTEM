
cmake_minimum_required(VERSION 2.8.3)
project(pitt_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs
        message_generation #    genmsg
        message_runtime
        pcl_conversions
        pcl_ros
        sensor_msgs
        std_msgs
        pcl_msgs
        )

add_message_files(
        DIRECTORY msg
        FILES
        InliersCluster.msg
        Support.msg
        ClustersOutput.msg
        TrackedShape.msg  # contains the info of a tracked object
        TrackedShapes.msg # vector of trackedShape.msg
        Attribute.msg
        All_shapes_attributes.msg
)

add_service_files(DIRECTORY srv
        FILES
        ClusterSegmentation.srv
        DeepFilter.srv
        ArmFilter.srv
        PrimitiveSegmentation.srv
        SupportSegmentation.srv
	       ColorSrvMsg.srv

        )

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        pcl_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime )
