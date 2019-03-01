# Human-like Memory for Robots - **Vision Module**

## Objective of the Project

The Vision part of the Human-like Memory for Robots project is designed to acquire data from a scene under the Baxter using PITT, filter and optimize these data, and then pass the data to the Reasoner part.

## The System's Architecture

### Overall Architecture

This repository has two main nodes: A node called VisNode, and a client node called VisClient.
The object is to filter the raw data from RANSAC, and then pass the proper results to the ARMOR in order to add scenes to the ontology.
**Data structure:**
* object id
* shape
* color
* radius
* height
* centroid
* (optional) axis

### Description of the Nodes

VisNode subscribes to the topic /ransac_segmentation/trackedShapes and publishes to the topic /scene_data.
VisClient is a client to the ARMOR service, and also a subscriber to the topic /scene_data.

| Name | Description | Input | Output |
| ------| -----------| ---- | ---- |
| *VisNode* | Ransac segmentation publishes the information of current scene. VisNode collects these data, calculates the average attributes and optimized shapes. | /ransac_segmentation/trackedShapes | /scene_data 
| *VisClient* | VisClient does not manipulate the data. It executes a passing-by process. | /scene_data | ARMOR server


## Implementation

### Prerequisites

* Ubuntu 16.04
* ROS Kinetic
* Vision sensor (Kinect)
* [ARMOR](https://github.com/EmaroLab/armor) package
    - [ARMOR msgs](https://github.com/EmaroLab/armor_msgs)
* [Primitive identification tracking tagging](https://github.com/EmaroLab/primitive_identification_tracking_tagging) package
    - [PITT object table segmentation](https://github.com/salunkhedurgesh/VISION_SUBSYSTEM/tree/master/pitt_object_table_segmentation) 
    - [PITT msgs](https://github.com/salunkhedurgesh/VISION_SUBSYSTEM/tree/master/pitt_msgs)
    - [PITT geometry tracking](https://github.com/EmaroLab/pitt_geometric_tracking/tree/5e38571e5f30a84aaadabb4d1dc0d9e269460ae0)

### Updated files
In pitt_object_table_segmentation:
* VisNode.cpp
* VisClient.cpp

In pitt_msgs:
* Attribute.msg
* All_shapes_attributes.msg

### How to run the project

* Make sure all the prerequisites are set up
* Source the workspace
* Build the workspace
    ```
    catkin_make
    ```
* Launch the object table segmentation
    ``` 
    roslaunch pitt_object_table_segmentation table_segmentation.launch
    ```
* Check the camera
    ```
    rostopic list
    ```
* Run VisNode
    ```
    rosrun pitt_object_table_segmentation VisNode
    ```
* Run VisClient
    ```
    rosrun pitt_object_table_segmentation VisClient
    ```
* (optional) Read the data
    ```
    rostopic echo /ransac_segmentation
    ```
    and / or
    ```
    rostopic echo /scene_data
    ```

## Results
The average attributes given by VisNode are computed taking into account the different shapes, this way the values published on /scene_data will be as stable as possible. The publish operation is executed with a buffer of dimension 1 such that only the updated information about the current scene is given to the subscribers.
The module delivers a list of properties to the Encoding module of the architecture and the whole scene (composed by objects represented by those properties) will be loaded in the ontology.

## Recommendations

* The detectable colors are: Red, Green, Blue, Yellow, Pink. 
    This is due to the color service in PITT is using the HSV (Hue, Saturation, Value) color model. 
    If the scene contains objects of other colors, the use of color service must be commented, or there will be errors in executing.
* After launch the table segmentation, there will appear some red lines in the terminal. This is not errors but a normal beginning output of RANSAC.
* The VisNode is for one scene only. VisNode has to be re-initialized if the scene changes.
    This is due to that the PITT is detecting the same scene as different scene. This limit can be solved along the developing of PITT.

## Authors

* Alessandra Ventura: [aleventura96@gmail.com](mailto:aleventura96@gmail.com)
* Durgesh Salunkhe: [salunkhedurgesh@gmail.com](mailto:salunkhedurgesh@gmail.com)
* Zijian Liu: [liuzijian42@gmail.com](mailto:liuzijian42@gmail.com)
