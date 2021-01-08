opensot_visual_servoing
-----------------------

Package containing tasks for visual servoing.

Python-code
-----------

The code contained in the <code>python</code> folder implements a ROS node, used to perfom the detection of four point features from a camera image. It is actually used to implement the image processing block, to provide a vision-based controller with visual point features.
  
To run the node, lunch it in a terminal as

<code> ./image_processing_node image_raw:=image_topci_anme camera_info:=camera_info_topic_name</code>

For example:

<code> ./image_processing_node image_raw:=/camera/rgb/image_raw camera_info:=/camera/rgb/camera_info </code>
