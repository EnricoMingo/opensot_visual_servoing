opensot_visual_servoing
-----------------------

Package containing tasks for visual servoing.

CartesI/O visual servoing support
---------------------------------
To run the visual servoing framework, CartesI/O needs another node called ```image_processing_node``` which provides visual features to the ```opensot_visual_servoing``` task:

![block scheme](docs/cartesio_visual_servoing.jpeg?raw=true "block scheme")


Python code
-----------
  
To run the node, lunch it in a terminal as

<code> ./image_processing_node image_raw:=image_topic_name camera_info:=camera_info_topic_name</code>

For example:

<code> ./image_processing_node image_raw:=/camera/rgb/image_raw camera_info:=/camera/rgb/camera_info </code>
