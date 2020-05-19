# MRS extras resources for Gazebo

Resources for Gazebo/ROS simulator that depends on packages (e.g. mrs_msgs, mrs_lib) inside of the MRS system.

## Link attacher

Modified version from the `pal-robotics:` [gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher "Github page")

### Description
- World plugin that can attach or detach two Gazebo models with a virtual joint.
- The plugin is controlled by five ros services:
  * /link_attacher_node/apply_force
  * /link_attacher_node/attach
  * /link_attacher_node/attach_typed
  * /link_attacher_node/delete
  * /link_attacher_node/detach

### Usage
After building, activate by adding the following to your `world` definition.
```xml
  ...
  <world name="...">
  ...
    <plugin name="mrs_gazebo_link_attacher_plugin" filename="libMRSGazeboLinkAttacherPlugin.so"/>
  ...
```

### Example
Imagine that the world contains two object - `unit_box_1` and `unit_sphere_1`, where each of them has one link named  `link`.
Then their attaching is done by calling:

```bash
rosservice call /link_attacher_node/attach "model_name_1: 'unit_box_1'
link_name_1: 'link'
model_name_2: 'unit_sphere_1'
link_name_2: 'link'"
```

And similar thing for detaching:

```bash
rosservice call /link_attacher_node/detach "model_name_1: 'unit_box_1'
link_name_1: 'link'
model_name_2: 'unit_sphere_1'
link_name_2: 'link'"
```

## Thermal camera plugin

Modified version from the `hector_gazebo:` [hector_gazebo_thermal_camera](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/kinetic-devel/hector_gazebo_thermal_camera "Github page")

### Description
- Simulates a thermal camera sensor in Gazebo.
- Objects that can be detected by the thermo camera have to have specific visual properties. See example [fire_mockup](https://github.com/ctu-mrs/mrs_gazebo_extras_resources/blob/master/models/fire_mockup/model.sdf#L14-L18).
- The original plugin has been extended to publish tranform message on topic `/tf_gazebo_static`. To make this transformation visible in ROS use our [Static transform republisher plugin](../world_plugins/README.md#static-transform-republisher-plugin) in your `world` definition. 

### Usage
After building, activate by adding the following to your robot definition.

```xml
  ...
    <gazebo reference="${namespace}/${camera_frame_name}/thermal_camera_link">
      <sensor type="camera" name="mrs_thermal_camera_sensor">
        <update_rate>${frame_rate}</update_rate>
        <camera>
          <horizontal_fov>${hfov}</horizontal_fov>
          <image>
            <format>R8G8B8</format>
            <width>${3 * image_width}</width>
            <height>${3 * image_height}</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>300</far>
          </clip>
        </camera>
        <plugin name="thermal_camera_controller" filename="libMRSGazeboThermalCameraPlugin.so">
          <alwaysOn>true</alwaysOn>
          <cameraName>${camera_suffix}</cameraName>
          <updateRate>${frame_rate}</updateRate>
          <imageTopicName>${camera_topic_name}/rgb_image</imageTopicName>
          <cameraInfoTopicName>${camera_topic_name}/camera_info</cameraInfoTopicName>
          <rawTemperatureTopicName>${camera_topic_name}/raw_temp_array</rawTemperatureTopicName>
          <surroundingTemperature>20</surroundingTemperature>
          <maximalTemperature>150</maximalTemperature>
          <minimalTemperatureGreenColor>0.2</minimalTemperatureGreenColor>
          <noiseStdDev>4.0</noiseStdDev>
          <noiseStdDevMaxTemp>20.0</noiseStdDevMaxTemp>
          <frameName>${camera_frame_name}</frameName>
          <parentFrameName>${parent_frame_name}</parentFrameName>
          <sensorBaseFrameName>${sensor_base_frame_name}</sensorBaseFrameName>
          <x>${x}</x>
          <y>${y}</y>
          <z>${z}</z>
          <roll>${roll}</roll>
          <pitch>${pitch}</pitch>
          <yaw>${yaw}</yaw>
        </plugin>
      </sensor>
    </gazebo>
  ...
```
