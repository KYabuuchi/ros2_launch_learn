# how ros2 launch overrides duplicated parameters


## config.param.yaml

```yaml
/**:
  ros__parameters:
    before1: "param.yaml (wildcard)"
    before2: "param.yaml (wildcard)"
    after1: "param.yaml (wildcard)"
    after2: "param.yaml (wildcard)"

node1:
  ros__parameters:
    before2: "param.yaml (node1)"
    after2: "param.yaml (node1)"

ns/node2:
  ros__parameters:
    before2: "param.yaml (ns/node2)"
    after2:  "param.yaml (ns/node2)"

ns/node3:
  ros__parameters:
    before2: "param.yaml (ns/node3)"
    after2:  "param.yaml (ns/node3)"

```

## override_param.launch.xml

```xml
<launch>
  <arg name="param_file" default="$(find-pkg-share override_param)/config/config.param.yaml"/>
  <node name="node1" pkg="override_param" exec="main" output="screen">
      <param name="before1" value="launch"/>
      <param name="before2" value="launch"/>
      <param from="$(var param_file)"/>
      <param name="after1" value="launch"/>
      <param name="after2" value="launch"/>
  </node>
  <group>
    <push-ros-namespace namespace="ns"/>
    <node name="node2" pkg="override_param" exec="main" output="screen">
        <param name="before1" value="launch"/>
        <param name="before2" value="launch"/>
        <param from="$(var param_file)"/>
        <param name="after1" value="launch"/>
        <param name="after2" value="launch"/>
    </node>
    <node name="node3" pkg="override_param" exec="main" output="screen">
        <param from="$(var param_file)"/>
        <param name="after1" value="launch"/>
        <param name="after2" value="launch"/>
    </node>
  </group>
</launch>
```

## [description of ros2/launch_ros](https://github.com/ros2/launch_ros/blob/2bf4e6057dea57669c19395f6f39d390bd420ee7/launch_ros/launch_ros/actions/node.py#L175-L182)


## The result

* `ros2 launch override_param override_param.launch.xml`
* `ros2 launch override_param override_param.launch.py`

| node     | parameter name | value                  | reason                                                          |
|----------|----------------|------------------------|-----------------------------------------------------------------|
| node1    | before1        | param.yaml (wildcard)  | asigned most later                                              |
| node1    | before2        | param.yaml (node1)     | fully qualified node name in param.yaml                         |
| node1    | after1         | launch                 | asigned most later                                              |
| node1    | after2         | param.yaml (node1)     | fully qualified node name in param.yaml                         |
| ns.node2 | before1        | param.yaml (wildcard)  | asigned most later                                              |
| ns.node2 | before2        | param.yaml (ns/node2)  | fully qualified node name in param.yaml                         |
| ns.node2 | after1         | param.yaml (wildcard)  | :fire: I DONT KNOW WHY. I EXPECT `launch` :fire:                |
| ns.node2 | after2         | param.yaml (ns/node2)  | fully qualified node name in param.yaml                         |
| ns.node3 | before1        | param.yaml  (wildcard) | asigned most later                                              |
| ns.node3 | before2        | param.yaml (ns/node3)  | fully qualified node name in param.yaml                         |
| ns.node3 | after1         | launch                 | asigned most later                                              |
| ns.node3 | after2         | launch                 | :fire: I DONT KNOW WHY. I EXPECT `param.yaml (ns/node3)` :fire: |
