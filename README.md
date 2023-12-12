# how ros2 launch overrides duplicated parameters

## [description of ros2/launch_ros](https://github.com/ros2/launch_ros/blob/2bf4e6057dea57669c19395f6f39d390bd420ee7/launch_ros/launch_ros/actions/node.py#L175-L182)

> Multiple parameter dictionaries/files can be passed: each file path will be passed in in order to the node (where the last definition of a parameter takes effect).
  However, fully qualified node names override wildcards even when specified earlier. If `namespace` is not specified, dictionaries are prefixed by a wildcard namespace (`/**`) and other specific parameter declarations may overwrite it.

## The result

* `ros2 launch override_param override_param.launch.xml`

| node     | parameter name | actual stored value           | reason why the value is stored         |
|----------|----------------|-------------------------------|----------------------------------------|
| ns.node1 | before1        | param.yaml (wildcard)         | asigned most later                     |
| ns.node1 | before2        | launch                        | asigned only in launch.xml             |
| ns.node1 | after1         | param.yaml (wildcard)         | :x: I DONT KNOW WHY. i expect `launch`because launch is assigned most later. |
| ns.node1 | after2         | launch                        | asigned only in launch.xml             |
| ns.node2 | before1        | param.yaml  (wildcard)        | asigned only in param.yaml             |
| ns.node2 | before2        | default value defined in node | not asigned anywhere                   |
| ns.node2 | after1         | launch                        | asigned only in launch.xml             |
| ns.node2 | after2         | launch                        | asigned only in launch.xml             |

### Problem

As shown below, the difference between node1 and node2 is the presence of **before1/2** assignments in the launch file, but the **after1** value changes.

## Input files

### config.param.yaml

```yaml
/**:
  ros__parameters:
    before1: "param.yaml (wildcard)"
    after1: "param.yaml (wildcard)"
```

### override_param.launch.xml

The difference between node1 and node2 is whether `before1` and `before2` are asigned.

```xml
<launch>
  <arg name="param_file" default="$(find-pkg-share override_param)/config/config.param.yaml"/>
  <group>
    <push-ros-namespace namespace="ns"/>
    <node name="node1" pkg="override_param" exec="main" output="screen">
        <param name="before1" value="launch"/>
        <param name="before2" value="launch"/>
        <param from="$(var param_file)"/>
        <param name="after1" value="launch"/>
        <param name="after2" value="launch"/>
    </node>
    <node name="node2" pkg="override_param" exec="main" output="screen">
        <param from="$(var param_file)"/>
        <param name="after1" value="launch"/>
        <param name="after2" value="launch"/>
    </node>
  </group>
</launch>
```
