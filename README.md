# how ros2 launch overrides duplicated parameters

## [description of ros2/launch_ros](https://github.com/ros2/launch_ros/blob/2bf4e6057dea57669c19395f6f39d390bd420ee7/launch_ros/launch_ros/actions/node.py#L175-L182)

> Multiple parameter dictionaries/files can be passed: each file path will be passed in in order to the node (where the last definition of a parameter takes effect).
  However, fully qualified node names override wildcards even when specified earlier. If `namespace` is not specified, dictionaries are prefixed by a wildcard namespace (`/**`) and other specific parameter declarations may overwrite it.

## The result

* `ros2 launch override_param override_param.launch.xml`

| node     | parameter name | actual stored value    | reason why the value is stored                        |
|----------|----------------|------------------------|-------------------------------------------------------|
| node1    | before1        | param.yaml (wildcard)  | asigned most later                                    |
| node1    | before2        | param.yaml (node1)     | asigned by fully qualified node name in param.yaml    |
| node1    | after1         | launch                 | asigned most later                                    |
| node1    | after2         | param.yaml (node1)     | asigned by fully qualified node name in param.yaml    |
| ns.node2 | before1        | param.yaml (wildcard)  | asigned most later                                    |
| ns.node2 | before2        | param.yaml (ns/node2)  | asigned by fully qualified node name in param.yaml    |
| ns.node2 | after1         | param.yaml (wildcard)  | :x: I DONT KNOW WHY. i expect `launch`                |
| ns.node2 | after2         | param.yaml (ns/node2)  | asigned by fully qualified node name in param.yaml    |
| ns.node3 | before1        | param.yaml  (wildcard) | asigned most later                                    |
| ns.node3 | before2        | param.yaml (ns/node3)  | asigned by fully qualified node name in param.yaml    |
| ns.node3 | after1         | launch                 | asigned most later                                    |
| ns.node3 | after2         | launch                 | :x: I DONT KNOW WHY. i expect `param.yaml (ns/node3)` |

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
