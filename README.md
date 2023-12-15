# how ros2 launch overrides duplicated parameters

## [description of ros2/launch_ros](https://github.com/ros2/launch_ros/blob/2bf4e6057dea57669c19395f6f39d390bd420ee7/launch_ros/launch_ros/actions/node.py#L175-L182)

> Multiple parameter dictionaries/files can be passed: each file path will be passed in in order to the node (where the last definition of a parameter takes effect).
  However, fully qualified node names override wildcards even when specified earlier. If `namespace` is not specified, dictionaries are prefixed by a wildcard namespace (`/**`) and other specific parameter declarations may overwrite it.

## The result

* `ros2 launch override_param override_param.launch.xml`

| node  | parameter name | actual stored value |
|-------|----------------|---------------------|
| node1 | before1        | $HOME               |
| node1 | before2        | $HOME               |
| node1 | before1        | /home/kyabuuchi     |
| node1 | before2        | /home/kyabuuchi     |

## config.param.yaml

```yaml
/**:
  ros__parameters:
    before1: "$HOME"
    before2: $HOME
```

## override_param.launch.xml

```xml
<launch>
  <arg name="param_file" default="$(find-pkg-share override_param)/config/config.param.yaml"/>

  <node name="node1" pkg="override_param" exec="main" output="screen">
      <param from="$(var param_file)"/>
      <param name="after1" value="$(env HOME)"/>
      <param name="after2" value="$(env HOME)"/>
  </node>

</launch>
```
