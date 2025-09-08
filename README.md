# Kungshu Hardware Arm


## Introduction

## Build

## Run

Give user privilege to access network devices:

```bash
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' </path/to/arm_node>
```

or run as root:

```bash
$ su
# ros2 run <package_name> <node_name>
```

Specify the nwork interface to use:

```bash
$ ros2 run <package_name> <node_name> --ros-args __ns:=/arm -p left:=<interface> --ros-args -p right:=<interface>
```