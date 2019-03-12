# IARC-Fuses

Fused Sensory Enhancement for IARC Mission 8.

## HUD Demo

![hud](figs/hud.gif)

# Guide to Running the Simulation

## Run Sim

```bash
rosrun iarc_fuses sphinx.sh
```

### Stopping

```bash
tmuxinator stop sphinx
```

## Recording

```bash
rosbag record /tf /tf_static /bebop/image_raw/compressed /bebop/camera_info
```

## Playback

```bash
rosparam set /use_sim_time true
rosbag play 2019-03-09-01-54-29.bag --clock --rate 1.0 --pause -k
rosrun image_transport republish compressed in:=bebop/image_raw raw out:=bebop/image_raw
```

## Visualizing the results

```bash
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib g2o_viewer -renameTypes VERTEX_SE3:EXPMAP=VERTEX_SE3:QUAT,VERTEX_XYZ=VERTEX_TRACKXYZ -typeslib /usr/local/lib/libg2o_types_sba.so -typeslib /usr/local/lib/libg2o_types_slam3d.so -typeslib /usr/local/lib/libg2o_types_data.so /tmp/ba.g2o
```
