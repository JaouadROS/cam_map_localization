# cam_map_localization
cam_map_localization is a ros-package using a predefined map of fiducial markers to localize the camera with respect to a marker set as origin.

### Prerequisites
* Generate the map of the markers using Ref-I: [Mapping and Localization from Planar Markers](http://www.uco.es/investiga/grupos/ava/node/57/). Basically, it is a *file.yml* containing all detected markers in the scene and their position. Example:

```
%YAML:1.0
---
aruco_bc_dict: TAG36h11
aruco_bc_nmarkers: 25
aruco_bc_mInfoType: 1
aruco_bc_markers:
   - { id:0, corners:[ [ 1.0879868268966675e+00, -2.2400319576263428e-01,
       5.3463661670684814e-01 ], [ 1.0820814371109009e+00,
       -1.0438479483127594e-01, 4.3158048391342163e-01 ], [
       1.0862079858779907e+00, -2.0736083388328552e-01,
       3.1181859970092773e-01 ], [ 1.0921133756637573e+00,
       -3.2697921991348267e-01, 4.1487473249435425e-01 ] ] }
   - { id:2, corners:[  
   .
   .
   .
```

Then copy the generated file in */config* folder in *cam_map_localization*

* Calibration:

You need to calibration the camera and copy the generated file in */config* folder in *cam_map_localization*

* Install Aruco 3:

[ArUco: a minimal library for Augmented Reality applications based on OpenCV](http://www.uco.es/investiga/grupos/ava/node/26)

* Launch the package
```
roslaunch cam_map_localization camera_marker_detection.launch
```

Once any marker of the map is detected, you'll get the pose of the camera published in the topic */poseStampedDsr* and you can visulize that in *Rviz*

![alt text](https://i.imgur.com/M6Uja1A.png)

## References

I. Marker Mapper paper 
"Mapping and localization from planar markers"
RafaelMuñoz-Salinas, Manuel J.Marín-Jimenez, Enrique Yeguas-Bolivar, R.Medina-Carnicer, , Pattern Recognition, To appear.
https://www.sciencedirect.com/science/article/pii/S0031320317303151
[Read PDF]
 
 
II. Main ArUco paper:
"Automatic generation and detection of highly reliable fiducial markers under occlusion"
http://www.sciencedirect.com/science/article/pii/S0031320314000235
[DOWNLOAD PDF]
[DOWNLOAD Bibtex]
 
III. Generation of marker dictionaries:
"Generation of fiducial marker dictionaries using mixed integer linear programming"
http://www.sciencedirect.com/science/article/pii/S0031320315003544
[DOWNLOAD PDF]
[DOWNLOAD Bibtex]
 
