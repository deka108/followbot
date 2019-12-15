# Lab 5 - Follow Bot
Lab 5 for [COMSW4733 Computational Aspects of Robotics](http://www.cs.columbia.edu/~allen/F19/index.html) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Authors

| Name | UNI|
| - | - |
| Deka Auliya Akbar | da2897 |
| Madhavan Seshadri | ms5945 |
| Shravan Karthik | sk4653 |

----
### Prerequisites:

1. Installation of ROS on the machine
1. Installation of `turtlebot gazebo` packagae
1. Python 3.6 or above needs to be installed in the machine

----
### Usage
1. To run part 1, which implements the navigation of the bot along the yellow line, please use the following commands:
`roslaunch followbot launch.launch`
`python src/follower_prep.py`
1. To run part 2, which implements the navigation of the bot along the obstacle course with color markers, please use the following commands:
`ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world`
`python src/follower_color.py`
1. To run part 3 which implements the navigation of the bot along the obstacle course with shape markers, please use the following commands:
`ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world`
`python src/follower_shape.py`
1. To run part 4 which implements the which implements the navigation of the bot along the obstacle course with same color markers, use the following commands:
`ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=extra.world`
`python src/follower_extra.py`

----
### Methods in our implementation
1. Function `image_callback` takes `msg` as a parameter which provides input from the camera. Based of
this input a mask for yellow regions in HSV is applied on a region of the input. The centroid of the 
yellow region is computed using moments of the image and this is used to govern direction of travel.

1. Function `image_callback_color` takes `msg` as a parameter which provides input from the camera. Based of
this input a mask of red, blue, green and finally yellow is applied. 
    1. If a color (other than yellow) is detected, we set the cur_color variable to color detected using the mask. 
    1. Once we detect two paths, we use the cur_color information to update the direction (blue => right, green => left) of travel.
    1. The two paths is detected by taking advantage of the color of the centroid when there are diverging paths. During the divergence of two paths 
     the yellow mask center isn't yellow (would be gray). 
    1. If cur_color is red, we navigate to the centroid of the red_mask and then halt

1. Function `image_callback_shape` takes `msg` as a parameter which provides input from the camera. Based of this
input a mask of red, yellow is applied.
    1. If red pixels are detected using the red mask, we follow a straight path (angular z =0)
    1. The centroid is computed for the red mask, and the edges the red region in the image are also computed
    1. If the distance from the right most red pixel in the mask to the centroid is greater than the distance
     from the left most pixel to the centroid, we set the direction of traversal as right, and vice versa.
     The above method works, since the centroid lies away from the direction the triangle is pointing to.
    1. The direction update is performed when no red pixels are no longer directed
    1. If no red pixels are detected, follow the centroid of the yellow pixel. Apply #4 if necessary
    1. To detect a start, we use the property of the area of the image. If the area of the convex hull of
     the red mask is greater than a threshold, we identify the star, move to its centroid and halt the bot.

#### Video Link

The run for these implementations is available at https://www.youtube.com/playlist?list=PLMbCjZrjdlPNAQHShtGJahKy7xniH-x88
This is a single video capturing all the worlds.

