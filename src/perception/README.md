This folder is comprised of a text file containing list of necessary classes for object identification and a launch file for extracing the images from a rosbag file.

The data collection steps are outlined below.

### Recording and extracting image data from rosbags

1. Open a terminal window, follow the steps the source, build and run the vrx launch file.

2. In another terminal window, launch keyboard teleoperation.

3. In a third terminal window, navigate to the directory in which you want to store the rosbag file.
Use the command below after launching the vrx environment and keyboard teleoperation to record data.
    ```
    rosbag record /wamv/sensors/cameras/front_left_camera/image_raw /wamv/sensors/cameras/front_right_camera/image_raw /wamv/sensors/cameras/middle_right_camera/image_raw
    ```

4. Move the wam-v around the environment.

5. When sufficient data is recorded, use `CTRL + C` to stop recording the rosbag.

6. Use the launch file, `export.launch` and the tutorial, [How to export image and video data from a bag file](http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data), to extract the jpeg images from the rosbag.


### Labelling data points

1. Download the labelImg tool (see documentation [here](https://github.com/tzutalin/labelImg)).

    ```
    pip3 install labelImg
    ```

2. Launch the tool, specifying the path to the class file and images.
    ```
    labelImg [IMAGE_PATH] [PRE-DEFINED CLASS FILE]
    ```

3. Use the GUI to select data points and labels for an image.
4. Save in xml format.
