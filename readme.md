# ZED ORB SLAM

Processing a ZED SVO file through ORB SLAM3

## zed_config.yaml

- ORB slam requires the camera parameters in a yaml file
- ZED keeps their camera parameters in /usr/local/zed/settings
- Had to copy over values into the expected format for ORB SLAM
- ZED settings has camera calibration parameters for 2k, HD, VGA so need to copy parameters for the recorded type (if your SVO is 2k, copy the 2k calibration values)
    - TODO: Create an automated way to do this.
    



