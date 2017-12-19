# **LDMRS-PTU_LAAS**

## Data formats
> **Note:**

This section contains the processing of the data once is acquired by the system. The raw data can be converted to XYZ by using the model and its calibrated parameters on the file <i class="icon-file"></i> raw2txt.py. Integration with other software like [octomap](https://github.com/OctoMap/octomap/wiki/Importing-Data-into-OctoMap) need log format, which can be obtained by executing the file <i class="icon-file"></i> xyz2log.py.

* [raw2xyz.py]. Data content when RPM and joystick nodes are running (moving=1)
    * columns: ["tilt angle", "pan angle", "beta angle (horizontal angle)", "layer","radial distance","echo","Pulse width","flag","point number","scan number","Number of Return","roverX","roverY","roverZ","roverQw","roverQx","roverQy","roverQz","point_X","point_Y","point_Z"]

* [raw2xyz.py]. Data content when RPM and joystick nodes are not running (moving=0)
    * columns: ["tilt angle", "pan angle", "beta angle (horizontal angle)", "layer","radial distance","echo","Pulse width","flag","point number","scan number","Number of Return","point_X","point_Y","point_Z"]


## Calibration
> **Note:**
Calibration folder consist in graphical evaluation, intrinsic and extrinsic parameters based on experimental data. The experiment used for calibration was scanning a object-reference from different points taking in accoun the position of the rover and 3D data. Intrinsic parameters are calculated from segmented ground and extrinsic parameters from a segmented wall.
> Files <i class="icon-file"></i>:
    * data_1.mat
    * data_2.mat
    * data_3.mat
    * data_4.mat
    * segmented_data.mat

## re-Calibration
> **Note:**
Is a second calibration routine but using outside acquired data.

## Model
> **Note:**
Corresponds to the transformation matrices deduced from the system.
