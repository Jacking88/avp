# Assignment Ⅴ: AVP  Mapping
  For this assignment, you are required to complete the mapping algorithm for the AVP system. The assignment provides a dataset of segmented ipm images from simulation. You need to complete the process of the mapping algorithm and display the final result of the AVP mapping.  

- hw1: Complete ipmPlane2Global()  in avp_mapping.cc    
- hw2: Complete detectSlot() in slot.cc     
- hw3: Explain the effects of image-processing operations in extractSlot(), adjust their parameters and try to design your image processing method, is your result better?  

## Build & Run

Terminal (Open in l5_ws): 

```bash
catkin_make 
source ./devel/setup.bash
roslaunch avp_mapping avp_mapping.launch
```
 
There is "ipm_detection_result" window when running, showing the detection result of current ipm image;   
- press 's' to enter "step by step" mode, program will wait for keyboard input before handling next ipm image;    
- press 'r' to enter Automatic mode, program will run automatically;  
- The default mode is Automatic.  

You can see the result of avp mapping in rviz.

# Submission
You need to submit a ***PDF*** file that contains:  
- The completed code of hw1 and hw2, with explanation of your code;
- The report of hw3;
- The result(final rviz image) of your avp mapping.
- ***DO NOT*** submit the "l5_ws" and "ipm_data" dataset.  
