# Joint-State-Estimation
Codes related to the undergraduate thesis
The project is to implement Explicit Complamantary Filter by Robert Mahony (DOI: 10.1109/CDC.2005.1582367). In this work, the filter is implemented using low-cost IMUs Sparkfun Razor 
9DOF. The implementation is done by two different ways, one, by saving sensor output and running the filter offline, and two, by running the filter online. Implementation folder has 
all the necessary codes for online as well as offline implementation. One can simply run MahonyImplement_Animation_allAng_refine.py after connecting the IMU to Laptop/RPi with USB interface
and editing the port details in the code to get live plots of orientation in terms of Euler angles. Offcourse, ensure that you have relevant python packages installed. THere are also 
codes/results specific to data logging, plotting, testing in the dedicated folders. 
