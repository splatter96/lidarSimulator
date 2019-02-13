# lidarSimulator

This project will eventually contain a ROS node to simulate a LiDAR point cloud that would be generated by a placing objects in the scene discribed by stl objects.  
The data this simulation produces will eventually be used to train a neural network on detecting certain objects in  those point clouds.

The point cloud is generated by raytracing each laser beam produces by the LiDAR scanner.

The original raytracing code can be found under [tinytracer](https://github.com/ssloy/tinyraytracer).

## TODO
- [x] Load different objects from stl files
- [x] Integrate in ROS compilation
- [x] Use ROS/tf vectos and point objects
- [x] Export hit points instead of image
- [x] Form rays according to LiDAR scanner instead of pixelwise
- [x] Moveable objects (Offset from LiDAR)
- [ ] Publish as ROS visualization message
- [ ] Use same coordinate system for everything (Z pointing to sky, Y pointing out of screen)
- [ ] Expose a ROS action or actionLib server
- [ ] Calculate Intensities
- [ ] Performance optimizations
- [ ] OpenCL for actual raytracing