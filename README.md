# libBATools
tools to parse bundle adjustment results


## Dependencies

The code is based on the following prerequisites:
- Eigen (tested on 3.3.4)


## Compilation
prerequisites: cmake version >= 3.0

```sh
1. git clone https://github.com/StOriJimmy/libBATools.git
2. cd libBATools
```
**in the cmake-gui**
```
3. set CMAKE_INSTALL_PREFIX to the directory you what to install the compilation
4. set EIGEN_INCLUDE_DIR if no environment variable is given
5. cmake generate
6. open the vs solution, and install
```
A demo project is attached for test

## Supported File Format

- .info: _las and image registration result, cv system_

```
--------------------------------Camera Information--------------------------------  
CameraID x0 y0 f format_X format_Y pixelSize k0 k1 k2 k3 p1 p2 b1 b2 Attrib  
<num_cameras>
    <camera1>
    <camera2>
    <camera3>
    ...
注：采用CV畸变系数
--------------------------------Image Information--------------------------------
ImageID ImageName Xs Ys Zs Phi Omega Kappa StripID Attrib CameraID bFlag BlockID 
<num_images>
    <image1>
    <image2>
    ...
```

- bundle.out:  _Bundler file v0.3_ 
```
# Bundle file v0.3
    <num_cameras> <num_points>   [two integers]
    <camera1>
    <camera2>
       ...
    <cameraN>
    <point1>
    <point2>
       ...
    <pointM>
Each camera entry <cameraI> contains the estimated camera intrinsics and extrinsics, and has the form:
    <f> <k1> <k2>   [the focal length, followed by two radial distortion coeffs]
    <R>             [a 3x3 matrix representing the camera rotation]
    <t>             [a 3-vector describing the camera translation]
```
  more information can be found in the official website
> http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.3-manual.html#S6

- image list  
  Ascii file, recording image path (relative or absolute) and size. 
  If no image size information in the list, set the image info by yourself  
  For each line: 
```
image_path width height  
``` 

## Functions

- [x] convert world to image coordinates and backwards
- [x] convert world to camera coordinates and backwards
- [x] check point in image
- [x] get viewing direction and view point (camera center)
- [ ] ray tracing
- [ ] point-wise visibility check

## Contact
  
If you found bugs or have new ideas, do not hesitate to pull request :grimacing:   
If you have trouble compiling or using this software, email to liuxy0319@outlook.com