# libBATools
tools to parse bundle adjustment results


## Dependencies

The code is based on the following prerequisites:
- Eigen (tested on 3.3.4)
- vcglib (optional) [get vcglib](https://github.com/StOriJimmy/vcglib/tree/devel)

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

- .xml: ContextCapture BlocksExchange XMLformat
```
<BlocksExchange version="2.1">
  <!-- Definition of one or several useful spatial reference systems (SRS). -->
  <!-- Here, we need the French Lambert 93. -->
  <SpatialReferenceSystems>
    <SRS>
      <!-- In the sequel, the SRS is referred to by this ID. -->
      <Id>0</Id>
       <!-- User-friendly SRS name. -->
      <Name>Lambert 93</Name>
      <!-- The spatial reference system definition can be either a Well Known Text definition (e.g. WGS84), "EPSG: ", a PROJ.4 definition, or the path to a file containing such a definition. -->
      <Definition>EPSG:2154</Definition>
    </SRS>
  </SpatialReferenceSystems>
  <!-- Optional: a path to prefix image paths when they are not absolute (default: use the directory of the exchange file). -->
  <BaseImagePath>D:\data\Paris2012sample\Images</BaseImagePath>
  <!-- Block definition. A single file can define several blocks. -->
  <Block>
    <!-- Block name and description are optional. -->
    <Name>Paris2012</Name>
    <Description>Small sample of the Paris 2012 dataset</Description>
    <!-- The type of block is optional. Currently, there is a specific behavior for the "Aerial" type only. -->
    <Type>Aerial</Type>
    <!-- Photo downsampling rate can be optionally provided with <PhotoDownsamplingRate> tag -->
    <!-- Minmum and maximum viewing distance can be optionally provided with <MinimumViewingDistance> and <MaximumViewingDistance> tags -->
    <!-- ID of the base SRS used in the block. By default, the spatial reference system is assumed to be a local Cartesian coordinate system. -->
    <SRSId>0</SRSId>
    <Photogroups>
      <!-- All photos taken using the same physical camera, with identical focal length and dimensions must be gathered in a photogroup. -->
      <Photogroup>
        <!-- Optionally, a different spatial reference system can be specified for each photogroup with the <SRSId> tag. By default, the SRS of the block is assumed. -->
        <!-- Optionally, a name with the <Name> tag, and a description with a <Description> tag. -->
        <Name>UCX</Name>
        <!-- Image dimensions, in pixels. -->
        <ImageDimensions>
          <Width>9420</Width>
          <Height>14430</Height>
        </ImageDimensions>
        <!-- Optional camera model type Perspective or Fisheye (Perspective type is used if this tag is unspecified). -->
        <CameraModelType>Perspective</CameraModelType>
        <!-- Sensor's largest dimension, in millimeters. -->
        <!-- Sensor's pixel size, in millimeters, can be substituted for this field. -->
        <!-- For this dataset, it would be: -->
        <!-- <PixelSize>0.0072</PixelSize> -->
        <SensorSize>103.896</SensorSize>
        <!-- Focal length, in millimeters. This can be either the nominal specification or a closer estimate. -->
        <FocalLength>100.735601903992</FocalLength>
        <!-- Optionally, focal length, in pixels. Can replace FocalLength and SensorSize data. -->
        <!-- For this dataset, it would be: -->
        <!-- <FocalLengthPixels>13991.05582</FocalLengthPixels> -->
        <!-- Optionally, specification of the xy-axis of the camera sensor for pose rotation and for any position given in millimeter (i.e. <PrincipalPoint> and <Measurement> when using <xmm> and <ymm>) -->
        <!-- Possible values: XRightYDown (default), XRightYUp (more frequent in photogrammetry), XLeftYDown, XLeftYUp, XDownYRight, XDownYLeft, XUpYRight, XUpYLeft -->
        <CameraOrientation>XRightYDown</CameraOrientation>
        <!-- Principal point, with respect to the sensor center. This field is optional: by default, the sensor center is used as an initial estimate of the principal point. -->
        <PrincipalPoint>
          <!-- 2D image position of the principal point, in pixels. The origin is the center of the upper-left image pixel, the x-axis is oriented to the right side of the image, and the y-axis is oriented to the bottom of the image. -->
          <x>4683.755692</x>
          <y>7223.0141002</y>
          <!-- Alternatively, a 2D position with respect to the sensor center can be specified, in millimeters. In this case, the xy-axis are oriented according to CameraOrientation. -->
          <!-- For this dataset, it would be: -->
          <!-- <xmm>-0.1853590176</xmm> -->
          <!-- <ymm>0.06130152144</ymm> -->
        </PrincipalPoint>
        <!-- Lens distortion coefficients. This field is optional: by default, no distortion is assumed as an initial estimate. -->
        <!-- Distortion coefficients correspond to the classical Brown's distortion model, as described in http://en.wikipedia.org/wiki/Distortion_(optics). -->
        <Distortion>
          <K1>-0.0010041516639</K1>
          <K2>0.0056007056563</K2>
          <K3>-0.009874171117100001</K3>
          <P1>0</P1>
          <P2>0</P2>
        </Distortion>
        <!-- Fisheye focal matrix used only for the camera model type Fisheye. -->
        <FisheyeFocalMatrix>
          <M_00>1960</M_00>
          <M_01>0</M_01>
          <M_10>0</M_10>
          <M_11>1960</M_11>       
        </FisheyeFocalMatrix>
        <!-- Fisheye distortion coefficients used only for the camera model type Fisheye. -->
        <FisheyeDistortion>
          <P0>0</K1>
          <P1>1</K2>
          <P2>0</K3>
          <P3>0</P1>
          <P4>0</P2>
        </FisheyeDistortion>
        <Photo>
          <!-- The ID of a photograph must be unique over the whole block (in other words, even across other photogroups). -->
          <Id>146</Id>
          <!-- Path of the image. If not absolute, it is considered to be relative to BaseImagePath if defined, or otherwise to the directory of the exchange file. -->
          <ImagePath>071_2810.jpg</ImagePath>
          <!-- Optional path of the image mask. If not absolute, it is considered to be relative to BaseImagePath if defined, or otherwise to the directory of the exchange file. -->
          <!-- <MaskPath>071_2810_msk.jpg</MaskPath> -->
          <!-- Exterior orientation of the camera, as estimated by navigation devices or aerotriangulation. This field is optional. -->
          <!-- Optional component Id: 1 is the main component (default), 0 is no component. Photos with incomplete pose cannot belong to the main component -->
          <!-- <Component>1</Component> -->
          <Pose>
            <!-- 3D rotation of the camera in the spatial reference system defined above. -->
            <!-- It defines the rotation matrix transforming world coordinates (in the specified SRS) into camera coordinates (with axes defined by the CameraOrientation of the photogroup). -->
            <!-- The rotation is optional: the pose can be limited to the 3D position of the camera center (e.g. when GPS position only is available). -->
            <Rotation>
              <!-- It can be specified in one of the three representations below: -->
              <!-- 1. A 3x3 matrix, using the <M_ij> tags, where M_ij denotes the coefficient of the (i+1)th row and the (j+1)th column of the rotation matrix. -->
              <M_00>-0.9999982912233401</M_00>
              <M_01>-0.001636319085375301</M_01>
              <M_02>-0.0008602425863163225</M_02>
              <M_10>-0.001631068695467463</M_10>
              <M_11>0.9999802528616577</M_11>
              <M_12>-0.00606906089589293</M_12>
              <M_20>0.0008701565192966738</M_20>
              <M_21>-0.006067647409696231</M_21>
              <M_22>-0.9999812130648239</M_22>
              <!-- 2. Omega/Phi/Kappa angles, in degrees, using the <Omega>, <Phi>, <Kappa> tags. Please refer to the formula in the camera model document. -->
              <!-- For this dataset, it would be: -->
              <!-- <Omega>179.6523471469934</Omega> -->
              <!-- <Phi>0.04985630236313049</Phi> -->
              <!-- <Kappa>179.9065465708369</Kappa> -->
              <!-- 3. Heading/Pitch/Roll angles, in degrees using the <Heading>, <Pitch>, <Roll> tags. Please refer to the formula in the camera model document. -->
            </Rotation>
            <!-- 3D position of the camera center in the spatial reference system defined above. -->
            <!-- Depending on the SRS, x/y may stand either for longitude/latitude, for map projection coordinates, or for something else. -->
            <!-- Depending on the SRS, z may stand either for a geoid height, for an orthometric height, or for something else. -->
            <Center>
              <x>651999.7159189156</x>
              <y>6863073.633923346</y>
              <z>1318.897690166719</z>
            </Center>
          </Pose>
          <!-- Optional depth information can be provided with <NearDepth>, <MedianDepth>, and <FarDepth> tags -->
          <!-- Optional Exif data (each exif data property is optional) --><!-- 
          <ExifData>
            <ImageDimensions>
              <Width>4000</Width>
              <Height>3000</Height>
            </ImageDimensions>
            <PixelDimensions>
              <Width>0.02</Width>
              <Height>0.02</Height>
            </PixelDimensions>
            <GPS>
              <Latitude>45.5686684</Latitude>
              <Longitude>2.6551104</Longitude>
              <Altitude>1589.5469</Altitude>
            </GPS>
            <FocalLength>24</FocalLength>
            <FocalLength35mmEq>36</FocalLength35mmEq>
            <Make>FUJIFILM</Make>
            <Model>FinePix S3Pro</Model>
            <LensModel>FinePix S3Pro</LensModel>
            <DateTimeOriginal>2015-04-29T06:40:26</DateTimeOriginal>
          </ExifData> -->       
        </Photo>
        ...
      </Photogroup>
    </Photogroups>
    <!-- Control points are highly recommended for accurate positioning. -->
    <ControlPoints>
      <!-- Optionally, a different spatial reference system can be specified with the <SRSId> tag. By default, the SRS of the block is assumed. -->
      <ControlPoint>
        <!-- Control point name and description are optional. -->
        <Name>Control point #1</Name>
        <!-- Optionally, a different spatial reference system can be specified for each control point with the <SRSId> tag. By default, the SRS of the control points container is assumed. -->
        <!-- 3D position of the control point in the active spatial reference system. -->
        <!-- Depending on the SRS, x/y may stand either for longitude/latitude, for map projection coordinates, or for something else. -->
        <!-- Depending on the SRS, z may stand either for a geoid height, for an orthometric height, or for something else. -->
        <Position>
          <x>652788.0525588237</x>
          <y>6863015.362218254</y>
          <z>78.07000000122935</z>
        </Position>
        <!-- Image measurements. -->
        <Measurement>
          <!-- ID of the photo where the measurement is taken. The measurements of a control point must have different photo IDs. -->
          <PhotoId>151</PhotoId>
          <!-- 2D image position of the measurement, in pixels. The origin is the center of the upper-left image pixel, the x-axis is oriented to the right side of the image, and the y-axis is oriented to the bottom of the image. -->
          <!-- Alternatively, a 2D position with respect to the principal point, in millimeters. In this case, the xy-axis are oriented according to the CameraOrientation of the corresponding photogroup. -->
          <!-- For this dataset, it would be: -->
          <!-- <xmm>18.6231910176</xmm> -->
          <!-- <ymm>-4.48973352144</ymm> -->
          <x>7270.31</x>
          <y>6599.44</y>
        </Measurement>
        ...
      </ControlPoint>
      <ControlPoint>
        <Name>Control point #2</Name>
        <Position>
          <x>652123.2211166573</x>
          <y>6863245.976366176</y>
          <z>80.07999999914318</z>
        </Position>
        <!-- If specified, a control point may be a check point (default: false)-->
        <CheckPoint>true</CheckPoint>
        <!-- If specified, a control point may have accuracies -->
        <HorizontalAccuracy>0.01</HorizontalAccuracy>
        <VerticalAccuracy>0.10</VerticalAccuracy>
        <Measurement>
          <PhotoId>146</PhotoId>
          <x>3296.56</x>
          <y>9253.75</y>
        </Measurement>
        ...
      </ControlPoint>
      <ControlPoint>
        <Name>Control point #3</Name>
        <!-- If specified, a control point might be Full (default), Horizontal or Vertical -->
        <Category>Horizontal</Category>
        <Position> <!-- no z position -->
          <x>652365.1205012415</x>
          <y>6863549.148163618</y>
        </Position>
        <HorizontalAccuracy>0.01</HorizontalAccuracy> <!-- no vertical accuracy -->
        <Measurement>
          <PhotoId>95</PhotoId>
          <x>3178.26</x>
          <y>4020.21</y>
        </Measurement>
        ...
      </ControlPoint>
      ...
    </ControlPoints>
    <!-- Known tie points may be used to refine an aerotriangulation. -->
    <!-- Uncomment this section before import to get a single tie point in the imported block. Please note that a reconstruction created from a such block will fail. -->
    <!--<TiePoints>
      --><!-- Optionally, a different spatial reference system can be specified with the <SRSId> tag. By default, the SRS of the block is assumed. --><!--
      <TiePoint>
        --><!-- Tie point name and description are optional. --><!--
        --><!-- Optionally, a different spatial reference system can be specified for each tie point with the <SRSId> tag. By default, the SRS of the tie points container is assumed. --><!--
        --><!-- Optional 3D position of the tie point in the active spatial reference system. --><!--
        --><!-- Depending on the SRS, x/y may stand either for longitude/latitude, for map projection coordinates, or for something else. --><!--
        --><!-- Depending on the SRS, z may stand either for a geoid height, for an orthometric height, or for something else. --><!--
        <Position>
          <x>652119.8871409688</x>
          <y>6863304.305716386</y>
          <z>87.79328384995461</z>
        </Position>
        --><!-- Optional tie point color. --><!--
        <Color>
          <Red>0.59</Red>
          <Green>1.0</Green>
          <Blue>0.0</Blue>
        </Color>
        --><!-- Image measurements. --><!--
        <Measurement>
          --><!-- Optional measurement type User or Automatic (Automatic type is used if this tag is unspecified). --><!--
          <Type>Automatic</Type>
          --><!-- ID of the photo where the measurement is taken. The measurements of a tie point must have different photo IDs. --><!--
          <PhotoId>146</PhotoId>
          --><!-- 2D image position of the measurement, in pixels. The origin is the center of the upper-left image pixel, the x-axis is oriented to the right side of the image, and the y-axis is oriented to the bottom of the image. --><!--
          --><!-- Alternatively, a 2D position with respect to the principal point, in millimeters. In this case, the xy-axis are oriented according to the CameraOrientation of the corresponding photogroup. --><!--
              <x>3324.26001</x>
              <y>9930.269531</y>
        </Measurement>
        <Measurement>
              <PhotoId>158</PhotoId>
              <x>9079.006836000001</x>
              <y>9902.772461000001</y>
        </Measurement>
        <Measurement>
             <PhotoId>162</PhotoId>
             <x>6240.366211</x>
             <y>9896.118164</y>
        </Measurement>
      </TiePoint>
    </TiePoints>-->
    <!-- Optional block's positioning constraints based on user tie point (0 based indices). -->
    <!-- Each constraint is optional. -->
    <!--<PositioningConstraints>
         <OriginConstraint>
              <O>2</O>
         </OriginConstraint>
         <ScaleConstraint>
              <A>0</A>
              <B>3</B>
              <DistanceAB>56.350</DistanceAB>
         </ScaleConstraint>
         <AxisConstraint>
              <A>0</A>
              <B>3</B>
              <AxisAB>z</AxisAB> --><!-- x, y, or z --><!--
         </AxisConstraint>
         <OrientationConstraint>
              <A>0</A>
              <B>3</B>
              <C>2</C>
              <AxisAB>z</AxisAB> --><!-- x, y or z --><!--
              <AxisSideC>upperX</AxisSideC> --><!-- lowerX, upperX, lowerY, upperY, lowerZ, or upperZ --><!--
               </OrientationConstraint>
          </PositioningConstraints>-->
     </Block>
</BlocksExchange>
```
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