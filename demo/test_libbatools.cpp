//#pragma comment(lib,"F:/3D_reconstruction_item/facade_3DRec/ConvertWorldToImage/libBATool_xml/build/lib/x64/Debug/TinyXML.lib")
//#pragma comment(lib,"F:/3D_reconstruction_item/facade_3DRec/ConvertWorldToImage/libBATool_xml/build/lib/x64/Debug/libBAToolsd.lib")



#include "libBATools.h"
#include <tchar.h>
#include <iostream>
#include <string>

#ifdef SMART3D_SUPPORT
#include "tinystr.h"
#include "tinyxml.h"
#endif
using namespace std;


#define TEST_SMART3D 1
#define TEST_IMAGE_RETRIEVAL 0
#define TEST_HPR_CONVERT 0

#ifdef USE_VCGLIB
#include "vcg/space/intersection3.h"
#endif // USE_VCGLIB


#ifdef USE_VCGLIB
bool testBackProjection(const libba::BAImageInfo & image, const libba::baPoint2d & pt_img, const vcg::Plane3d & plane, libba::baPoint3d & pt_world)
{
	vcg::Line3d line;
	libba::baPoint3d cam_center = image.getViewPoint();
	line.SetOrigin(vcg::Point3d(*cam_center));

	libba::baPoint3d pt_img_world = image.convertImageToWorldCoordinates(pt_img);
	libba::baPoint3d pt_dir = pt_img_world - cam_center;
	line.SetDirection(vcg::Point3d(*pt_dir));
	vcg::Point3d pt;
	if (!vcg::IntersectionPlaneLine(plane, line, pt))
		return false;
	
	pt_world[0] = pt[0];
	pt_world[1] = pt[1];
	pt_world[2] = pt[2];
	return true;
}
#endif // USE_VCGLIB

//libba::baPoint3d convertWGS84_ENU_to_LLA(const libba::baPoint3d & enu_3d) 
//{
//
//}
//
//
//libba::baPoint2d convertWGS84_LLA_to_UTM(const libba::baPoint3d & lla) 
//{
//
//}

#if TEST_IMAGE_RETRIEVAL

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI


constexpr double DEG_TO_RAD = (M_PI / 180.0);
#if 0
libba::BAImageInfo DBToImageInfo(
	double longitude, double latitude, double height, double phi, double omega, double kappa, // 外方位元素
	double pwidth, double pheight, // 影像宽、长
	double fcallength, // 焦距
	double pixelsize,  // 像素大小
	std::string cameraorie, // 相机方向
	double principalx, double principaly, // 像主点
	std::string image_path // 影像路径
)
{
	phi = DEG_TO_RAD * phi;
	kappa = DEG_TO_RAD * kappa;
	omega = DEG_TO_RAD * omega;

	double M_00 = cos(phi)*cos(kappa);
	double M_01 = cos(omega)*sin(kappa) + sin(omega)*sin(phi)*cos(kappa);
	double M_02 = sin(omega)*sin(kappa) - cos(omega)*sin(phi)*cos(kappa);

	double M_10 = -cos(phi)*sin(kappa);
	double M_11 = cos(omega)*cos(kappa) - sin(omega)*sin(phi)*sin(kappa);
	double M_12 = sin(omega)*cos(kappa) + cos(omega)*sin(phi)*sin(kappa);

	double M_20 = sin(phi);
	double M_21 = -sin(omega)*cos(phi);
	double M_22 = cos(omega)*cos(phi);

	libba::BAImageInfo image_info;

	// 外方位元素

	auto & img_extri = image_info.Extrinsics();
	auto & rot = img_extri.rot;
	
	libba::baPoint3d a, b, c; // 列
	a = { M_00, M_10, M_20 };
	b = { M_01, M_11, M_21 };
	c = { M_02, M_12, M_22 };

	libba::baMatrix44d rot_temp;
	rot_temp[0] = M_00;
	rot_temp[1] = M_01;
	rot_temp[2] = M_02;
	rot_temp[3] = 0.0;
	rot_temp[4] = M_10;
	rot_temp[5] = M_11;
	rot_temp[6] = M_12;
	rot_temp[7] = 0.0;
	rot_temp[8] = M_20;
	rot_temp[9] = M_21;
	rot_temp[10] = M_22;
	rot_temp[11] = 0.0;
	rot_temp[12] = 0.0;
	rot_temp[13] = 0.0;
	rot_temp[14] = 0.0;
	rot_temp[15] = 1.0;

	// CameraOrientation 
	baMatrix44d CameraOrientation(0.0);
	CameraOrientation[15] = 1.0f;

	if (cameraorie == "XRightYDown") {
		CameraOrientation[0] = CameraOrientation[5] = CameraOrientation[10] = 1.0;
	}
	else if (cameraorie == "XLeftYDown") {
		CameraOrientation[0] = -1;
		CameraOrientation[5] = 1;
		CameraOrientation[10] = -1;
	}
	else if (cameraorie == "XLeftYUp") {
		CameraOrientation[0] = -1;
		CameraOrientation[5] = -1;
		CameraOrientation[10] = 1;
	}
	else if (cameraorie == "XRightYUp") {
		CameraOrientation[0] = 1;
		CameraOrientation[5] = -1;
		CameraOrientation[10] = -1;
	}
	else if (cameraorie == "XDownYRight") {
		CameraOrientation[1] = 1;
		CameraOrientation[4] = 1;
		CameraOrientation[10] = -1;
	}
	else if (cameraorie == "XDownYLeft") {
		CameraOrientation[1] = -1;
		CameraOrientation[4] = 1;
		CameraOrientation[10] = 1;
	}
	else if (cameraorie == "XUpYLeft") {
		CameraOrientation[1] = -1;
		CameraOrientation[4] = -1;
		CameraOrientation[10] = -1;
	}
	else if (cameraorie == "XUpYRight") {
		CameraOrientation[1] = 1;
		CameraOrientation[4] = -1;
		CameraOrientation[10] = 1;
	}
	
	// 转成 XRightYUp
	baMatrix44d XRightYUp(0.0);
	XRightYUp[0] = 1; XRightYUp[5] = -1; XRightYUp[10] = -1; XRightYUp[15] = 1;

	rot = XRightYUp.mat_dot(CameraOrientation).mat_dot(rot_temp);

	img_extri.tra = libba::baPoint3d(longitude, latitude, height);

	// 内方位元素
	auto& cam_intri = image_info.Intrinsics();
	cam_intri.centerPx = libba::baPoint2d(principalx, pheight - principaly);
	cam_intri.focalMm = fcallength;
	cam_intri.pixelSizeMm = libba::baPoint2d(pixelsize, pixelsize);
	cam_intri.k[0] = 0;
	cam_intri.k[1] = 0;
	cam_intri.k[2] = 0;
	cam_intri.tang_distor[0] = 0;
	cam_intri.tang_distor[1] = 0;

	image_info.width() = pwidth;
	image_info.height() = pheight;
	image_info.m_path = image_info.m_name = image_path;
	
	return image_info;
}	
#endif

int _tmain(int argc, _TCHAR* argv[])
{
#if 0
	//////////////////////////////////////////////////////////////////////////
	//! 输入参数
	libba::baPoint3d view_point; // 视点/人眼
	libba::baPoint3d view_upDir; // 模型坐标系视角竖直向上方向
	libba::baPoint3d view_dir;   // 模型坐标系视线方向
	libba::baPoint3d obj_center; // 物方中心/鼠标当前位置
	double frame_width, frame_height; // 前端显示宽、高
	std::vector<libba::BAImageInfo> current_images; // 鼠标所在格网对应的影像组

	view_upDir.normalize();
	view_dir.normalize();

	//////////////////////////////////////////////////////////////////////////
	//! 查找最优影像
	double min_angle(FLT_MAX);
	libba::BAImageInfo best_image;
	libba::baPoint3d obj_to_view = view_point - obj_center;
	for (auto image_info : current_images) {
		libba::baPoint2d point_in_image = image_info.convertWorldToImageCoordinates(obj_center);
		if (!image_info.isInImage(point_in_image))
			continue;

		libba::baPoint3d cam_dir = image_info.getViewDir();
		libba::baPoint3d cam_center = image_info.getViewPoint();

		libba::baPoint3d cam_to_obj = (obj_center - cam_center); cam_to_obj.normalize();
		libba::baPoint3d obj_to_cam = -cam_to_obj;
		double obj_angle = obj_to_cam.dot(obj_to_view);
		double cam_angle = cam_dir.dot(cam_to_obj);

		if (obj_angle < 0.0f || cam_angle < 0.0f) {
			continue;
		}

		obj_angle = acos(obj_angle);
		cam_angle = acos(cam_angle);

		double angle = obj_angle + cam_angle;
		if (angle < min_angle) {
			min_angle = angle;
			best_image = image_info;
		}
	}

	if (min_angle > M_PI) {
		return 1; // 找不到合格的影像
	}
	
	//////////////////////////////////////////////////////////////////////////
	//! 影像纠正
	libba::baPoint3d view_plane_dir = libba::cross_p3(view_dir, view_upDir);
	view_plane_dir.normalize();
	// 物方，需要显示的角点
	/*
	2       1
	   obj
	3       4
	*/
	libba::baPoint3d corner_1 = obj_center + (view_upDir * frame_height + view_plane_dir * frame_width);
	libba::baPoint3d corner_2 = obj_center + (view_upDir * frame_height - view_plane_dir * frame_width);
	libba::baPoint3d corner_3 = obj_center - (view_upDir * frame_height + view_plane_dir * frame_width);
	libba::baPoint3d corner_4 = obj_center - (view_upDir * frame_height - view_plane_dir * frame_width);

	// 像方，对应顶点

	libba::baPoint2d img_corner_1 = best_image.convertWorldToImageCoordinates(corner_1);
	libba::baPoint2d img_corner_2 = best_image.convertWorldToImageCoordinates(corner_2);
	libba::baPoint2d img_corner_3 = best_image.convertWorldToImageCoordinates(corner_3);
	libba::baPoint2d img_corner_4 = best_image.convertWorldToImageCoordinates(corner_4);

	// 判断是否在影像内
	if (!best_image.isInImage(img_corner_1) ||
		!best_image.isInImage(img_corner_2) ||
		!best_image.isInImage(img_corner_3) ||
		!best_image.isInImage(img_corner_4)) {
		// 仅输出部分，或者这一步需要在找best_image时判断
	}
#endif
	// 双线性内插

	// 输出影像

	return 0;
}
#endif // TEST_IMAGE_RETRIEVAL


#if TEST_SMART3D

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc < 2) {
		std::cerr << "please input the smart3d xml path" << std::endl;
		return 1;
	}
	
	//loadSmart3dXML函数测试
	std::vector<libba::BAImageInfo> image_data;
	if (!(libba::loadSmart3dXML(argv[1], image_data))) 
	{
		std::cerr << "failed to load image bundle files" << std::endl;
		return EXIT_FAILURE;
	}
	return 1;
	
	for (int i = 0; i < image_data.size(); i++)
	{
		libba::BAImageInfo image_info = image_data[i];
		if (image_data[i].m_name == "DJI_0282") {
			libba::baPoint3d obj_center(-14.3886, 78.7521, 55.3733);

			libba::baPoint2d image_point = image_info.convertWorldToImageCoordinates(obj_center);

			if (image_info.isInImage(image_point)) {
				std::cout << "point in image!" << image_point[0] << " " << image_point[1] << std::endl;
			}

			libba::baPoint3d cam_dir = image_info.getViewDir();
			libba::baPoint3d cam_center = image_info.getViewPoint();

			libba::baPoint3d view_point(-35.7712, 26.9136, 111.558);
			libba::baPoint3d cam_to_obj = obj_center - cam_center;
			cam_to_obj = cam_to_obj / sqrt(cam_to_obj.dot(cam_to_obj));

			libba::baPoint3d obj_to_cam = -cam_to_obj;
			libba::baPoint3d obj_to_view = view_point - obj_to_cam; 

			obj_to_view = obj_to_view / sqrt(obj_to_view.dot(obj_to_view));

			double obj_angle = obj_to_cam.dot(obj_to_view);
			double cam_angle = cam_dir.dot(cam_to_obj);

			double obj_angle1 = acos(obj_angle);
			double cam_angle1 = acos(cam_angle);

			double angle = obj_angle1 + cam_angle1;

			int dddd = 1;
		}
	}

	return EXIT_SUCCESS;


//	if (argc < 3) {
//		return false;
//	}
//
//	std::string bundle_out = argv[1];
//	std::string image_list = argv[2];
//
//	std::vector<libba::BAImageInfo> image_data;
//	if (!loadBundleOutFiles(image_list, bundle_out, image_data)) {
//		std::cerr << "failed to load image bundle files" << std::endl;
//		return EXIT_FAILURE;
//	}
//
//	std::cout << image_data.size() << " images loaded!" << std::endl;
//
//	libba::baPoint2d image_point;
//	if (!testProjection(image_data[0], libba::baPoint3d(316384.156250, 234354.078125,55.636), image_point)) {
//		std::cout << "point out of range" << std::endl;
//	}
//	
//#ifdef USE_VCGLIB
//	vcg::Plane3d plane;
//	plane.Set(vcg::Point3d(0, 0, 1), 55);
//	libba::baPoint3d pt_world;
//	if (!testBackProjection(image_data[0], libba::baPoint2d(4996, 2958), plane, pt_world)) {
//		std::cout << "intersection failed" << std::endl;
//	}
//#endif // USE_VCGLIB
//
//	return EXIT_SUCCESS;
}

#endif // TEST_SMART3D

#if TEST_HPR_CONVERT

int _tmain(int argc, _TCHAR* argv[])
{
	std::cout << "THE FILE SHOULD ONLY CONTAINS H P R FOR EACH LINE!" << std::endl;
	if (argc < 2) {
		std::cerr << "please input the HPR path" << std::endl;
		return 1;
	}
	FILE *fp = fopen(argv[1], "r");
	char output_path[1024];
	strcpy(output_path, argv[1]); strcat(output_path, "_ex.txt");
	FILE* fp_out = fopen(output_path, "w");
	char lbuf[1024];
	while (fgets(lbuf, 1024, fp)) {
		double heading, pitch, roll;
		sscanf(lbuf, "%lf%lf%lf", &heading, &pitch, &roll);

		double h = heading * M_PI / 180;
		double p = pitch * M_PI / 180;
		double r = roll * M_PI / 180;
		double R[9];
		R[0] = cos(p)*cos(r);
		R[1] = -cos(h)*sin(r) + sin(h)*sin(p)*cos(r);
		R[2] = sin(h)*sin(r) + cos(h)*sin(p)*cos(r);
		R[3] = cos(p)*sin(r);
		R[4] = cos(h) * cos(r) + sin(h)*sin(p)*sin(r);
		R[5] = -sin(h)*cos(r) + cos(h)*sin(p)*sin(r);
		R[6] = -sin(p);
		R[7] = sin(h) * cos(p);
		R[8] = cos(h)*cos(p);

		double phi, omega, kappa;
		libba::R2POK2(R, phi, omega, kappa);
		fprintf(fp_out, "%lf %lf %lf\n", phi, omega, kappa);
	}
	return 0;
}

#endif