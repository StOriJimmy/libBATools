//#pragma comment(lib,"F:/3D_reconstruction_item/facade_3DRec/ConvertWorldToImage/libBATool_xml/build/lib/x64/Debug/TinyXML.lib")
//#pragma comment(lib,"F:/3D_reconstruction_item/facade_3DRec/ConvertWorldToImage/libBATool_xml/build/lib/x64/Debug/libBAToolsd.lib")



#include "libBATools.h"
#include <tchar.h>
#include <iostream>
#include <string>
#include "typedefImpl.hpp"

#ifdef SMART3D_SUPPORT
#include "tinystr.h"
#include "tinyxml.h"
#endif
using namespace std;


#define TEST_SMART3D 0
#define TEST_IMAGE_RETRIEVAL 1


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

	// CameraOrientation 
	libba::TVec<double, 16> CameraOrientation(0.0);
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
	libba::TVec<double, 16> XRightYUp(0.0);
	XRightYUp[0] = 1; XRightYUp[5] = -1; XRightYUp[10] = -1; XRightYUp[15] = 1;

	rot = XRightYUp.mat_dot(CameraOrientation).mat_dot(rot);

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

int _tmain(int argc, _TCHAR* argv[])
{
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

	// 双线性内插

	// 输出影像

	return 0;
}
#endif // TEST_IMAGE_RETRIEVAL


#if TEST_SMART3D

int _tmain(int argc, _TCHAR* argv[])
{
	const char * cXmlName = "Block_AT_withouttiepoints.xml";
	const char * cXmlName2 = "CCXFJ01export.xml";
	
	//loadSmart3dXML函数测试
	std::vector<libba::BAImageInfo> image_data;
	if (!(libba::loadSmart3dXML(argv[1], image_data))) 
	{
		std::cerr << "failed to load image bundle files" << std::endl;
		return EXIT_FAILURE;
	}

	/*for (int i = 0; i < image_data.size(); i++)
	{
		cout << image_data[i].Extrinsics().imagepath << endl;
		cout << image_data.size() << endl;
	}*/

	int count = 0;
	for (int i = 0; i < image_data.size(); i++)
	{
		
		/*cout << image_data[i].Extrinsics().photo_ID << endl;
		cout << image_data[i].Extrinsics().imagepath << endl;
		cout << image_data[i].Extrinsics().center_cam[0] << "  " << image_data[i].Extrinsics().center_cam[1] << "  " << image_data[i].Extrinsics().center_cam[2] << endl;
		*/
		if (image_data[i].Extrinsics().photo_ID == 139) {
			int test = 1;
		}
		/*if (image_data[i].Extrinsics().photo_ID == 139)
		{
			image_point = image_data[i].convertWorldToImageCoordinates_test(libba::baPoint3d(340048.9425284779, 4295006.2696251385, 73.59));
			
			cout << image_data[i].Extrinsics().photo_ID << endl;
			cout << image_data[i].Extrinsics().imagepath << endl;
			cout << image_data[i].Intrinsics().width << "  " << image_data[i].Intrinsics().height << endl;
			cout << (int)image_point[0] << "  " << (int)image_point[1] << endl;
			
		}*/

		libba::baPoint2d image_point = image_data[i].convertWorldToImageCoordinates_xml/*(libba::baPoint3d(11.60, -55.66, 43.29));*/(libba::baPoint3d(448595.17, 4856971.02, 213.89));
		if ((image_point[0] >= 0) && (image_point[0] <= image_data[i].Intrinsics().width) && 
			(image_point[1] >= 0) && (image_point[1] <= image_data[i].Intrinsics().height))
		{
			cout << image_data[i].Extrinsics().photo_ID << endl;
			cout << image_data[i].Extrinsics().imagepath << endl;
			cout << image_data[i].Intrinsics().width << "  " << image_data[i].Intrinsics().height << endl;
			cout << (int)image_point[0] << "  " << (int)image_point[1] << endl;
			cout << image_data[i].Extrinsics().center_cam[0] << "  " << image_data[i].Extrinsics().center_cam[1] << "  " << image_data[i].Extrinsics().center_cam[2] << endl;
			cout << image_data[i].Extrinsics().rot_matirx[0] << image_data[i].Extrinsics().rot_matirx[1] << image_data[i].Extrinsics().rot_matirx[8] << endl << endl;
		}
		
	}

	

	return 0;








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