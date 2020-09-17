﻿#include "libBATools.h"
#include "typedefImpl.hpp"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <io.h>
#include <map>
#include <Eigen/Core>
#include <Eigen/LU>

#include <string>
#include <iomanip>

#include <iostream>
#ifdef SMART3D_SUPPORT
#include "tinystr.h"
#include "tinyxml.h"
#endif // SMART3D_SUPPORT



using namespace std;
using namespace Eigen;

LIBBATOOLS_NAMESPACE_BEGIN


#ifdef SMART3D_SUPPORT
LIBBA_API bool loadSmart3dXML(const char * ccXmlName, std::vector<BAImageInfo>& image_infos)
{
	//创建一个XML的文档对象。
	TiXmlDocument *myDocument = new TiXmlDocument(ccXmlName);
	if (!myDocument->LoadFile()) {
		return false;
	}

	//获得xml的头，即声明部分
	TiXmlDeclaration* decl = myDocument->FirstChild()->ToDeclaration();
#ifdef _DEBUG
	cout << "xml文件的版本是:" << decl->Version() << endl;
	cout << "xml的编码格式是：" << decl->Encoding() << endl;
#endif // _DEBUG

	//获得根元素
	TiXmlElement *RootElement = myDocument->RootElement();
	if (!RootElement) {
		return false;
	}

	//输出根元素名称

	//cout << "根元素:" << RootElement->Value() << endl;

	/*
	for (TiXmlElement *child1 = RootElement->FirstChildElement(); child1; child1 = child1->NextSiblingElement())
	{
		//该层 <SpatialReferenceSystems> 和 <Block> 下的 <SRSId> 决定了photo中Center的参考系 

		if (child1->Value() == string("Block"))
		{
			//cout << "Succeed to find label Blook" << endl;
			for (TiXmlElement *child2 = child1->FirstChildElement(); child2; child2 = child2->NextSiblingElement())
			{
				if (child2->Value() == string("Photogroups"))
				{
					//cout << "Succeed to find label Photogroups" << endl;
					for (TiXmlElement *child3 = child2->FirstChildElement(); child3; child3 = child3->NextSiblingElement())
					{
						if (child3->Value() == string("Photogroup"))
						{
							//cout << "Succeed to find label Photogroup" << endl;

							//预定义变量来存储同一个photogroup内所有photo相同的变量（即相机自身的参数）
							long double focalLengthPixels, principlePoint_x, principlePoint_y, k1, k2, k3, p1, p2, camaraOrientation, skew, pixelRatio;
							int width(0), height(0);
							double sensor_size(0.f);

							//一个计算photo个数的计数器
							int count_photo = 0;

							for (TiXmlElement *child4 = child3->FirstChildElement(); child4; child4 = child4->NextSiblingElement())
							{
								//对于该层的<photo>针对各组影像相同的参数  在vector构造完成后以赋值方式循环输入（每个元素对应的成员变量值是重复的）
								
								if (child4->Value() == string("ImageDimensions"))
								{
									//cout << "Succeed to find label ImageDimensions" << endl;
									for (TiXmlElement *child5 = child4->FirstChildElement(); child5; child5 = child5->NextSiblingElement())
									{
										if (child5->Value() == string("Width")) {
											width = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("Height")) {
											height = std::stold(child5->FirstChild()->Value());
											continue;
										}
									}
									continue;
								}

								if (child4->Value() == string("FocalLengthPixels")) 
								{
									focalLengthPixels = std::stold(child4->FirstChild()->Value());
									continue;
								}
								if (child4->Value() == string("FocalLength"))
								{
									//cout << "Succeed to find label FocalLengthPixels" << endl;
									focalLengthPixels = std::stold(child4->FirstChild()->Value());//*6000/23.5;
									continue;
								}
								if (child4->Value() == string("SensorSize")) {
									sensor_size = std::stod(child4->FirstChild()->Value());
									continue;
								}

								if (child4->Value() == string("PrincipalPoint"))
								{
									//cout << "Succeed to find label PrincipalPoint" << endl;
									for (TiXmlElement *child5 = child4->FirstChildElement(); child5; child5 = child5->NextSiblingElement())
									{
										if (child5->Value() == string("x")) {
											principlePoint_x = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("y")) {
											principlePoint_y = std::stold(child5->FirstChild()->Value());
											continue;
										}
									}
									continue;
								}

								if (child4->Value() == string("Distortion"))
								{
									//cout << "Succeed to find label Distortion" << endl;
									for (TiXmlElement *child5 = child4->FirstChildElement(); child5; child5 = child5->NextSiblingElement())
									{
										if (child5->Value() == string("K1")) {
											k1 = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("K2")) {
											k2 = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("K3")) {
											k3 = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("P1")) {
											p1 = std::stold(child5->FirstChild()->Value());
											continue;
										}
										if (child5->Value() == string("P2")) {
											p2 = std::stold(child5->FirstChild()->Value());
											continue;
										}
									}
									continue;
								}

								if (child4->Value() == string("AspectRatio"))
								{
									//cout << "Succeed to find label AspectRatio" << endl;
									pixelRatio = std::stold(child4->FirstChild()->Value());
									continue;
								}

								if (child4->Value() == string("Skew"))
								{
									//cout << "Succeed to find label Skew" << endl;
									skew = std::stold(child4->FirstChild()->Value());
									continue;
								}

								//在这一层搜集各个image_info类变量的成员变量值后进行push_back  遍历完所有<Photo>得到image_info
								if (child4->Value() == string("Photo"))
								{
									// 补充完影像信息
									if (count_photo == 0) {
										if (fabs(sensor_size) > 1e-6) {
											focalLengthPixels = focalLengthPixels * std::max(width, height) / sensor_size;
										}
									}

									//计数器
									count_photo++;
									cout << "photo_num:" << count_photo << endl;

									//预定义中间变量存储读取的text文本内容
									long double omega, phi, kappa, x, y, z;
									string imagepath;
									int id;
									double M_00, M_01, M_02, M_10, M_11, M_12, M_20, M_21, M_22;

									bool rotation = false;

									for (TiXmlElement *child5 = child4->FirstChildElement(); child5; child5 = child5->NextSiblingElement())
									{
										if (child5->Value() == string("Id")) {
											id = std::stold(child5->FirstChild()->Value());
											continue;
										}

										if (child5->Value() == string("ImagePath")) {
											imagepath = child5->FirstChild()->Value();
											continue;
										}

										if (child5->Value() == string("Pose")) {
											for (TiXmlElement *child6 = child5->FirstChildElement(); child6; child6 = child6->NextSiblingElement())
											{
												if (child6->Value() == string("Rotation"))
												{
													for (TiXmlElement *child7 = child6->FirstChildElement(); child7; child7 = child7->NextSiblingElement())
													{
														//case 1
														if (child7->Value() == string("Omega")) {
															omega = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("Phi")) {
															phi = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("Kappa")) {
															kappa = std::stold(child7->FirstChild()->Value());
															continue;
														}

														//case 2
														if (child7->Value() == string("M_00")) {
															rotation = true;
															M_00 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_01")) {
															M_01 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_02")) {
															M_02 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_10")) {
															M_10 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_11")) {
															M_11 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_12")) {
															M_12 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_20")) {
															M_20 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_21")) {
															M_21 = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("M_22")) {
															M_22 = std::stold(child7->FirstChild()->Value());
															continue;
														}
													}
													continue;
												}

												if (child6->Value() == string("Center"))
												{
													for (TiXmlElement *child7 = child6->FirstChildElement(); child7; child7 = child7->NextSiblingElement())
													{
														if (child7->Value() == string("x")) {
															x = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("y"))	{
															y = std::stold(child7->FirstChild()->Value());
															continue;
														}
														if (child7->Value() == string("z")) {
															z = std::stold(child7->FirstChild()->Value());
															continue;
														}
													}

													continue;
												}

											}
											continue;
										}
									}

									BAImageInfo tempinfo;
									tempinfo.Extrinsics().center_cam[0] = x;
									tempinfo.Extrinsics().center_cam[1] = y;
									tempinfo.Extrinsics().center_cam[2] = z;

									if (!rotation)
									{
										//case 1
// 										tempinfo.Extrinsics().rot_fko[0] = phi;
// 										tempinfo.Extrinsics().rot_fko[1] = kappa;
// 										tempinfo.Extrinsics().rot_fko[2] = omega;
// 
// 										R(0, 0) = cos(Extrinsics().rot_fko[0])*cos(Extrinsics().rot_fko[1]);
// 										R(0, 1) = cos(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[1]) + sin(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[0])*cos(Extrinsics().rot_fko[1]);
// 										R(0, 2) = sin(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[1]) - cos(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[0])*cos(Extrinsics().rot_fko[1]);
// 										R(1, 0) = -cos(Extrinsics().rot_fko[0])*sin(Extrinsics().rot_fko[1]);
// 										R(1, 1) = cos(Extrinsics().rot_fko[2])*cos(Extrinsics().rot_fko[1]) - sin(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[0])*sin(Extrinsics().rot_fko[1]);
// 										R(1, 2) = sin(Extrinsics().rot_fko[2])*cos(Extrinsics().rot_fko[1]) + cos(Extrinsics().rot_fko[2])*sin(Extrinsics().rot_fko[0])*sin(Extrinsics().rot_fko[1]);
// 										R(2, 0) = sin(Extrinsics().rot_fko[0]);
// 										R(2, 1) = -sin(Extrinsics().rot_fko[2])*cos(Extrinsics().rot_fko[0]);
// 										R(2, 2) = cos(Extrinsics().rot_fko[2])*cos(Extrinsics().rot_fko[0]);

										M_00 = cos(phi)*cos(kappa);
										M_01 = cos(omega)*sin(kappa) + sin(omega)*sin(phi)*cos(kappa);
										M_02 = sin(omega)*sin(kappa) - cos(omega)*sin(phi)*cos(kappa);
										M_10 = -cos(phi)*sin(kappa);
										M_11 = cos(omega)*cos(kappa) - sin(omega)*sin(phi)*sin(kappa);
										M_12 = sin(omega)*cos(kappa) + cos(omega)*sin(phi)*sin(kappa);
										M_20 = sin(phi);
										M_21 = -sin(omega)*cos(phi);
										M_22 = cos(omega)*cos(phi);
									}
									//else

									{
										//case 2
										tempinfo.Extrinsics().rot_matirx[0] = M_00;
										tempinfo.Extrinsics().rot_matirx[1] = M_01;
										tempinfo.Extrinsics().rot_matirx[2] = M_02;
										tempinfo.Extrinsics().rot_matirx[3] = M_10;
										tempinfo.Extrinsics().rot_matirx[4] = M_11;
										tempinfo.Extrinsics().rot_matirx[5] = M_12;
										tempinfo.Extrinsics().rot_matirx[6] = M_20;
										tempinfo.Extrinsics().rot_matirx[7] = M_21;
										tempinfo.Extrinsics().rot_matirx[8] = M_22;
									}

									tempinfo.Intrinsics().focal_pix = focalLengthPixels;
									tempinfo.Intrinsics().priciplepoint_img[0] = principlePoint_x;
									tempinfo.Intrinsics().priciplepoint_img[1] = principlePoint_y;

									tempinfo.Intrinsics().k_p[0] = k1;
									tempinfo.Intrinsics().k_p[1] = k2;
									tempinfo.Intrinsics().k_p[2] = k3;
									tempinfo.Intrinsics().k_p[3] = p1;
									tempinfo.Intrinsics().k_p[4] = p2;

									tempinfo.Intrinsics().pixelRatio = pixelRatio;
									tempinfo.Intrinsics().skew = skew;

									tempinfo.Extrinsics().imagepath = imagepath;
									tempinfo.Extrinsics().photo_ID = id;

									tempinfo.Intrinsics().width = width;
									tempinfo.Intrinsics().height = height;

									image_infos.push_back(tempinfo);
									
								}
							}
							cout << "image_infos size: " << image_infos.size() << endl;

							continue;
						}
					}
					
					continue;
				}
			}

			continue;
		}
	}
	*/
	cout << "Succeed to load XML into vector image_infos" << endl;
	return true;
}
#endif // SMART3D_SUPPORT

BAImageInfo::Camera::Camera()
	: focalMm(0.f)
	, viewportPx(baPoint2i(0, 0))
	, pixelSizeMm(baPoint2d(0.0, 0.0))
	, centerPx(baPoint2d(0.0, 0.0))
	, distorCenterPx(baPoint2d(0.0, 0.0))
	, tang_distor(baPoint2d(0.0, 0.0))
{
	std::fill(*k, *k + 4, 0.0);
}

BAImageInfo::ReferenceFrame::ReferenceFrame()
	: tra(0.f, 0.f, 0.f)
{
	std::fill(*rot, *rot + 16, 0.f);
	rot[0] = rot[5] = rot[10] = rot[15] = 1.0f;
}

BAImageInfo::BAImageInfo()
{
}

BAImageInfo::~BAImageInfo()
{
}


baPoint2d BAImageInfo::Camera::distortedToUndistorted(const baPoint2d & p) const
{
	if (fabs(k[0]) < FLT_EPSILON && fabs(k[1]) < FLT_EPSILON) return p;
	///< r^2 = x^2 + y^2
	double rr = p.dot(p);
	///< rp = 1+k1*r^2+k2*r^4+k3*r^6
	double rp = 1.0
		+ k[0] * rr
		+ k[1] * rr * rr
		+ k[2] * rr * rr * rr;

	///< p' = (x', y')
	///< p'.x = p.x * rp + p1*(r^2+2*x^2) + 2*p2*x*y
	///< p'.y = p.y * rp + p2*(r^2+x*y^2) + 2*p1*x*y
	double p1(tang_distor[0]), p2(tang_distor[1]);
	baPoint2d p_;
	p_[0] = p[0] * rp + p1 * (rr + 2. * p[0] * p[0]) + 2. * p2*p[0]*p[1];
	p_[1] = p[1] * rp + p2 * (rr + 2. * p[1] * p[1]) + 2. * p1*p[0]*p[1];
	return p_;
}

baPoint2d BAImageInfo::Camera::undistortedToDistorted(const baPoint2d & p) const
{
	if (fabs(k[0]) < FLT_EPSILON && fabs(k[1]) < FLT_EPSILON) return p;
	baPoint2d p0(*p), pd(*p);
	const size_t iteration_times(20);
	for (size_t i = 0; i < iteration_times; i++) {
		double rr = pd.dot(pd);
		double rp = 1.0
			+ k[0] * rr
			+ k[1] * rr * rr
			+ k[2] * rr * rr * rr;
		double distradial = 1. / rp;
		double p1(tang_distor[0]), p2(tang_distor[1]);
		baPoint2d dist_tangent;
		dist_tangent[0] = p1 * (rr + 2. * pd[0] * pd[0]) + 2. * p2 * pd[0] * pd[1];
		dist_tangent[1] = p2 * (rr + 2. * pd[1] * pd[1]) + 2. * p1 * pd[0] * pd[1];

		baPoint2d pd_ = pd;
		pd = (p0 - dist_tangent)*distradial;
		double dist = (pd_ - pd).dot(pd_ - pd);
		if (dist < 1e-6) {
			break;
		}
	}
	return pd;
}

baPoint2d BAImageInfo::Camera::localToViewportPx(const baPoint2d & p) const
{
	baPoint2d np;
	np[0] = (p[0] / pixelSizeMm[0]) + centerPx[0];
	np[1] = (p[1] / pixelSizeMm[1]) + centerPx[1];
	return np;
}

baPoint2d BAImageInfo::Camera::viewportPxToLocal(const baPoint2d & p) const
{
	baPoint2d ps;
	ps[0] = (p[0] - centerPx[0]) * pixelSizeMm[0];
	ps[1] = (p[1] - centerPx[1]) * pixelSizeMm[1];
	return ps;
}

baPoint3d BAImageInfo::convertWolrdToCameraCoordinates(const baPoint3d & pt_3d) const
{
	baPoint3d p = pt_3d - getViewPoint();
	auto rot = Extrinsics().rot;
	baPoint3d cp;
	cp[0] = rot[0] * p[0] + rot[1] * p[1] + rot[2] * p[2];
	cp[1] = rot[4] * p[0] + rot[5] * p[1] + rot[6] * p[2];
	cp[2] = rot[8] * p[0] + rot[9] * p[1] + rot[10] * p[2];

	cp[2] = -cp[2];
	return cp;
}

baPoint3d BAImageInfo::convertCameraToWolrdCoordinates(const baPoint3d & pt_3d) const
{
	baPoint3d cp(*pt_3d);
	cp[2] = -cp[2];
	auto rot = Extrinsics().rot;

	Eigen::Matrix4d mm, mmi;
	mm.setIdentity();
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			mm(i, j) = rot[i + 4 * j];
	mmi = mm.inverse();
	baPoint3d p;
	p[0] = mmi(0, 0) * cp[0] + mmi(1, 0) * cp[1] + mmi(2, 0) * cp[2];
	p[1] = mmi(0, 1) * cp[0] + mmi(1, 1) * cp[1] + mmi(2, 1) * cp[2];
	p[2] = mmi(0, 2) * cp[0] + mmi(1, 2) * cp[1] + mmi(2, 2) * cp[2];
	p += getViewPoint();
	return p;
}

baPoint2d BAImageInfo::convertWorldToImageCoordinates(const baPoint3d & pt_3d) const
{
/*
	Bundle - this is the original conversion, now it is transfered to the photogrammetry methods
	pinhole camera model; the parameters we estimate for each camera are
	a focal length (f), two radial distortion parameters (k1 and k2),
	a rotation (R), and translation (t). The formula for projecting
	a 3D point X into a camera (R, t, f) is:

	P = R * X + t       (conversion from world to camera coordinates)
	p = -P / P.z        (perspective division)
	p' = f * r(p) * p   (conversion to pixel coordinates)

	r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.

	This gives a projection in pixels, where the origin of the image is
	the center of the image, the positive x-axis points right, and the
	positive y-axis points up (in addition, in the camera coordinate
	system, the positive z-axis points backwards, so the camera is looking
	down the negative z-axis, as in OpenGL).

	Finally, the equations above imply that the camera viewing direction is:
	R' * [0 0 -1]'  (i.e., the third row of R or third column of R')
	(where ' indicates the transpose of a matrix or vector).
	and the 3D position of a camera is
	-R' * t .
*/
/*
	Photogrammetry Camera Coordinate Conversion
	from world coordinate to camera coordinate
	|X|	  |a1 b1 c1|    |XA|   |Xs|
	|Y| = |a2 b2 c2| * (|YA| - |Ys|)
	|Z|   |a3 b3 c3|    |ZA|   |Zs|
	where:
		|a1 b1 c1|     |Xs|
	R = |a2 b2 c2| T = |Ys|
		|a3 b3 c3|     |Zs|
	R - the rotation matrix;
	(XA, YA, ZA) - the object's coordinate in world coordinate system;
	(Xs, Ys, Zs) - the camera's coordinate in world coordinate system.

	for the point with (X, Y, Z) coordinates in the local camera coordinate system,
	the projected coordinates in the image frame can be calculated using the following equations
	x = X / Z;
	y = Y / Z;
	x' = x(1+k1*r^2 + k2*r^4 + k3*r^6) + P1(r^2+2*x^2) + 2*P2*xy;
	y' = y(1+k1*r^2 + k2*r^4 + k3*r^6) + P2(r^2+2*y^2) + 2*P1*xy;
	u = {cx} + x'{fx} + y'{skew};
	v = {cy} + y'{fy};
	where:
	r = sqrt(x^2 + y^2);
	(X, Y, Z) - point coordinates in the local camera coordinate system;
	(u, v) - projected point coordinates in the image coordinate system (in pixels);
	(fx, fy) - focal lengths;
	(cx, cy) - principal point coordinates;
	k1, k2, k3 - radial distortion coefficients;
	p1, p2 - tangential distortion coefficients;
	skew - skew coefficient between the x and the y axis (not used).
*/
	
	///< XYZ = (X, Y, Z)
	baPoint3d XYZ = convertWolrdToCameraCoordinates(pt_3d);
	///< p = (x, y)
	baPoint3d p = XYZ / XYZ[2];
	///< p' = (x', y')
	baPoint2d p_ = Intrinsics().distortedToUndistorted(baPoint2d(*p));

	///< local plane coord
	baPoint2d p_f = p_ * Intrinsics().focalMm;

	///< (u, v) - image coord in pixel
	///< u = cx(pixel)+p'.x * f(mm) / pixelsize(mm)
	///< v = cy(pixel)+p'.y * f(mm) / pixelsize(mm)	
	baPoint2d uv = Intrinsics().localToViewportPx(p_f);

	///< left bottom to left top
	uv[1] = height() - uv[1];
	return uv;
}

baPoint3d BAImageInfo::convertImageToWorldCoordinates(const baPoint2d & pt_2d) const
{
	baPoint2d uv(*pt_2d);
	///< left top to left bottom
	uv[1] = height() - uv[1];

	baPoint2d p_f = Intrinsics().viewportPxToLocal(uv);
	baPoint2d p_ = p_f / Intrinsics().focalMm;

	baPoint2d p = Intrinsics().undistortedToDistorted(p_);
	baPoint3d p_cam(p[0], p[1], 1.f);
	p_cam *= Intrinsics().focalMm;
	return convertCameraToWolrdCoordinates(p_cam);
}

bool BAImageInfo::isInImage(const baPoint2d & p) const
{
	return p[0] >= 0 && p[0] <= width()
		&& p[1] >= 0 && p[1] <= height();
}

bool BAImageInfo::isInImage(const baPoint3d & p, baPoint2d * pt_img) const
{
	baPoint2d pt_2d = convertWorldToImageCoordinates(p);
	if (pt_img) *pt_img = pt_2d;
	return isInImage(pt_2d);
}

const baPoint3d BAImageInfo::getViewDir() const
{
	auto rot = Extrinsics().rot;
	return baPoint3d(-rot[8], -rot[9], -rot[10]);
}

const baPoint3d BAImageInfo::getViewPoint() const
{
	return Extrinsics().tra;
}

int & BAImageInfo::width()
{
	return Intrinsics().viewportPx[0];
}

int BAImageInfo::width() const
{
	return Intrinsics().viewportPx[0];
}

int & BAImageInfo::height()
{
	return Intrinsics().viewportPx[1];
}

int BAImageInfo::height() const
{
	return Intrinsics().viewportPx[1];
}

inline void Dos2Unix(char *strCmd) { size_t len = strlen(strCmd); for (size_t i = 0; i < len; i++) { if (strCmd[i] == '\\') strCmd[i] = '/'; } }

LIBBA_API bool loadImageList(const std::string & image_list, std::vector<BAImageInfo> & image_infos)
{
	FILE *fp = fopen(image_list.c_str(), "r");
	if (!fp) {
		std::cerr << "Cannot open file : " << image_list;
		return false;
	}
	char file_dir[256];
	strcpy(file_dir, image_list.c_str()); Dos2Unix(file_dir);
	char *pS = strrchr(file_dir, '/') + 1;
	if (!pS) {
		std::cerr << "Wrong list file path : " << file_dir;
		return false;
	}
	strcpy(pS, "");

	image_infos.clear();

	char lbuf[1024];
	char image_path_in_list[256];
	int index = 0;
	while (fgets(lbuf, 1024, fp))
	{
		int width(-1), height(-1);
		char image_path[256];
		sscanf(lbuf, "%s%d%d", image_path_in_list, &width, &height);
		sprintf_s(image_path, "%s", image_path_in_list);
		if (_access(image_path, 0) == -1) {
			sprintf_s(image_path, "%s%s", file_dir, image_path_in_list);
			if (_access(image_path, 0) == -1) {
				std::cerr << "Cannot find the image file: " << image_path;
			}
		}
		if (width <= 0 || height <= 0) {
			continue;
		}

		char filename[_MAX_FNAME]; char ext[_MAX_ENV];
		_splitpath(image_path, NULL, NULL, filename, ext);

		BAImageInfo image_info;
		image_info.m_name = std::string(filename) + std::string(ext);
		image_info.m_path = image_path;
		image_info.width() = width;
		image_info.height() = height;

		image_infos.push_back(image_info);
	}
	fclose(fp);
	return true;
}

LIBBA_API bool loadBaInfoCVFile(const std::string & file_path, std::vector<BAImageInfo>& image_infos)
{
	std::ifstream fin;
	fin.open(file_path.c_str());
	if (!fin.is_open()) {
		std::cerr << "Cannot open file : " << file_path;
		return false;
	}
	std::string lbuf;
	size_t cam_num(0), img_num(0);
	getline(fin, lbuf);	/// --------------------------------Camera Information--------------------------------
	getline(fin, lbuf);	/// CameraID	x0 y0 f	format_X format_Y pixelSize	k0 k1 k2 k3	p1 p2 b1 b2 Attrib
	getline(fin, lbuf);	/// camera sum
	sscanf(lbuf.c_str(), "%d", &cam_num);

	struct Cam_Intri_Input {
		double x0, y0, f, format_X, format_Y, pixelSize, k0, k1, k2, k3, p1, p2;
	};
	std::map<int, Cam_Intri_Input> map_index_cam;
	for (size_t i = 0; i < cam_num; i++) {
		Cam_Intri_Input _cam;
		int index;
		getline(fin, lbuf);	std::stringstream sbuf(lbuf);
		sbuf >> index >> _cam.x0 >> _cam.y0 >> _cam.f >> _cam.format_X >> _cam.format_Y >> _cam.pixelSize >> _cam.k0 >> _cam.k1 >> _cam.k2 >> _cam.k3 >> _cam.p1 >> _cam.p2;
		map_index_cam[index] = _cam;
	}

	getline(fin, lbuf);	/// 注：采用CV畸变系数
	getline(fin, lbuf);	/// --------------------------------Image Information--------------------------------
	getline(fin, lbuf);	/// ImageID ImageName Xs Ys Zs Phi Omega Kappa StripID Attrib CameraID bFlag BlockID 
	getline(fin, lbuf);	/// image sum
	sscanf(lbuf.c_str(), "%d", &img_num);

	std::vector<BAImageInfo> image_info_valid;
	image_info_valid.reserve(image_infos.size());

	if (img_num != image_infos.size()) {
		std::cerr << "WARNING: image number does not match the image list: " << file_path;
	}
	for (size_t i = 0; i < img_num; i++) {
		int cam_index; double Xs, Ys, Zs, Phi, Omega, Kappa; std::string image_name;
		int img_index, strip_id, attrib;	/// useless
		getline(fin, lbuf);	std::stringstream sbuf(lbuf);
		sbuf >> img_index >> image_name >> Xs >> Ys >> Zs >> Phi >> Omega >> Kappa >> strip_id >> attrib >> cam_index;
		std::string name = image_name;
		//! get image in project
		auto _img = std::find_if(image_infos.begin(), image_infos.end(), [=](BAImageInfo img) {
			return name == img.m_name;
		});
		if (_img == image_infos.end()) {
			std::cerr << "cannot find image " << name << std::endl;
			continue;
		}
		auto& cam_intri = (*_img).Intrinsics();

		//! Intrinsic
		if (map_index_cam.find(cam_index) == map_index_cam.end()) {
			std::cerr << "invalid camera number" << std::endl;
			return false;
		}
		Cam_Intri_Input intri = map_index_cam[cam_index];
		//cam_intri.viewportPx = ;
		cam_intri.centerPx = baPoint2d((intri.x0 + intri.format_X / 2.), (intri.y0 + intri.format_Y / 2.)) / intri.pixelSize;
		cam_intri.focalMm = intri.f;
		cam_intri.pixelSizeMm = baPoint2d(intri.pixelSize, intri.pixelSize);
		cam_intri.k[0] = intri.k1;
		cam_intri.k[1] = intri.k2;
		cam_intri.k[2] = intri.k3;
		cam_intri.tang_distor[0] = intri.p1;
		cam_intri.tang_distor[1] = intri.p2;

		//! Extrinsic
		baPoint3d a, b, c;

		double a1 = cos(Phi)*cos(Kappa) - sin(Phi)*sin(Omega)*sin(Kappa);
		double a2 = -cos(Phi)*sin(Kappa) - sin(Phi)*sin(Omega)*cos(Kappa);
		double a3 = -sin(Phi)*cos(Omega);
		a = baPoint3d(a1, a2, a3);

		double b1 = cos(Omega)*sin(Kappa);
		double b2 = cos(Omega)*cos(Kappa);
		double b3 = -sin(Omega);
		b = baPoint3d(b1, b2, b3);

		double c1 = sin(Phi)*cos(Kappa) + cos(Phi)*sin(Omega)*sin(Kappa);
		double c2 = -sin(Phi)*sin(Kappa) + cos(Phi)*sin(Omega)*cos(Kappa);
		double c3 = cos(Phi)*cos(Omega);
		c = baPoint3d(c1, c2, c3);

		auto & img_extri = (*_img).Extrinsics();
		auto & rot = img_extri.rot;
		rot[0] = a[0]; rot[4] = a[1]; rot[8] = a[2];
		rot[1] = b[0]; rot[5] = b[1]; rot[9] = b[2];
		rot[2] = c[0]; rot[6] = c[1]; rot[10] = c[2];

		img_extri.tra = baPoint3d(Xs, Ys, Zs);

		image_info_valid.push_back(*_img);
	}
	std::swap(image_info_valid, image_infos);

	fin.close();
	//std::cout << "bundle file loaded from: " << file_path << std::endl;
	return true;
}

LIBBA_API bool loadBaInfoCVFiles(const std::string & image_list, const std::string & file_path, std::vector<BAImageInfo>& image_infos)
{
	if (!loadImageList(image_list, image_infos)) {
		return false;
	}
	return loadBaInfoCVFile(file_path, image_infos);
}

LIBBA_API bool loadBundleOutFile(const std::string & file_path, std::vector<BAImageInfo>& image_infos)
{
/*
	# Bundle file v0.3
	<num_cameras> <num_points>[two integers]
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

	Each point entry <pointI> has the form:
	<position>      [a 3-vector describing the 3D position of the point]
	<color>         [a 3-vector describing the RGB color of the point]
	<view list>     [a list of views the point is visible in]
*/

	std::ifstream ifstr;
	ifstr.open(file_path.c_str());
	if (!ifstr.is_open()) {
		std::cerr << "Cannot open file : " << file_path << std::endl;
		return false;
	}
	std::string lbuf;
	getline(ifstr, lbuf);	/// # Bundle file v0.3
	size_t img_num = 0, point_num = 0;
	ifstr >> img_num >> point_num;
	if (img_num != image_infos.size()) {
		std::cerr << "image number does not match the image list!" << file_path << std::endl;
		return false;
	}
	for (auto& _img : image_infos) {
		auto& cmr_intri = _img.Intrinsics();
		double f, k1, k2;
		ifstr >> f >> k1 >> k2;
		cmr_intri.focalMm = f;
		cmr_intri.k[0] = k1;
		cmr_intri.k[1] = k2;

		//cmr_intri.viewportPx = baPoint2i(_img.width, _img.height);
		cmr_intri.centerPx[0] = (int)((double)cmr_intri.viewportPx[0] / 2.0f);
		cmr_intri.centerPx[1] = (int)((double)cmr_intri.viewportPx[1] / 2.0f);
		cmr_intri.pixelSizeMm = baPoint2d(1.f, 1.f);

		auto & rot = _img.Extrinsics().rot;
		ifstr >> rot[0] >> rot[1] >> rot[2]
			>> rot[4] >> rot[5] >> rot[6]
			>> rot[8] >> rot[9] >> rot[10];

		///< T = -R^T * T
		auto & tra = _img.Extrinsics().tra;
		ifstr >> tra[0] >> tra[1] >> tra[2];
		double t0 = rot[0] * tra[0] + rot[4] * tra[1] + rot[8] * tra[2];
		double t1 = rot[1] * tra[0] + rot[5] * tra[1] + rot[9] * tra[2];
		double t2 = rot[2] * tra[0] + rot[6] * tra[1] + rot[10] * tra[2];
		tra = baPoint3d(-t0, -t1, -t2);
	}
	ifstr.close();
	return true;
}

LIBBA_API bool loadBundleOutFiles(const std::string & image_list, const std::string & sfm_out, std::vector<BAImageInfo> & image_infos)
{
	if (!loadImageList(image_list, image_infos)) {
		return false;
	}
	return loadBundleOutFile(sfm_out, image_infos);
}

LIBBATOOLS_NAMESPACE_END
