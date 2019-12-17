#include "libBATools.h"
#include <tchar.h>
#include <iostream>

#ifdef USE_VCGLIB
#include "vcg/space/intersection3.h"
#endif // USE_VCGLIB

bool testProjection(const libba::BAImageInfo & image, const libba::baPoint3d & pt_world, libba::baPoint2d & pt_img)
{
	return image.isInImage(pt_world, &pt_img);
}

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

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc < 3) {
		return false;
	}

	std::string bundle_out = argv[1];
	std::string image_list = argv[2];

	std::vector<libba::BAImageInfo> image_data;
	if (!loadBundleOutFiles(image_list, bundle_out, image_data)) {
		std::cerr << "failed to load image bundle files" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << image_data.size() << " images loaded!" << std::endl;

	libba::baPoint2d image_point;
	if (!testProjection(image_data[0], libba::baPoint3d(316384.156250, 234354.078125,55.636), image_point)) {
		std::cout << "point out of range" << std::endl;
	}
	
#ifdef USE_VCGLIB
	vcg::Plane3d plane;
	plane.Set(vcg::Point3d(0, 0, 1), 55);
	libba::baPoint3d pt_world;
	if (!testBackProjection(image_data[0], libba::baPoint2d(4996, 2958), plane, pt_world)) {
		std::cout << "intersection failed" << std::endl;
	}
#endif // USE_VCGLIB

	return EXIT_SUCCESS;
}
