#include "libBATools.h"
#include <tchar.h>
#include <iostream>

bool testProjection(const libba::BAImageInfo & image, const libba::baPoint3d & pt_world, libba::baPoint2d & pt_img)
{
	return image.isInImage(pt_world, &pt_img);
}

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

	libba::baPoint2d image_point;
	if (!testProjection(image_data[0], libba::baPoint3d(0, 0, 0), image_point)) {
		std::cout << "point out of range" << std::endl;
	}

	return 0;
}
