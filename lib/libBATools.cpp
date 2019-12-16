#include "libBATools.h"

LIBBATOOLS_NAMESPACE_BEGIN

BAImageInfo::BAImageInfo()
{
}

BAImageInfo::~BAImageInfo()
{
}

LIBBA_API bool loadInfoCVFile(const std::string & file_path, std::vector<BAImageInfo>& image_infos)
{
	return true;
}

LIBBA_API bool loadBundleFiles(const std::string & image_list, const std::string & sfm_out, std::vector<BAImageInfo> & image_infos)
{
	return true;
}

LIBBATOOLS_NAMESPACE_END
