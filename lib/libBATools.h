#ifndef __LIB_BATOOLS_H__
#define __LIB_BATOOLS_H__

#include <vector>

#ifndef LIBBATOOLS_NAMESPACE_BEGIN
#define LIBBATOOLS_NAMESPACE_BEGIN namespace libba {
#endif
#ifndef LIBBATOOLS_NAMESPACE_END
#define LIBBATOOLS_NAMESPACE_END }
#endif

LIBBATOOLS_NAMESPACE_BEGIN

#define USE_BATOOLS_AS_DLL

#if defined(USE_BATOOLS_AS_DLL) && defined(_WIN32)
#ifdef libBATools_EXPORTS
#define LIBBA_API __declspec(dllexport)
#else 
#define LIBBA_API __declspec(dllimport)
#endif // libBATools_EXPORTS
#else
#define LIBBA_API
#endif 

#ifndef _TVEC
#define _TVEC
template <typename T, int N> 
class TVec
{
public:
	/// constructors & implements
	TVec(void) {}
	explicit TVec(T const* values) { std::copy(values, values + N, v); return *this; }
	explicit TVec(T const& value) { std::fill(v, v + N, value); return *this; }
	TVec(T const& v1, T const& v2) { v[0] = v1; v[1] = v2; }
	TVec(T const& v1, T const& v2, T const& v3) { v[0] = v1; v[1] = v2; v[2] = v3; }
	
	T& operator[] (int index) { return v[index]; }
	T const& operator[] (int index) const { return v[index]; }
protected:
	T v[N];
};

#endif // _TVEC

typedef TVec<double, 2> baPoint2d;
typedef TVec<double, 3> baPoint3d;

class LIBBA_API BAImageInfo
{
public:
	BAImageInfo();
	~BAImageInfo();
public:
	//! Left Top
	baPoint2d getImagePoint(baPoint3d pt);
protected:

};

LIBBA_API bool loadInfoCVFile(const std::string & file_path, std::vector<BAImageInfo> & image_infos);
LIBBA_API bool loadBundleFiles(const std::string & image_list, const std::string & sfm_out, std::vector<BAImageInfo> & image_infos);


LIBBATOOLS_NAMESPACE_END
#endif // !__LIB_BATOOLS_H__