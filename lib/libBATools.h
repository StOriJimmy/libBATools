/*
===============================================================================

  @FILE:  libBATools.hpp
  @Brief: parse image bundle adjustment results, coordinates transfer
  
  @Author: Xinyi Liu
  @Date:   15 December 2019
  @E-mail: liuxy0319@whu.edu.cn

  CHANGE HISTORY:
	16 December 2019 -- created and first commit by Xinyi Liu

===============================================================================
*/

#ifndef __LIB_BATOOLS_H__
#define __LIB_BATOOLS_H__

#include <vector>
#include <string>

#ifndef LIBBATOOLS_NAMESPACE_BEGIN
#define LIBBATOOLS_NAMESPACE_BEGIN namespace libba {
#endif
#ifndef LIBBATOOLS_NAMESPACE_END
#define LIBBATOOLS_NAMESPACE_END }
#endif

LIBBATOOLS_NAMESPACE_BEGIN

#if !defined (USE_BATOOLS_AS_LIB) && defined(_WIN32)
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
class LIBBA_API TVec
{
public:
	/// constructors & implements
	TVec(void) {}
	explicit TVec(T const* values);
	explicit TVec(T const& value);
	TVec(T const& v1, T const& v2);
	TVec(T const& v1, T const& v2, T const& v3);
	
	T*			operator* (void);
	T const*	operator* (void) const;
	T&			operator[] (int index);
	T const&	operator[] (int index) const;

	TVec<T, N>& operator+= (T const & rhs);
	TVec<T, N>	operator+ (T const & rhs) const;
	TVec<T, N>& operator-= (T const & rhs);
	TVec<T, N>	operator- (T const & rhs) const;
	TVec<T, N>	operator- () const;
	TVec<T, N>& operator/= (T const & rhs);
	TVec<T, N>	operator/ (T const & rhs) const;
	TVec<T, N>& operator*= (T const & rhs);
	TVec<T, N>	operator* (T const & rhs) const;
		
	TVec<T, N>& operator-= (TVec<T, N> const& rhs);
	TVec<T, N>	operator- (TVec<T, N> const& rhs) const;
	TVec<T, N>& operator+= (TVec<T, N> const& rhs);
	TVec<T, N>	operator+ (TVec<T, N> const& rhs) const;
	
	T dot(TVec<T, N> const& other) const;
	void normalize() const;
protected:
	T v[N];
};

#endif // _TVEC

template class __declspec(dllexport) TVec<int, 2>;
template class __declspec(dllexport) TVec<double, 2>;
template class __declspec(dllexport) TVec<double, 3>;

typedef TVec<int, 2>	baPoint2i;
typedef TVec<double, 2> baPoint2d;
typedef TVec<double, 3> baPoint3d;
typedef TVec<double, 16> baMatrix44d;

LIBBA_API inline libba::baPoint3d cross_p3(libba::baPoint3d a, libba::baPoint3d b) {
	return { a[1] * b[2] - a[2] * b[1],a[2] * b[0] - a[0] * b[2],a[0] * b[1] - a[1] * b[0] };
}

class LIBBA_API BAImageInfo
{
public:
	BAImageInfo();
	~BAImageInfo();

	//! PERSPECTIVE
	class Camera
	{
	public:
		Camera();

		baPoint2d distortedToUndistorted(const baPoint2d & p) const;
		baPoint2d undistortedToDistorted(const baPoint2d & p) const;
		baPoint2d localToViewportPx(const baPoint2d & p) const;
		baPoint2d viewportPxToLocal(const baPoint2d & p) const;

		double focalMm;				///< Focal Distance: the distance between focal center and image plane. Expressed in mm
		baPoint2i viewportPx;		///< Dimension of the Image Plane (in pixels)
		baPoint2d pixelSizeMm;		///< Dimension in mm of a single pixel
		baPoint2d centerPx;			///< Position of the projection of the focal center on the image plane. Expressed in pixels
		baPoint2d distorCenterPx;	///< Position of the radial distortion center on the image plane in pixels
		TVec<double, 4> k;			///< 1st & 2nd order radial lens distortion coefficient (only the first 2 terms are used)
		baPoint2d tang_distor;		///< p1,p2 1st & 2nd order tangent distortion coefficient
	};

	class ReferenceFrame
	{
	public:
		ReferenceFrame();
		~ReferenceFrame() {}

		baPoint3d tra;
		/*
			0 1 2 3
			4 5 6 7
			8 9 10 11
			12 13 14 15
		*/
		baMatrix44d rot;
	};

	typedef Camera			CameraIntr;
	typedef ReferenceFrame	CameraExtr;

public:
		
	baPoint3d convertWolrdToCameraCoordinates(const baPoint3d & pt_3d) const;
	baPoint3d convertCameraToWolrdCoordinates(const baPoint3d & pt_3d) const;

	baPoint2d convertWorldToImageCoordinates(const baPoint3d & pt_3d) const;	///< left top
	baPoint3d convertImageToWorldCoordinates(const baPoint2d & pt_2d) const;

	/*baPoint3d convertWGS84_ENU_to_LLA(const baPoint3d & enu_3d) const;
	baPoint2d convertWGS84_LLA_to_UTM(const baPoint3d & lla) const;*/



	/// @input: p, point in image coordinate
	bool isInImage(const baPoint2d & p) const;
	/// @input: p, point in world coordinate
	/// @output: pt_img, projected point in image coordinate
	bool isInImage(const baPoint3d & p, baPoint2d * pt_img = nullptr) const;

	/// viewing dir
	const baPoint3d getViewDir() const;
	/// camera center
	const baPoint3d getViewPoint() const;

	int &width();
	int width() const;
	int &height();
	int height() const;

	CameraIntr	&Intrinsics() { return m_intrinsics; }
	CameraIntr	Intrinsics() const { return m_intrinsics; }
	CameraExtr	&Extrinsics() { return m_extrinsics; }
	CameraExtr	Extrinsics() const { return m_extrinsics; }

public:
	std::string m_name;
	std::string m_path;

protected:
	CameraIntr m_intrinsics;
	CameraExtr m_extrinsics;
};



//! image list, each line: image_path, width, height
LIBBA_API bool loadImageList(const std::string & image_list, std::vector<BAImageInfo>& image_infos);

//! load .info, given image_infos loaded from imagelist (invalid images will be excluded)
LIBBA_API bool loadBaInfoCVFile(const std::string & file_path, std::vector<BAImageInfo> & image_infos);

//! load .info + imagelist.txt
LIBBA_API bool loadBaInfoCVFiles(const std::string & image_list, const std::string & file_path, std::vector<BAImageInfo> & image_infos);

//! load bundle.out, given image_infos loaded from imagelist
LIBBA_API bool loadBundleOutFile(const std::string & file_path, std::vector<BAImageInfo> & image_infos);

//! load bundle.out + imagelist.txt
LIBBA_API bool loadBundleOutFiles(const std::string & image_list, const std::string & sfm_out, std::vector<BAImageInfo> & image_infos);

////////////////////////////   func for smart3d   /////////////////////////////////
//! load xml, GET image_infos loaded from imagelist
LIBBA_API bool loadSmart3dXML(const char * ccXmlName, std::vector<BAImageInfo>& image_infos);

LIBBATOOLS_NAMESPACE_END
#endif // !__LIB_BATOOLS_H__


