#pragma once
#include "libBATools.h"
#include <algorithm>
#include <numeric>

LIBBATOOLS_NAMESPACE_BEGIN

template<typename T, int N>
inline 
TVec<T, N>::TVec(T const * values) {	std::copy(values, values + N, v); }

template<typename T, int N>
inline 
TVec<T, N>::TVec(T const & values) { std::fill(v, v + N, values); }

template<typename T, int N>
inline
TVec<T, N>::TVec(T const & v1, T const & v2) { v[0] = v1; v[1] = v2; }

template<typename T, int N>
inline
TVec<T, N>::TVec(T const & v1, T const & v2, T const & v3) { v[0] = v1; v[1] = v2; v[2] = v3; }

template <typename T, int N>
inline T*
TVec<T, N>::operator* (void) { return v; }

template <typename T, int N>
inline T const*
TVec<T, N>::operator* (void) const { return v; }

template <typename T, int N>
inline T&
TVec<T, N>::operator[] (int index) { return v[index]; }

template <typename T, int N>
inline T const&
TVec<T, N>::operator[] (int index) const { return v[index]; }

template <typename T, int N>
inline TVec<T, N>&
TVec<T, N>::operator+= (T const& rhs) { std::for_each(v, v + N, [=](T & _v) {_v += rhs; });	return *this; }

template <typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator+ (T const& rhs) const { return TVec<T, N>(*this) += rhs; }

template <typename T, int N>
inline TVec<T, N>&
TVec<T, N>::operator-= (T const& rhs) { std::for_each(v, v + N, [=](T & _v) {_v -= rhs; });	return *this; }

template <typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator- (T const& rhs) const { return TVec<T, N>(*this) -= rhs; }

template<typename T, int N>
inline TVec<T, N> TVec<T, N>::operator-() const { return (*this)*(-1); }

template <typename T, int N>
inline TVec<T, N>&
TVec<T, N>::operator/= (T const& rhs) { std::for_each(v, v + N, [&](T & _v) {_v /= rhs; });	return *this; }

template <typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator/ (T const& rhs) const { return TVec<T, N>(*this) /= rhs; }

template <typename T, int N>
inline TVec<T, N>&
TVec<T, N>::operator*= (T const& rhs) { std::for_each(v, v + N, [&](T & _v) {_v *= rhs; });	return *this; }

template <typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator* (T const& rhs) const { return TVec<T, N>(*this) *= rhs; }

template<typename T, int N>
inline TVec<T, N>& 
TVec<T, N>::operator-=(TVec<T, N> const & rhs) { std::transform(v, v + N, *rhs, v, std::minus<T>()); return *this; }

template<typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator-(TVec<T, N> const & rhs) const { return TVec<T, N>(*this) -= rhs; }

template<typename T, int N>
inline TVec<T, N>&
TVec<T, N>::operator+=(TVec<T, N> const & rhs) { std::transform(v, v + N, *rhs, v, std::plus<T>()); return *this; }

template<typename T, int N>
inline TVec<T, N>
TVec<T, N>::operator+(TVec<T, N> const & rhs) const { return TVec<T, N>(*this) -= rhs; }

template<typename T, int N>
inline T
TVec<T, N>::dot(TVec<T, N> const & other) const { return std::inner_product(v, v + N, *other, T(0)); }

LIBBATOOLS_NAMESPACE_END
