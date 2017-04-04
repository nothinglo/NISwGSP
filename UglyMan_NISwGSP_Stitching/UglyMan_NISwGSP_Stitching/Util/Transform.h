//
//  Transform.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__Transform__
#define __UglyMan_Stitiching__Transform__

#include "Configure.h"

Mat getConditionerFromPts(const vector<Point2> & pts);

Mat getNormalize2DPts(const vector<Point2> & pts,
                      vector<Point2> & newpts);

template <typename T>
T normalizeAngle(T x);
                 
template <typename T>
Point_<T> applyTransform3x3(T x, T y, const Mat & matT);

template <typename T>
Point_<T> applyTransform2x3(T x, T y, const Mat & matT);

template <typename T>
Size_<T> normalizeVertices(vector<vector<Point_<T> > > & vertices);

template <typename T>
Rect_<T> getVerticesRects(const vector<Point_<T> > & vertices);

template <typename T>
vector<Rect_<T> > getVerticesRects(const vector<vector<Point_<T> > > & vertices);

template <typename T>
T getSubpix(const Mat & img, const Point2f & pt);

template <typename T, size_t n>
Vec<T, n> getSubpix(const Mat & img, const Point2f & pt);

template <typename T>
Vec<T, 3> getEulerZXYRadians(const Mat_<T> & rot_matrix);

template <typename T>
bool isEdgeIntersection(const Point_<T> & src_1, const Point_<T> & dst_1,
                        const Point_<T> & src_2, const Point_<T> & dst_2,
                        double * scale_1 = NULL, double * scale_2 = NULL);
template <typename T>
bool isRotationInTheRange(const T rotation, const T min_rotation, const T max_rotation);

#endif /* defined(__UglyMan_Stitiching__Transform__) */
