//
//  ImageData.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__ImageData__
#define __UglyMan_Stitiching__ImageData__

#include <memory>
#include "Statistics.h"
#include "FeatureController.h"
#include "MeshGrid.h"

class LineData {
public:
    LineData(const Point2 & _a,
             const Point2 & _b,
             const double _width,
             const double _length);
    Point2 data[2];
    double width, length;
private:
};

typedef const bool (LINES_FILTER_FUNC)(const double _data, \
                                       const Statistics & _statistics);

LINES_FILTER_FUNC LINES_FILTER_NONE;
LINES_FILTER_FUNC LINES_FILTER_WIDTH;
LINES_FILTER_FUNC LINES_FILTER_LENGTH;


class ImageData {
public:
    string file_name, file_extension;
    const string * file_dir, * debug_dir;
    ImageData(const string & _file_dir,
              const string & _file_full_name,
              LINES_FILTER_FUNC * _width_filter,
              LINES_FILTER_FUNC * _length_filter,
              const string * _debug_dir = NULL);
    
    const Mat & getGreyImage() const;
    const vector<LineData> & getLines() const;
    const vector<Point2> & getFeaturePoints() const;
    const vector<FeatureDescriptor> & getFeatureDescriptors() const;
    
    void clear();
    
    Mat img, rgba_img, alpha_mask;
    unique_ptr<Mesh2D> mesh_2d;
    
private:
    LINES_FILTER_FUNC * width_filter, * length_filter;
    
    mutable Mat grey_img;
    mutable vector<LineData> img_lines;
    mutable vector<Point2> feature_points;
    mutable vector<FeatureDescriptor> feature_descriptors;
};

#endif /* defined(__UglyMan_Stitiching__ImageData__) */
