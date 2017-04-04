//
//  ImageData.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "ImageData.h"

LineData::LineData(const Point2 & _a,
                   const Point2 & _b,
                   const double _width,
                   const double _length) {
    data[0] = _a;
    data[1] = _b;
    width   = _width;
    length  = _length;
}

const bool LINES_FILTER_NONE(const double _data,
                             const Statistics & _statistics) {
    return true;
};

const bool LINES_FILTER_WIDTH (const double _data,
                               const Statistics & _statistics) {
    return _data >= MAX(2.f, (_statistics.min + _statistics.mean) / 2.f);
    return true;
};

const bool LINES_FILTER_LENGTH(const double _data,
                               const Statistics & _statistics) {
    return _data >= MAX(10.f, _statistics.mean);
    return true;
};


ImageData::ImageData(const string & _file_dir,
                     const string & _file_full_name,
                     LINES_FILTER_FUNC * _width_filter,
                     LINES_FILTER_FUNC * _length_filter,
                     const string * _debug_dir) {
    
    file_dir = &_file_dir;
    std::size_t found = _file_full_name.find_last_of(".");
    assert(found != std::string::npos);
    file_name = _file_full_name.substr(0, found);
    file_extension = _file_full_name.substr(found);
    debug_dir = _debug_dir;

    grey_img = Mat();
    
    width_filter  = _width_filter;
    length_filter = _length_filter;
    
    img = imread(*file_dir + file_name + file_extension);
    rgba_img = imread(*file_dir + file_name + file_extension, IMREAD_UNCHANGED);
    
    float original_img_size = img.rows * img.cols;
    
    if(original_img_size > DOWN_SAMPLE_IMAGE_SIZE) {
        float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / original_img_size);
        resize(img, img, Size(), scale, scale);
        resize(rgba_img, rgba_img, Size(), scale, scale);
    }
    
    assert(rgba_img.channels() >= 3);
    if(rgba_img.channels() == 3) {
        cvtColor(rgba_img, rgba_img, CV_BGR2BGRA);
    }
    vector<Mat> channels;
    split(rgba_img, channels);
    alpha_mask = channels[3];
    mesh_2d = make_unique<MeshGrid>(img.cols, img.rows);
}

const Mat & ImageData::getGreyImage() const {
    if(grey_img.empty()) {
        cvtColor(img, grey_img, CV_BGR2GRAY);
    }
    return grey_img;
}

const vector<LineData> & ImageData::getLines() const {
    if(img_lines.empty()) {
        const Mat & grey_image = getGreyImage();
        Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
        
        vector<Vec4f>  lines;
        vector<double> lines_width, lines_prec, lines_nfa;
        ls->detect(grey_image, lines, lines_width, lines_prec, lines_nfa);
        
        vector<double> lines_length;
        vector<Point2> lines_points[2];
        
        const int line_count = (int)lines.size();
        
        lines_length.reserve(line_count);
        lines_points[0].reserve(line_count);
        lines_points[1].reserve(line_count);
        
        for(int i = 0; i < line_count; ++i) {
            lines_points[0].emplace_back(lines[i][0], lines[i][1]);
            lines_points[1].emplace_back(lines[i][2], lines[i][3]);
            lines_length.emplace_back(norm(lines_points[1][i] - lines_points[0][i]));
        }
        
        const Statistics width_statistics(lines_width), length_statistics(lines_length);
        for(int i = 0; i < line_count; ++i) {
            if( width_filter( lines_width[i],  width_statistics) &&
               length_filter(lines_length[i], length_statistics)) {
                img_lines.emplace_back(lines_points[0][i],
                                       lines_points[1][i],
                                       lines_width[i],
                                       lines_length[i]);
            }
        }
#ifndef NDEBUG
        vector<Vec4f> draw_lines;
        draw_lines.reserve(img_lines.size());
        for(int i = 0; i < img_lines.size(); ++i) {
            draw_lines.emplace_back(img_lines[i].data[0].x, img_lines[i].data[0].y,
                                    img_lines[i].data[1].x, img_lines[i].data[1].y);
        }
        Mat canvas = Mat::zeros(grey_image.rows, grey_image.cols, grey_image.type());
        ls->drawSegments(canvas, draw_lines);
        imwrite(*debug_dir + "line-result-" + file_name + file_extension, canvas);
#endif
    }
    return img_lines;
}

const vector<Point2> & ImageData::getFeaturePoints() const {
    if(feature_points.empty()) {
        FeatureController::detect(getGreyImage(), feature_points, feature_descriptors);
    }
    return feature_points;
}
const vector<FeatureDescriptor> & ImageData::getFeatureDescriptors() const {
    if(feature_descriptors.empty()) {
        FeatureController::detect(getGreyImage(), feature_points, feature_descriptors);
    }
    return feature_descriptors;
}

void ImageData::clear() {
    img.release();
    grey_img.release();
    img_lines.clear();
    feature_points.clear();
    feature_descriptors.clear();
}