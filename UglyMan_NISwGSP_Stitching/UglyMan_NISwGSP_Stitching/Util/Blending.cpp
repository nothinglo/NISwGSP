//
//  Blending.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Blending.h"

Mat getMatOfLinearBlendWeight(const Mat & image) {
    Mat result(image.size(), CV_32FC1, Scalar::all(0));
    for(int y = 0; y < result.rows; ++y) {
        int w_y = min(y + 1, result.rows - y);
        for(int x = 0; x < result.cols; ++x) {
            result.at<float>(y, x) = min(x + 1, result.cols - x) * w_y;
        }
    }
    return result;
}

vector<Mat> getMatsLinearBlendWeight(const vector<Mat> & images) {
    vector<Mat> result;
    result.reserve(images.size());
    for(int i = 0; i < images.size(); ++i) {
        result.emplace_back(getMatOfLinearBlendWeight(images[i]));
    }
    return result;
}

Mat Blending(const vector<Mat> & images,
             const vector<Point2> & origins,
             const Size2 target_size,
             const vector<Mat> & weight_mask,
             const bool ignore_weight_mask) {
    
    Mat result = Mat::zeros(round(target_size.height), round(target_size.width), CV_8UC4);
    
    vector<Rect2> rects;
    rects.reserve(origins.size());
    for(int i = 0; i < origins.size(); ++i) {
        rects.emplace_back(origins[i], images[i].size());
    }
    for(int y = 0; y < result.rows; ++y) {
        for(int x = 0; x < result.cols; ++x) {
            Point2i p(x, y);
            Vec3f pixel_sum(0, 0, 0);
            float weight_sum = 0.f;
            for(int i = 0; i < rects.size(); ++i) {
                Point2i pv(round(x - origins[i].x), round(y - origins[i].y));
                if(pv.x >= 0 && pv.x < images[i].cols &&
                   pv.y >= 0 && pv.y < images[i].rows) {
                    Vec4b v = images[i].at<Vec4b>(pv);
                    Vec3f value = Vec3f(v[0], v[1], v[2]);
                    if(ignore_weight_mask) {
                        if(v[3] > 127) {
                            pixel_sum += value;
                            weight_sum += 1.f;
                        }
                    } else {
                        float weight = weight_mask[i].at<float>(pv);
                        pixel_sum += weight * value;
                        weight_sum += weight;
                    }
                }
            }
            if(weight_sum) {
                pixel_sum /= weight_sum;
                result.at<Vec4b>(p) = Vec4b(round(pixel_sum[0]), round(pixel_sum[1]), round(pixel_sum[2]), 255);
            }
        }
    }
    return result;
}