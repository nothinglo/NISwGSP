//
//  ImageDebugger.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "ImageDebugger.h"

Mat getImageOfFeaturePairs(const Mat & img1,
                           const Mat & img2,
                           const vector<Point2> & f1,
                           const vector<Point2> & f2) {
    assert(f1.size() == f2.size());
    assert(img1.type() == img2.type());
    
    const int CIRCLE_RADIUS    = 5;
    const int CIRCLE_THICKNESS = 1;
    const int LINE_THICKNESS   = 1;
    const int RGB_8U_RANGE     = 256;
    
    Mat result = Mat::zeros(max(img1.rows, img2.rows), img1.cols + img2.cols, CV_8UC3);
    Mat left (result, Rect(0, 0, img1.cols, img1.rows));
    Mat right(result, Rect(img1.cols, 0, img2.cols, img2.rows));
    
    Mat img1_8UC3, img2_8UC3;
    
    if(img1.type() == CV_8UC3) {
        img1_8UC3 = img1;
        img2_8UC3 = img2;
    } else {
        img1.convertTo(img1_8UC3, CV_8UC3);
        img2.convertTo(img2_8UC3, CV_8UC3);
    }
    img1_8UC3.copyTo(left);
    img2_8UC3.copyTo(right);
    
    for(int i = 0; i < f1.size(); ++i) {
        Scalar color(rand() % RGB_8U_RANGE, rand() % RGB_8U_RANGE, rand() % RGB_8U_RANGE);
        circle(result, f1[i], CIRCLE_RADIUS, color, CIRCLE_THICKNESS, LINE_AA);
        line(result, f1[i], f2[i] + Point2(img1.cols, 0), color, LINE_THICKNESS, LINE_AA);
        circle(result, f2[i] + Point2(img1.cols, 0), CIRCLE_RADIUS, color, CIRCLE_THICKNESS, LINE_AA);
    }
    return result;
}