//
//  APAP_Stitching.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "APAP_Stitching.h"

void APAP_Stitching::apap_project(const vector<Point2> & _p_src,
                                  const vector<Point2> & _p_dst,
                                  const vector<Point2> & _src,
                                  vector<Point2>       & _dst,
                                  vector<Mat>          & _homographies) {
    vector<Point2> nf1, nf2, cf1, cf2;
    Mat N1, N2, C1, C2;
    N1 = getNormalize2DPts(_p_src, nf1);
    N2 = getNormalize2DPts(_p_dst, nf2);
    C1 = getConditionerFromPts(nf1);
    C2 = getConditionerFromPts(nf2);
    cf1.reserve(nf1.size());
    cf2.reserve(nf2.size());
    for(int i = 0; i < nf1.size(); ++i) {
        cf1.emplace_back(nf1[i].x * C1.at<double>(0, 0) + C1.at<double>(0, 2),
                         nf1[i].y * C1.at<double>(1, 1) + C1.at<double>(1, 2));

        cf2.emplace_back(nf2[i].x * C2.at<double>(0, 0) + C2.at<double>(0, 2),
                         nf2[i].y * C2.at<double>(1, 1) + C2.at<double>(1, 2));
    }
    double sigma_inv_2 = 1. / (APAP_SIGMA * APAP_SIGMA), gamma = APAP_GAMMA;
    MatrixXd A = MatrixXd::Zero(cf1.size() * DIMENSION_2D,
                                HOMOGRAPHY_VARIABLES_COUNT);
    
#ifndef NDEBUG
    if(_dst.empty() == false) {
        _dst.clear();
        printError("F(apap_project) dst is not empty");
    }
    if(_homographies.empty() == false) {
        _homographies.clear();
        printError("F(apap_project) homographies is not empty");
    }
#endif
    _dst.reserve(_src.size());
    _homographies.reserve(_src.size());
    for(int i = 0; i < _src.size(); ++i) {
        for(int j = 0; j < _p_src.size(); ++j) {
            Point2 d = _src[i] - _p_src[j];
            double www = MAX(gamma, exp(-sqrt(d.x * d.x + d.y * d.y) * sigma_inv_2));
            A(2*j  , 0) = www * cf1[j].x;
            A(2*j  , 1) = www * cf1[j].y;
            A(2*j  , 2) = www * 1;
            A(2*j  , 6) = www * -cf2[j].x * cf1[j].x;
            A(2*j  , 7) = www * -cf2[j].x * cf1[j].y;
            A(2*j  , 8) = www * -cf2[j].x;
            
            A(2*j+1, 3) = www * cf1[j].x;
            A(2*j+1, 4) = www * cf1[j].y;
            A(2*j+1, 5) = www * 1;
            A(2*j+1, 6) = www * -cf2[j].y * cf1[j].x;
            A(2*j+1, 7) = www * -cf2[j].y * cf1[j].y;
            A(2*j+1, 8) = www * -cf2[j].y;
        }
        JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV);
        MatrixXd V = jacobi_svd.matrixV();
        Mat H(3, 3, CV_64FC1);
        for(int j = 0; j < V.rows(); ++j) {
            H.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
        }
        H = C2.inv() * H * C1;
        H = N2.inv() * H * N1;

        _dst.emplace_back(applyTransform3x3(_src[i].x, _src[i].y, H));
        _homographies.emplace_back(H);
    }
}