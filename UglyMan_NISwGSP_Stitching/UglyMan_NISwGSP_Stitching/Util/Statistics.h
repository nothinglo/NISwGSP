//
//  Statistics.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__Statistics__
#define __UglyMan_Stitiching__Statistics__

#include <vector>
#include <cmath>
#include "ErrorController.h"

class Statistics {
public:
    template <typename T>
    static void getMeanAndVariance(const vector<T> & _vec, double & _mean, double & _var);
    
    template <typename T>
    static void getMeanAndSTD(const vector<T> & _vec, double & _mean, double & _std);
    
    template <typename T>
    static void getMin(const vector<T> & _vec, double & _min);
    
    template <typename T>
    static void getMax(const vector<T> & _vec, double & _max);
    
    template <typename T>
    static void getMinAndMax(const vector<T> & _vec, double & _min, double & _max);
    
    template <typename T>
    static void getMedianWithCopyData(const vector<T> & _vec, double & _median);
    
    template <typename T>
    static void getMedianWithoutCopyData(vector<T> & _vec, double & _median);
    
    template <typename T>
    Statistics(const vector<T> & _vec);
    
    double mean, var, std, min, max;
private:
    
};

#endif /* defined(__UglyMan_Stitiching__Statistics__) */
