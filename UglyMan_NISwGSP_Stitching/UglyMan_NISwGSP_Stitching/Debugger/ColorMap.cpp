//
//  ColorMap.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "ColorMap.h"

Scalar getBlueToRedScalar(double v, double vmin, double vmax) {
    Scalar c = {1.0, 1.0, 1.0, 1.0}; // white
    double dv;
    
    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;
    
    if (v < (vmin + 0.25 * dv)) {
        c[2] = 0;
        c[1] = 4 * (v - vmin) / dv;
    } else if (v < (vmin + 0.5 * dv)) {
        c[2] = 0;
        c[0] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    } else if (v < (vmin + 0.75 * dv)) {
        c[2] = 4 * (v - vmin - 0.5 * dv) / dv;
        c[0] = 0;
    } else {
        c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        c[0] = 0;
    }
    return c;
}
