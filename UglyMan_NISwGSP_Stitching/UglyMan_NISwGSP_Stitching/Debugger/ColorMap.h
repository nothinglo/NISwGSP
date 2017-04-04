//
//  ColorMap.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__ColorMap__
#define __UglyMan_Stitiching__ColorMap__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

Scalar getBlueToRedScalar(double v, double vmin = -1, double vmax = 1);

#endif /* defined(__UglyMan_Stitiching_ColorMap__) */
