//
//  APAP_Stitching.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__APAP_Stitching__
#define __UglyMan_Stitiching__APAP_Stitching__

#include "Configure.h"
#include "Transform.h"

class APAP_Stitching {
public:
    static void apap_project(const vector<Point2> & _p_src,
                             const vector<Point2> & _p_dst,
                             const vector<Point2> & _src,
                             vector<Point2>       & _dst,
                             vector<Mat>          & _homographies);
private:
};

#endif /* defined(__UglyMan_Stitiching__APAP_Stitching__) */
