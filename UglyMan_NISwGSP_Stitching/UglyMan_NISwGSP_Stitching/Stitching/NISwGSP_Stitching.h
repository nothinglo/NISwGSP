//
//  NISwGSP_Stitching.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__NISwGSP_Stitching__
#define __UglyMan_Stitiching__NISwGSP_Stitching__

#include "MeshOptimization.h"

class NISwGSP_Stitching : public MeshOptimization {
public:
    NISwGSP_Stitching(const MultiImages & _multi_images);
    
    void setWeightToAlignmentTerm(const double _weight);
    void setWeightToLocalSimilarityTerm(const double _weight);
    void setWeightToGlobalSimilarityTerm(const double _weight_beta,
                                         const double _weight_gamma,
                                         const enum GLOBAL_ROTATION_METHODS _global_rotation_method);
    
    void writeImage(const Mat & _image, const string _post_name) const;
    
    Mat solve(const BLENDING_METHODS & _blend_method);
private:
};

#endif /* defined(__UglyMan_Stitiching__NISwGSP_Stitching__) */
