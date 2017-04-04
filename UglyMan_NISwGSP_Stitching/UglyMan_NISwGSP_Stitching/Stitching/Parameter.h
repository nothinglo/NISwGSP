//
//  Parameter.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__Parameter__
#define __UglyMan_Stitiching__Parameter__

#include <queue>
#include <dirent.h>
#include <sys/stat.h>
#include "Configure.h"
#include "InputParser.h"

class Parameter {
public:
    Parameter(const string & _file_name);
    
    string file_name, file_dir;
    string stitching_parse_file_name;
    
    string result_dir, debug_dir;
    vector<string> image_file_full_names;
    /* configure */
    int grid_size;
    int down_sample_image_size;
    
    /* stitching file */
    double global_homography_max_inliers_dist;
    double  local_homogrpahy_max_inliers_dist;
       int  local_homography_min_features_count;

    int images_count;
    
    int center_image_index;
    double center_image_rotation_angle;
    
    const vector<vector<bool> >   & getImagesMatchGraph() const;
    const vector<pair<int, int> > & getImagesMatchGraphPairList() const;
private:
    mutable vector<vector<bool> >   images_match_graph_manually;
    mutable vector<vector<bool> >   images_match_graph_automatically; /* TODO */
    mutable vector<pair<int, int> > images_match_graph_pair_list;
};

#endif /* defined(__UglyMan_Stitiching__Parameter__) */
