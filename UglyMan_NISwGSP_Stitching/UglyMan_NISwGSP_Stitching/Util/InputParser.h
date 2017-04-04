//
//  InputParser.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__InputParser__
#define __UglyMan_Stitiching__InputParser__

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "ErrorController.h"

using namespace std;

class InputParser {
public:
    template <typename T>
    T get(const string & key, const T * ptr = NULL) const;
    
    template <typename T>
    vector<T> getVec(const string & key,
                     const bool sure_exist = true) const;
    
    InputParser(const string & file_name);
private:
    map<string, string> data;
    
};

#endif /* defined(__UglyMan_Stitiching__InputParser__) */
