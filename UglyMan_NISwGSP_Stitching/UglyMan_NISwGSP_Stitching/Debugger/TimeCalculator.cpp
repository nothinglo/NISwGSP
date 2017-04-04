//
//  TimeCalculator.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "TimeCalculator.h"

void TimeCalculator::start() {
    begin_time = omp_get_wtime();
}
double TimeCalculator::end(const string output) const {
    double result = omp_get_wtime() - begin_time;
    if(output.empty() == false) {
        printf("[TIME] %.4fs : %s\n", result, output.c_str());
    }
    return result;
}