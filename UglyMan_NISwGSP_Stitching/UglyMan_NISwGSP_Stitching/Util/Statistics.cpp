//
//  Statistics.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Statistics.h"

template <typename T>
void Statistics::getMeanAndVariance(const vector<T> & _vec,
                                    double & _mean, double & _var) {
    _mean = 0, _var = 0;
    
    const int count = (int)_vec.size();
    for(int i = 0; i < count; ++i) {
        _mean += _vec[i];
    } _mean /= count;
    
    for(int i = 0; i < count; ++i) {
        _var += (_vec[i] * _vec[i]);
    }
    _var = (_var / count) - (_mean * _mean);

}

template <typename T>
void Statistics::getMeanAndSTD(const vector<T> & _vec,
                               double & _mean, double & _std) {
    getMeanAndVariance(_vec, _mean, _std);
    _std = sqrt(_std);
    
}

template <typename T>
void Statistics::getMin(const vector<T> & _vec, double & _min) {
    _min = *std::min_element(_vec.begin(), _vec.end());
}

template <typename T>
void Statistics::getMax(const vector<T> & _vec, double & _max) {
    _max = *std::max_element(_vec.begin(), _vec.end());
}

template <typename T>
void Statistics::getMinAndMax(const vector<T> & _vec, double & _min, double & _max) {
    getMin<T>(_vec, _min);
    getMax<T>(_vec, _max);
}

template <typename T>
void Statistics::getMedianWithCopyData(const vector<T> & _vec, double & _median) {
    vector<T> v = _vec;
    getMedianWithoutCopyData(v, _median);
}

template <typename T>
void Statistics::getMedianWithoutCopyData(vector<T> & _vec, double & _median) {
    size_t n = _vec.size() / 2;
    std::nth_element(_vec.begin(), _vec.begin() + n, _vec.end());
    _median = _vec[n];
    if((_vec.size() & 1) == 0) {
        std::nth_element(_vec.begin(), _vec.begin() + n - 1, _vec.end());
        _median = (_median + _vec[n-1]) * 0.5;
    }
}

template <typename T>
Statistics::Statistics(const vector<T> & _vec) {
    getMeanAndVariance<T>(_vec, mean, var);
    std = sqrt(var);
    getMinAndMax<T>(_vec, min, max);
}

template void Statistics::getMeanAndVariance<   int>(const vector<   int> & _vec, double & _mean, double & _var);
template void Statistics::getMeanAndVariance< float>(const vector< float> & _vec, double & _mean, double & _var);
template void Statistics::getMeanAndVariance<double>(const vector<double> & _vec, double & _mean, double & _var);

template void Statistics::getMeanAndSTD<   int>(const vector<   int> & _vec, double & _mean, double & _std);
template void Statistics::getMeanAndSTD< float>(const vector< float> & _vec, double & _mean, double & _std);
template void Statistics::getMeanAndSTD<double>(const vector<double> & _vec, double & _mean, double & _std);

template void Statistics::getMin<   int>(const vector<   int> & _vec, double & _min);
template void Statistics::getMin< float>(const vector< float> & _vec, double & _min);
template void Statistics::getMin<double>(const vector<double> & _vec, double & _min);

template void Statistics::getMax<   int>(const vector<   int> & _vec, double & _max);
template void Statistics::getMax< float>(const vector< float> & _vec, double & _max);
template void Statistics::getMax<double>(const vector<double> & _vec, double & _max);

template void Statistics::getMinAndMax<   int>(const vector<   int> & _vec, double & _min, double & _max);
template void Statistics::getMinAndMax< float>(const vector< float> & _vec, double & _min, double & _max);
template void Statistics::getMinAndMax<double>(const vector<double> & _vec, double & _min, double & _max);

template void Statistics::getMedianWithCopyData<   int>(const vector<   int> & _vec, double & _median);
template void Statistics::getMedianWithCopyData< float>(const vector< float> & _vec, double & _median);
template void Statistics::getMedianWithCopyData<double>(const vector<double> & _vec, double & _median);

template void Statistics::getMedianWithoutCopyData<   int>(vector<   int> & _vec, double & _median);
template void Statistics::getMedianWithoutCopyData< float>(vector< float> & _vec, double & _median);
template void Statistics::getMedianWithoutCopyData<double>(vector<double> & _vec, double & _median);

template Statistics::Statistics(const vector<   int> & _vec);
template Statistics::Statistics(const vector< float> & _vec);
template Statistics::Statistics(const vector<double> & _vec);
