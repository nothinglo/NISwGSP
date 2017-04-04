//
//  Mesh2D.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Mesh2D.h"

Mesh2D::Mesh2D(const int _cols, const int _rows) {
    nw = _cols / GRID_SIZE + (_cols % GRID_SIZE != 0);
    nh = _rows / GRID_SIZE + (_rows % GRID_SIZE != 0);
    lw = _cols / (double)nw;
    lh = _rows / (double)nh;
}
Mesh2D::~Mesh2D() {
    
}

const vector<Point2> & Mesh2D::getPolygonsCenter() const {
    if(polygons_center.empty()) {
        const vector<Point2> & vertices = getVertices();
        const vector<Indices> & polygons_indices = getPolygonsIndices();
        polygons_center.reserve(polygons_indices.size());
        for(int i = 0; i < polygons_indices.size(); ++i) {
            Point2 center(0, 0);
            for(int j = 0; j < polygons_indices[i].indices.size(); ++j) {
                center += vertices[polygons_indices[i].indices[j]];
            }
            polygons_center.emplace_back(center / (FLOAT_TYPE)polygons_indices[i].indices.size());
        }
    }
    return polygons_center;
}

template <typename T>
int Mesh2D::getGridIndexOfPoint(const Point_<T> & _p) const {
    Point2i grid_p(_p.x / lw, _p.y / lh);
    grid_p.x = grid_p.x - (grid_p.x == nw);
    grid_p.y = grid_p.y - (grid_p.y == nh);
    return grid_p.x + grid_p.y * nw;
}

template int Mesh2D::getGridIndexOfPoint< float>(const Point_< float> & _p) const;
template int Mesh2D::getGridIndexOfPoint<double>(const Point_<double> & _p) const;
