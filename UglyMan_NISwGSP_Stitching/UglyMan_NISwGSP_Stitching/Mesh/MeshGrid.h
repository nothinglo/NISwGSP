//
//  MeshGrid.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__MeshGrid__
#define __UglyMan_Stitiching__MeshGrid__

#include "Mesh2D.h"

class MeshGrid : public Mesh2D {
public:
    MeshGrid(const int _cols, const int _rows);
    const vector<Point2> & getVertices() const;
    const vector<Edge>   & getEdges() const;
    const vector<Indices> & getPolygonsIndices() const;
    const vector<Indices> & getPolygonsNeighbors() const;
    const vector<Indices> & getPolygonsEdges() const;
    const vector<Indices> & getVertexStructures() const;
    const vector<Indices> & getEdgeStructures() const;
    const vector<Indices> & getTriangulationIndices() const;
    const int & getPolygonVerticesCount() const;
    const vector<int> & getBoundaryVertexIndices() const;
    const vector<int> & getBoundaryEdgeIndices() const;
    
    InterpolateVertex getInterpolateVertex(const Point_<float> & _p) const;
    InterpolateVertex getInterpolateVertex(const Point_<double> & _p) const;
    
    template <typename T>
    InterpolateVertex getInterpolateVertexTemplate(const Point_<T> & _p) const;
private:
    
};

#endif /* defined(__UglyMan_Stitiching__MeshGrid__) */
