//
//  MeshOptimization.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__MeshOptimization__
#define __UglyMan_Stitiching__MeshOptimization__

#include "MultiImages.h"
#include "Blending.h"

class MeshOptimization {
public:
    MeshOptimization(const MultiImages & _multi_images);

    virtual Mat solve(const BLENDING_METHODS & _blend_method) = 0;
protected:
    void setWeightToAlignmentTerm(const double _weight);
    void setWeightToLocalSimilarityTerm(const double _weight);
    void setWeightToGlobalSimilarityTerm(const double _weight_beta,
                                         const double _weight_gamma,
                                         const enum GLOBAL_ROTATION_METHODS _global_rotation_method);
    
    const MultiImages & getMultiImages() const;
    
    double getAlignmentTermWeight() const;
    double getLocalSimilarityTermWeight() const;
    double getGlobalSimilarityTermWeightBeta() const;
    double getGlobalSimilarityTermWeightGamma() const;
    enum GLOBAL_ROTATION_METHODS getGlobalRotationMethod() const;
    
    void reserveData(vector<Triplet<double> > & _triplets,
                     vector<pair<int, double> > & _b_vector,
                     const int _start_index);
    
    void prepareAlignmentTerm(vector<Triplet<double> > & _triplets) const;
    void prepareSimilarityTerm(vector<Triplet<double> > & _triplets,
                               vector<pair<int, double> > & _b_vector) const;
    
    vector<vector<Point2> > getImageVerticesBySolving(vector<Triplet<double> > & _triplets,
                                                      const vector<pair<int, double> > & _b_vector) const;
    
private:
    
    int getAlignmentTermEquationsCount() const;

    int getVerticesCount() const;
    int getEdgesCount() const;
    int getEdgeNeighborVerticesCount() const;
    
    const MultiImages * multi_images;
    
    double alignment_weight;
    double local_similarity_weight;
    double global_similarity_weight_beta, global_similarity_weight_gamma;
    
    pair<int, int> alignment_equation; /* begin, count */
    pair<int, int> local_similarity_equation;
    pair<int, int> global_similarity_equation;
    enum GLOBAL_ROTATION_METHODS global_rotation_method;
};

#endif /* defined(__UglyMan_Stitiching__MeshOptimization__) */
