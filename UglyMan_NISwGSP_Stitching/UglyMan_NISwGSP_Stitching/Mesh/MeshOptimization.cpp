//
//  MeshOptimization.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "MeshOptimization.h"

MeshOptimization::MeshOptimization(const MultiImages & _multi_images) {
    multi_images = &_multi_images;
    
    alignment_weight = 0;
    local_similarity_weight = 0;
    global_similarity_weight_beta = global_similarity_weight_gamma = 0;
    global_rotation_method = GLOBAL_ROTATION_METHODS_SIZE;
    
    alignment_equation = make_pair(0, 0);
    local_similarity_equation = make_pair(0, 0);
    global_similarity_equation = make_pair(0, 0);
}

void MeshOptimization::setWeightToAlignmentTerm(const double _weight) {
    alignment_weight = _weight;
}

void MeshOptimization::setWeightToLocalSimilarityTerm(const double _weight) {
    local_similarity_weight = _weight;
}

void MeshOptimization::setWeightToGlobalSimilarityTerm(const double _weight_beta,
                                                       const double _weight_gamma,
                                                       const enum GLOBAL_ROTATION_METHODS _global_rotation_method) {
    global_similarity_weight_beta  = _weight_beta;
    global_similarity_weight_gamma = _weight_gamma;
    global_rotation_method         = _global_rotation_method;
}

const MultiImages & MeshOptimization::getMultiImages() const {
    return *multi_images;
}

double MeshOptimization::getAlignmentTermWeight() const {
    return alignment_weight;
}
double MeshOptimization::getLocalSimilarityTermWeight() const {
    return local_similarity_weight;
}
double MeshOptimization::getGlobalSimilarityTermWeightBeta() const {
    return global_similarity_weight_beta;
}
double MeshOptimization::getGlobalSimilarityTermWeightGamma() const {
    return global_similarity_weight_gamma;
}

enum GLOBAL_ROTATION_METHODS MeshOptimization::getGlobalRotationMethod() const {
    return global_rotation_method;
}

void MeshOptimization::reserveData(vector<Triplet<double> > & _triplets,
                                   vector<pair<int, double> > & _b_vector,
                                   const int _start_index) {
    int equation = _start_index;
    const bool alignment_term = alignment_weight;
    const bool local_similarity_term = local_similarity_weight;
    const bool global_similarity_term = (global_similarity_weight_beta || global_similarity_weight_gamma);
    int edge_count = (local_similarity_term || global_similarity_term) ? getEdgesCount() : 0;
    int similarity_equation_count = (edge_count) ? edge_count * DIMENSION_2D : 0;
    int edge_neighbor_vertices_count = (similarity_equation_count) ? getEdgeNeighborVerticesCount() : 0;
    
    alignment_equation.first = equation;
    alignment_equation.second = (alignment_term) ? getAlignmentTermEquationsCount() : 0;
    equation += alignment_equation.second;
    
    local_similarity_equation.first = equation;
    local_similarity_equation.second = (local_similarity_term) ? similarity_equation_count : 0;
    equation += local_similarity_equation.second;

    global_similarity_equation.first = equation;
    global_similarity_equation.second = (global_similarity_term) ? similarity_equation_count : 0;
    
    _triplets.reserve(alignment_equation.second * 8 +
                      (local_similarity_term) * (edge_neighbor_vertices_count * 8 + edge_count * 4) +
                      (global_similarity_term) * (edge_neighbor_vertices_count * 8) +
                      _start_index);
    _b_vector.reserve((global_similarity_term) * edge_neighbor_vertices_count * 4 +
                      _start_index);
}

void MeshOptimization::prepareAlignmentTerm(vector<Triplet<double> > & _triplets) const {
    if(alignment_equation.second) {
        const int equation = alignment_equation.first;
        
        const vector<vector<InterpolateVertex> > & mesh_interpolate_vertex_of_matching_pts = multi_images->getInterpolateVerticesOfMatchingPoints();
        const vector<detail::MatchesInfo> & pairwise_matches = multi_images->getPairwiseMatchesByMatchingPoints();
        const vector<pair<int, int> > & images_match_graph_pair_list = multi_images->parameter.getImagesMatchGraphPairList();
        const vector<int> & images_vertices_start_index = multi_images->getImagesVerticesStartIndex();

        int eq_count = 0;
        for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            const int pm_index = m1 * (int)multi_images->images_data.size() + m2;
            const vector<Indices> & polygons_indices_1 = multi_images->images_data[m1].mesh_2d->getPolygonsIndices();
            const vector<Indices> & polygons_indices_2 = multi_images->images_data[m2].mesh_2d->getPolygonsIndices();
            
            for(int j = 0; j < pairwise_matches[pm_index].matches.size(); ++j) {
                const DMatch & D_Match = pairwise_matches[pm_index].matches[j];
                
                for(int dim = 0; dim < DIMENSION_2D; ++dim) {
                    for(int k = 0; k < multi_images->images_data[m1].mesh_2d->getPolygonVerticesCount(); ++k) {
                        _triplets.emplace_back(equation + eq_count + dim,
                                               images_vertices_start_index[m1] + dim +
                                               DIMENSION_2D * (polygons_indices_1[mesh_interpolate_vertex_of_matching_pts[m1][D_Match.queryIdx].polygon].indices[k]),
                                                alignment_weight * mesh_interpolate_vertex_of_matching_pts[m1][D_Match.queryIdx].weights[k]);
                    }
                    for(int k = 0; k < multi_images->images_data[m2].mesh_2d->getPolygonVerticesCount(); ++k) {
                        _triplets.emplace_back(equation + eq_count + dim,
                                               images_vertices_start_index[m2] + dim +
                                               DIMENSION_2D * (polygons_indices_2[mesh_interpolate_vertex_of_matching_pts[m2][D_Match.trainIdx].polygon].indices[k]),
                                               -alignment_weight * mesh_interpolate_vertex_of_matching_pts[m2][D_Match.trainIdx].weights[k]);
                    }
                }
                eq_count += DIMENSION_2D;
            }
        }
        assert(eq_count == alignment_equation.second);
    }
}

void MeshOptimization::prepareSimilarityTerm(vector<Triplet<double> > & _triplets,
                                             vector<pair<int, double> > & _b_vector) const {
    const bool local_similarity_term = local_similarity_equation.second;
    const bool global_similarity_term = global_similarity_equation.second;
    if(local_similarity_term || global_similarity_term) {
        const vector<int> & images_vertices_start_index = multi_images->getImagesVerticesStartIndex();
        const vector<vector<double> > & images_grid_space_matching_pts_weight = multi_images->getImagesGridSpaceMatchingPointsWeight(global_similarity_weight_gamma);
        const vector<SimilarityElements> & images_similarity_elements = multi_images->getImagesSimilarityElements(global_rotation_method);
        int eq_count = 0, eq_count_rotation = 0;
        for(int i = 0; i < multi_images->images_data.size(); ++i) {
            const vector<Edge> & edges = multi_images->images_data[i].mesh_2d->getEdges();
            const vector<Point2> & vertices = multi_images->images_data[i].mesh_2d->getVertices();
            const vector<Indices> & v_neighbors = multi_images->images_data[i].mesh_2d->getVertexStructures();
            const vector<Indices> & e_neighbors = multi_images->images_data[i].mesh_2d->getEdgeStructures();
            
            const double similarity[DIMENSION_2D] = {
                images_similarity_elements[i].scale * cos(images_similarity_elements[i].theta),
                images_similarity_elements[i].scale * sin(images_similarity_elements[i].theta)
            };
            
            for(int j = 0; j < edges.size(); ++j) {
                const int & ind_e1 = edges[j].indices[0];
                const int & ind_e2 = edges[j].indices[1];
                const Point2 & src = multi_images->images_data[i].mesh_2d->getVertices()[ind_e1];
                const Point2 & dst = multi_images->images_data[i].mesh_2d->getVertices()[ind_e2];
                set<int> point_ind_set;
                for(int e = 0; e < EDGE_VERTEX_SIZE; ++e) {
                    for(int v = 0; v < v_neighbors[edges[j].indices[e]].indices.size(); ++v) {
                        int v_index = v_neighbors[edges[j].indices[e]].indices[v];
                        if(v_index != ind_e1) {
                            point_ind_set.insert(v_index);
                        }
                    }
                }
                Mat Et, E_Main(DIMENSION_2D, DIMENSION_2D, CV_64FC1), E((int)point_ind_set.size() * DIMENSION_2D, DIMENSION_2D, CV_64FC1);
                set<int>::const_iterator it = point_ind_set.begin();
                for(int p = 0; it != point_ind_set.end(); ++p, ++it) {
                    Point2 e = vertices[*it] - src;
                    E.at<double>(DIMENSION_2D * p    , 0) =  e.x;
                    E.at<double>(DIMENSION_2D * p    , 1) =  e.y;
                    E.at<double>(DIMENSION_2D * p + 1, 0) =  e.y;
                    E.at<double>(DIMENSION_2D * p + 1, 1) = -e.x;
                }
                transpose(E, Et);
                Point2 e_main = dst - src;
                E_Main.at<double>(0, 0) =  e_main.x;
                E_Main.at<double>(0, 1) =  e_main.y;
                E_Main.at<double>(1, 0) =  e_main.y;
                E_Main.at<double>(1, 1) = -e_main.x;
                
                Mat G_W = (Et * E).inv(DECOMP_SVD) * Et;
                Mat L_W = - E_Main * G_W;
                
                double _global_similarity_weight = global_similarity_weight_beta;
                if(global_similarity_weight_gamma) {
                    double sum_weight = 0;
                    for(int p = 0; p < e_neighbors[j].indices.size(); ++p) {
                        sum_weight += images_grid_space_matching_pts_weight[i][e_neighbors[j].indices[p]];
                    }
                    _global_similarity_weight = _global_similarity_weight + global_similarity_weight_gamma * (sum_weight / e_neighbors[j].indices.size());
                }
                double _local_similarity_weight = 1;
                it = point_ind_set.begin();
                for(int p = 0; it != point_ind_set.end(); ++p, ++it) {
                    for(int xy = 0; xy < DIMENSION_2D; ++xy) {
                        for(int dim = 0; dim < DIMENSION_2D; ++dim) {
                            if(local_similarity_term) {
                                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                                                       images_vertices_start_index[i]  + DIMENSION_2D * (*it) + xy,
                                                       _local_similarity_weight *
                                                       local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
                                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                                                       images_vertices_start_index[i]  + DIMENSION_2D * ind_e1 + xy,
                                                       _local_similarity_weight *
                                                      -local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
                            }
                            if(global_similarity_term) {
                                _triplets.emplace_back(global_similarity_equation.first + eq_count + dim,
                                                      images_vertices_start_index[i]    + DIMENSION_2D * (*it) + xy,
                                                       _global_similarity_weight * G_W.at<double>(dim, DIMENSION_2D * p + xy));
                                _triplets.emplace_back(global_similarity_equation.first + eq_count + dim,
                                                      images_vertices_start_index[i]    + DIMENSION_2D * ind_e1 + xy,
                                                      -_global_similarity_weight * G_W.at<double>(dim, DIMENSION_2D * p + xy));
                                _b_vector.emplace_back(global_similarity_equation.first + eq_count + dim, _global_similarity_weight * similarity[dim]);
                            }
                        }
                    }
                }
                if(local_similarity_term) {
                    _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e2    ,
                                           _local_similarity_weight *  local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e2 + 1,
                                           _local_similarity_weight *  local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e1    ,
                                           _local_similarity_weight * -local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e1 + 1,
                                           _local_similarity_weight * -local_similarity_weight);
                }
                eq_count += DIMENSION_2D;
                ++eq_count_rotation;
            }
        }
        assert(! local_similarity_term || eq_count ==  local_similarity_equation.second);
        assert(!global_similarity_term || eq_count == global_similarity_equation.second);
    }
}

int MeshOptimization::getAlignmentTermEquationsCount() const {
    int result = 0;
    const vector<pair<int, int> > & images_match_graph_pair_list = multi_images->parameter.getImagesMatchGraphPairList();
    const vector<detail::MatchesInfo> & pairwise_matches = multi_images->getPairwiseMatchesByMatchingPoints();
    for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
        const pair<int, int> & match_pair = images_match_graph_pair_list[i];
        const int & m1 = match_pair.first, & m2 = match_pair.second;
        const int pm_index = m1 * (int)multi_images->images_data.size() + m2;
        result += pairwise_matches[pm_index].matches.size();
    }
    return result * DIMENSION_2D;
}

int MeshOptimization::getVerticesCount() const {
    int result = 0;
    for(int i = 0; i < multi_images->images_data.size(); ++i) {
        result += multi_images->images_data[i].mesh_2d->getVertices().size();
    }
    return result * DIMENSION_2D;
}

int MeshOptimization::getEdgesCount() const {
    int result = 0;
    for(int i = 0; i < multi_images->images_data.size(); ++i) {
        result += multi_images->images_data[i].mesh_2d->getEdges().size();
    }
    return result;
}

int MeshOptimization::getEdgeNeighborVerticesCount() const {
    int result = 0;
    for(int i = 0; i < multi_images->images_data.size(); ++i) {
        const vector<Edge> & edges = multi_images->images_data[i].mesh_2d->getEdges();
        const vector<Indices> & v_neighbors = multi_images->images_data[i].mesh_2d->getVertexStructures();
        for(int j = 0; j < edges.size(); ++j) {
            for(int e = 0; e < EDGE_VERTEX_SIZE; ++e) {
                result += v_neighbors[edges[j].indices[e]].indices.size();
            }
        }
        result -= edges.size();
    }
    return result;
}

vector<vector<Point2> > MeshOptimization::getImageVerticesBySolving(vector<Triplet<double> > & _triplets,
                                                                    const vector<pair<int, double> > & _b_vector) const {
    const int equations = global_similarity_equation.first + global_similarity_equation.second;

    LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
    SparseMatrix<double> A(equations, getVerticesCount());
    VectorXd b = VectorXd::Zero(equations), x;
    
#ifndef NDEBUG
    TimeCalculator timer;
    timer.start();
    cout << "A = [" << equations << ", " << getVerticesCount() << "]" << endl;
#endif
    A.setFromTriplets(_triplets.begin(), _triplets.end());
    for(int i = 0; i < _b_vector.size(); ++i) {
        b[_b_vector[i].first] = _b_vector[i].second;
    }
#ifndef NDEBUG
    timer.end("Initial A matrix");
    timer.start();
#endif
    lscg.compute(A);
    x = lscg.solve(b);
#ifndef NDEBUG
    timer.end("Solve Ax = b");
    cout << "#Iterations:     " << lscg.iterations() << endl;
    cout << "Estimated error: " << lscg.error()      << endl;
#endif
    
    vector<vector<Point2> > vertices;
    vertices.resize(multi_images->images_data.size());
    for(int i = 0, x_index = 0; i < vertices.size(); ++i) {
        int count = (int)multi_images->images_data[i].mesh_2d->getVertices().size() * DIMENSION_2D;
        vertices[i].reserve(count);
        for(int j = 0; j < count; j += DIMENSION_2D) {
            vertices[i].emplace_back(x[x_index + j    ],
                                     x[x_index + j + 1]);
        }
        x_index += count;
    }
    return vertices;
}