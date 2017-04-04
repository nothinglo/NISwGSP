//
//  MultiImages.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "MultiImages.h"

MultiImages::MultiImages(const string & _file_name,
                         LINES_FILTER_FUNC * _width_filter,
                         LINES_FILTER_FUNC * _length_filter) : parameter(_file_name) {
    
    for(int i = 0; i < parameter.image_file_full_names.size(); ++i) {
#ifndef NDEBUG
        images_data.emplace_back(parameter.file_dir,
                                 parameter.image_file_full_names[i],
                                 _width_filter,
                                 _length_filter,
                                 &parameter.debug_dir);
#else
        images_data.emplace_back(parameter.file_dir,
                                 parameter.image_file_full_names[i],
                                 _width_filter,
                                 _length_filter);
#endif
    }
}

void MultiImages::doFeatureMatching() const {
    const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
    
    images_features.resize(images_data.size());
    images_features_mask.resize(images_data.size());
    for(int i = 0; i < images_data.size(); ++i) {
        const vector<Point2> & vertices = images_data[i].mesh_2d->getVertices();
        images_features_mask[i].resize(vertices.size(), false);
        for(int j = 0; j < vertices.size(); ++j) {
            images_features[i].keypoints.emplace_back(vertices[j], 0);
        }
    }
    pairwise_matches.resize(images_data.size() * images_data.size());
    
    apap_homographies.resize(images_data.size());
    apap_overlap_mask.resize(images_data.size());
    apap_matching_points.resize(images_data.size());
    for(int i = 0; i < images_data.size(); ++i) {
        apap_homographies[i].resize(images_data.size());
        apap_overlap_mask[i].resize(images_data.size());
        apap_matching_points[i].resize(images_data.size());
    }
    const vector<vector<vector<Point2> > > & feature_matches = getFeatureMatches();
    for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
        const pair<int, int> & match_pair = images_match_graph_pair_list[i];
        const int & m1 = match_pair.first, & m2 = match_pair.second;
        APAP_Stitching::apap_project(feature_matches[m1][m2],
                                     feature_matches[m2][m1],
                                     images_data[m1].mesh_2d->getVertices(), apap_matching_points[m1][m2], apap_homographies[m1][m2]);
        APAP_Stitching::apap_project(feature_matches[m2][m1],
                                     feature_matches[m1][m2],
                                     images_data[m2].mesh_2d->getVertices(), apap_matching_points[m2][m1], apap_homographies[m2][m1]);
        
        const int PAIR_SIZE = 2;
        const vector<Point2> * out_dst[PAIR_SIZE] = { &apap_matching_points[m1][m2], &apap_matching_points[m2][m1] };
        
        apap_overlap_mask[m1][m2].resize(apap_homographies[m1][m2].size(), false);
        apap_overlap_mask[m2][m1].resize(apap_homographies[m2][m1].size(), false);
        
        const int pm_index = m1 * (int)images_data.size() + m2;
        const int m_index[PAIR_SIZE] = {m2, m1};
        vector<DMatch> & D_matches = pairwise_matches[pm_index].matches;
        for(int j = 0; j < PAIR_SIZE; ++j) {
            for(int k = 0; k < out_dst[j]->size(); ++k) {
                if((*out_dst[j])[k].x >= 0 && (*out_dst[j])[k].y >= 0 &&
                   (*out_dst[j])[k].x <= images_data[m_index[j]].img.cols &&
                   (*out_dst[j])[k].y <= images_data[m_index[j]].img.rows) {
                    if(j) {
                        apap_overlap_mask[m2][m1][k] = true;
                        D_matches.emplace_back(images_features[m_index[j]].keypoints.size(), k, 0);
                        images_features_mask[m2][k] = true;
                    } else {
                        apap_overlap_mask[m1][m2][k] = true;
                        D_matches.emplace_back(k, images_features[m_index[j]].keypoints.size(), 0);
                        images_features_mask[m1][k] = true;
                    }
                    images_features[m_index[j]].keypoints.emplace_back((*out_dst[j])[k], 0);
                }
            }
        }
        pairwise_matches[pm_index].confidence  = 2.; /*** need > 1.f ***/
        pairwise_matches[pm_index].src_img_idx = m1;
        pairwise_matches[pm_index].dst_img_idx = m2;
        pairwise_matches[pm_index].inliers_mask.resize(D_matches.size(), 1);
        pairwise_matches[pm_index].num_inliers = (int)D_matches.size();
        pairwise_matches[pm_index].H = apap_homographies[m1][m2].front(); /*** for OpenCV findMaxSpanningTree funtion ***/
    }
}

const vector<detail::ImageFeatures> & MultiImages::getImagesFeaturesByMatchingPoints() const {
    if(images_features.empty()) {
        doFeatureMatching();
    }
    return images_features;
}

const vector<detail::MatchesInfo> & MultiImages::getPairwiseMatchesByMatchingPoints() const {
    if(pairwise_matches.empty()) {
        doFeatureMatching();
    }
    return pairwise_matches;
}

const vector<detail::CameraParams> & MultiImages::getCameraParams() const {
    if(camera_params.empty()) {
        camera_params.resize(images_data.size());
        /*** Focal Length ***/
        const vector<vector<vector<bool> > > & apap_overlap_mask = getAPAPOverlapMask();
        const vector<vector<vector<Mat> > >  & apap_homographies = getAPAPHomographies();
        
        vector<Mat> translation_matrix;
        translation_matrix.reserve(images_data.size());
        for(int i = 0; i < images_data.size(); ++i) {
            Mat T(3, 3, CV_64FC1);
            T.at<double>(0, 0) = T.at<double>(1, 1) = T.at<double>(2, 2) = 1;
            T.at<double>(0, 2) = images_data[i].img.cols * 0.5;
            T.at<double>(1, 2) = images_data[i].img.rows * 0.5;
            T.at<double>(0, 1) = T.at<double>(1, 0) = T.at<double>(2, 0) = T.at<double>(2, 1) = 0;
            translation_matrix.emplace_back(T);
        }
        vector<vector<double> > image_focal_candidates;
        image_focal_candidates.resize(images_data.size());
        for(int i = 0; i < images_data.size(); ++i) {
            for(int j = 0; j < images_data.size(); ++j) {
                for(int k = 0; k < apap_overlap_mask[i][j].size(); ++k) {
                    if(apap_overlap_mask[i][j][k]) {
                        double f0, f1;
                        bool f0_ok, f1_ok;
                        Mat H = translation_matrix[j].inv() * apap_homographies[i][j][k] * translation_matrix[i];
                        detail::focalsFromHomography(H / H.at<double>(2, 2),
                                                     f0, f1, f0_ok, f1_ok);
                        if(f0_ok && f1_ok) {
                            image_focal_candidates[i].emplace_back(f0);
                            image_focal_candidates[j].emplace_back(f1);
                        }
                    }
                }
            }
        }
        for(int i = 0; i < camera_params.size(); ++i) {
            if(image_focal_candidates[i].empty()) {
                camera_params[i].focal = images_data[i].img.cols + images_data[i].img.rows;
            } else {
                Statistics::getMedianWithoutCopyData(image_focal_candidates[i], camera_params[i].focal);
            }
        }
        /********************/
        /*** 3D Rotations ***/
        vector<vector<Mat> > relative_3D_rotations;
        relative_3D_rotations.resize(images_data.size());
        for(int i = 0; i < relative_3D_rotations.size(); ++i) {
            relative_3D_rotations[i].resize(images_data.size());
        }
        const vector<detail::ImageFeatures> & images_features    = getImagesFeaturesByMatchingPoints();
        const vector<detail::MatchesInfo>   & pairwise_matches   = getPairwiseMatchesByMatchingPoints();
        const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
        for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            const int m_index = m1 * (int)images_data.size() + m2;
            const detail::MatchesInfo & matches_info = pairwise_matches[m_index];
            const double & focal1 = camera_params[m1].focal;
            const double & focal2 = camera_params[m2].focal;
            
            MatrixXd A = MatrixXd::Zero(matches_info.num_inliers * DIMENSION_2D,
                                        HOMOGRAPHY_VARIABLES_COUNT);
            
            for(int j = 0; j < matches_info.num_inliers; ++j) {
                Point2d p1 = Point2d(images_features[m1].keypoints[matches_info.matches[j].queryIdx].pt) -
                             Point2d(translation_matrix[m1].at<double>(0, 2), translation_matrix[m1].at<double>(1, 2));
                Point2d p2 = Point2d(images_features[m2].keypoints[matches_info.matches[j].trainIdx].pt) -
                             Point2d(translation_matrix[m2].at<double>(0, 2), translation_matrix[m2].at<double>(1, 2));
                A(2*j  , 0) =  p1.x;
                A(2*j  , 1) =  p1.y;
                A(2*j  , 2) =         focal1;
                A(2*j  , 6) = -p2.x *   p1.x / focal2;
                A(2*j  , 7) = -p2.x *   p1.y / focal2;
                A(2*j  , 8) = -p2.x * focal1 / focal2;
                
                A(2*j+1, 3) =  p1.x;
                A(2*j+1, 4) =  p1.y;
                A(2*j+1, 5) =         focal1;
                A(2*j+1, 6) = -p2.y *   p1.x / focal2;
                A(2*j+1, 7) = -p2.y *   p1.y / focal2;
                A(2*j+1, 8) = -p2.y * focal1 / focal2;
            }
            JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV);
            MatrixXd V = jacobi_svd.matrixV();
            Mat R(3, 3, CV_64FC1);
            for(int j = 0; j < V.rows(); ++j) {
                R.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
            }
            SVD svd(R, SVD::FULL_UV);
            relative_3D_rotations[m1][m2] = svd.u * svd.vt;
        }
        queue<int> que;
        vector<bool> labels(images_data.size(), false);
        const int & center_index = parameter.center_image_index;
        const vector<vector<bool> > & images_match_graph = parameter.getImagesMatchGraph();
        
        que.push(center_index);
        relative_3D_rotations[center_index][center_index] = Mat::eye(3, 3, CV_64FC1);
        
        while(que.empty() == false) {
            int now = que.front();
            que.pop();
            labels[now] = true;
            for(int i = 0; i < images_data.size(); ++i) {
                if(labels[i] == false) {
                    if(images_match_graph[now][i]) {
                        relative_3D_rotations[i][i] = relative_3D_rotations[now][i] * relative_3D_rotations[now][now];
                        que.push(i);
                    }
                    if(images_match_graph[i][now]) {
                        relative_3D_rotations[i][i] = relative_3D_rotations[i][now].inv() * relative_3D_rotations[now][now];
                        que.push(i);
                    }
                }
            }
        }
        /********************/
        for(int i = 0; i < camera_params.size(); ++i) {
            camera_params[i].aspect = 1;
            camera_params[i].ppx = translation_matrix[i].at<double>(0, 2);
            camera_params[i].ppy = translation_matrix[i].at<double>(1, 2);
            camera_params[i].t = Mat::zeros(3, 1, CV_64FC1);
            camera_params[i].R = relative_3D_rotations[i][i].inv();
            camera_params[i].R.convertTo(camera_params[i].R, CV_32FC1);
        }
        
        Ptr<detail::BundleAdjusterBase> adjuster = makePtr<detail::BundleAdjusterReproj>();
        adjuster->setTermCriteria(TermCriteria(TermCriteria::EPS, CRITERIA_MAX_COUNT, CRITERIA_EPSILON));
        
        Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
        refine_mask(0, 0) = 1; /* (0, 0)->focal, (0, 2)->ppx, (1, 2)->ppy, (1, 1)->aspect */
        adjuster->setConfThresh(1.f);
        adjuster->setRefinementMask(refine_mask);
        
        if (!(*adjuster)(images_features, pairwise_matches, camera_params)) {
            printError("F(getCameraParams) camera parameters adjuster failed");
        }
        
        Mat center_rotation_inv = camera_params[parameter.center_image_index].R.inv();
        for(int i = 0; i < camera_params.size(); ++i) {
            camera_params[i].R = center_rotation_inv * camera_params[i].R;
        }
        /* wave correction */
        if(WAVE_CORRECT != WAVE_X) {
            vector<Mat> rotations;
            rotations.reserve(camera_params.size());
            for(int i = 0; i < camera_params.size(); ++i) {
                rotations.emplace_back(camera_params[i].R);
            }
            waveCorrect(rotations, ((WAVE_CORRECT == WAVE_H) ? detail::WAVE_CORRECT_HORIZ : detail::WAVE_CORRECT_VERT));
            for(int i = 0; i < camera_params.size(); ++i) {
                camera_params[i].R = rotations[i];
            }
        }
        /*******************/
    }
    return camera_params;
}

const vector<vector<bool> > & MultiImages::getImagesFeaturesMaskByMatchingPoints() const {
    if(images_features_mask.empty()) {
        doFeatureMatching();
    }
    return images_features_mask;
}

const vector<vector<vector<bool> > > & MultiImages::getAPAPOverlapMask() const {
    if(apap_overlap_mask.empty()) {
        doFeatureMatching();
    }
    return apap_overlap_mask;
}
const vector<vector<vector<Mat> > > & MultiImages::getAPAPHomographies() const {
    if(apap_homographies.empty()) {
        doFeatureMatching();
    }
    return apap_homographies;
}

const vector<vector<vector<Point2> > > & MultiImages::getAPAPMatchingPoints() const {
    if(apap_matching_points.empty()) {
        doFeatureMatching();
    }
    return apap_matching_points;
}

const vector<vector<InterpolateVertex> > & MultiImages::getInterpolateVerticesOfMatchingPoints() const {
    if(mesh_interpolate_vertex_of_matching_pts.empty()) {
        mesh_interpolate_vertex_of_matching_pts.resize(images_data.size());
        const vector<detail::ImageFeatures> & images_features = getImagesFeaturesByMatchingPoints();
        for(int i = 0; i < mesh_interpolate_vertex_of_matching_pts.size(); ++i) {
            mesh_interpolate_vertex_of_matching_pts[i].reserve(images_features[i].keypoints.size());
            for(int j = 0; j < images_features[i].keypoints.size(); ++j) {
                mesh_interpolate_vertex_of_matching_pts[i].emplace_back(images_data[i].mesh_2d->getInterpolateVertex(images_features[i].keypoints[j].pt));
            }
        }
    }
    return mesh_interpolate_vertex_of_matching_pts;
}

const vector<int> & MultiImages::getImagesVerticesStartIndex() const {
    if(images_vertices_start_index.empty()) {
        images_vertices_start_index.reserve(images_data.size());
        int index = 0;
        for(int i = 0; i < images_data.size(); ++i) {
            images_vertices_start_index.emplace_back(index);
            index += images_data[i].mesh_2d->getVertices().size() * DIMENSION_2D;
        }
    }
    return images_vertices_start_index;
}

const vector<SimilarityElements> & MultiImages::getImagesSimilarityElements(const enum GLOBAL_ROTATION_METHODS & _global_rotation_method) const {
    const vector<vector<SimilarityElements> *> & images_similarity_elements = {
        &images_similarity_elements_2D, &images_similarity_elements_3D
    };
    vector<SimilarityElements> & result = *images_similarity_elements[_global_rotation_method];
    if(result.empty()) {
        result.reserve(images_data.size());
        const vector<detail::CameraParams> & camera_params = getCameraParams();
        for(int i = 0; i < images_data.size(); ++i) {
            result.emplace_back(fabs(camera_params[parameter.center_image_index].focal / camera_params[i].focal),
                                -getEulerZXYRadians<float>(camera_params[i].R)[2]);
        }
        double rotate_theta = parameter.center_image_rotation_angle;
        for(int i = 0; i < images_data.size(); ++i) {
            double a = (result[i].theta - rotate_theta) * 180 / M_PI;
            result[i].theta = normalizeAngle(a) * M_PI / 180;
        }
        
        const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
        const vector<vector<pair<double, double> > > & images_relative_rotation_range = getImagesRelativeRotationRange();

        switch (_global_rotation_method) {
            case GLOBAL_ROTATION_2D_METHOD:
            {
                class RotationNode {
                public:
                    int index, parent;
                    RotationNode(const int _index, const int _parent) {
                        index = _index, parent = _parent;
                    }
                private:
                    
                };
                const double TOLERANT_THETA = TOLERANT_ANGLE * M_PI / 180;
                vector<pair<int, double> > theta_constraints;
                vector<bool> decided(images_data.size(), false);
                vector<RotationNode> priority_que;
                theta_constraints.emplace_back(parameter.center_image_index, result[parameter.center_image_index].theta);
                decided[parameter.center_image_index] = true;
                priority_que.emplace_back(parameter.center_image_index, -1);
                const vector<vector<bool> > & images_match_graph = parameter.getImagesMatchGraph();
                while(priority_que.empty() == false) {
                    RotationNode node = priority_que.front();
                    priority_que.erase(priority_que.begin());
                    if(!decided[node.index]) {
                        decided[node.index] = true;
                        result[node.index].theta = result[node.parent].theta + getImagesMinimumLineDistortionRotation(node.parent, node.index);
                    }
                    for(int i = 0; i < decided.size(); ++i) {
                        if(!decided[i]) {
                            const int e[EDGE_VERTEX_SIZE] = { node.index, i };
                            for(int j = 0; j < EDGE_VERTEX_SIZE; ++j) {
                                if(images_match_graph[e[j]][e[!j]]) {
                                    RotationNode new_node(i, node.index);
                                    if(isRotationInTheRange<double>(0, result[node.index].theta + images_relative_rotation_range[node.index][i].first  - TOLERANT_THETA,
                                                                       result[node.index].theta + images_relative_rotation_range[node.index][i].second + TOLERANT_THETA)) {
                                        priority_que.insert(priority_que.begin(), new_node);
                                        result[i].theta = 0;
                                        decided[i] = true;
                                        theta_constraints.emplace_back(i, 0);
                                    } else {
                                        priority_que.emplace_back(new_node);
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
                const int equations_count = (int)(images_match_graph_pair_list.size() + theta_constraints.size()) * DIMENSION_2D;
                SparseMatrix<double> A(equations_count, images_data.size() * DIMENSION_2D);
                VectorXd b = VectorXd::Zero(equations_count);
                vector<Triplet<double> > triplets;
                triplets.reserve(theta_constraints.size() * 2 + images_match_graph_pair_list.size() * 6);
                
                int equation = 0;
                for(int i = 0; i < theta_constraints.size(); ++i) {
                    triplets.emplace_back(equation    , DIMENSION_2D * theta_constraints[i].first    , STRONG_CONSTRAINT);
                    triplets.emplace_back(equation + 1, DIMENSION_2D * theta_constraints[i].first + 1, STRONG_CONSTRAINT);
                    b[equation    ] = STRONG_CONSTRAINT * cos(theta_constraints[i].second);
                    b[equation + 1] = STRONG_CONSTRAINT * sin(theta_constraints[i].second);
                    equation += DIMENSION_2D;
                }
                for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
                    const pair<int, int> & match_pair = images_match_graph_pair_list[i];
                    const int & m1 = match_pair.first, & m2 = match_pair.second;
                    const FLOAT_TYPE & MLDR_theta = getImagesMinimumLineDistortionRotation(m1, m2);
                    triplets.emplace_back(equation    , DIMENSION_2D * m1    ,  cos(MLDR_theta));
                    triplets.emplace_back(equation    , DIMENSION_2D * m1 + 1, -sin(MLDR_theta));
                    triplets.emplace_back(equation    , DIMENSION_2D * m2    ,               -1);
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m1    ,  sin(MLDR_theta));
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m1 + 1,  cos(MLDR_theta));
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m2 + 1,               -1);
                    equation += DIMENSION_2D;
                }
                assert(equation == equations_count);
                A.setFromTriplets(triplets.begin(), triplets.end());
                LeastSquaresConjugateGradient<SparseMatrix<double> > lscg(A);
                VectorXd x = lscg.solve(b);

                for(int i = 0; i < images_data.size(); ++i) {
                    result[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
                }
            }
                break;
            case GLOBAL_ROTATION_3D_METHOD:
            {
                const int equations_count = (int)images_match_graph_pair_list.size() * DIMENSION_2D + DIMENSION_2D;
                SparseMatrix<double> A(equations_count, images_data.size() * DIMENSION_2D);
                VectorXd b = VectorXd::Zero(equations_count);
                vector<Triplet<double> > triplets;
                triplets.reserve(images_match_graph_pair_list.size() * 6 + DIMENSION_2D);
                
                b[0] = STRONG_CONSTRAINT * cos(result[parameter.center_image_index].theta);
                b[1] = STRONG_CONSTRAINT * sin(result[parameter.center_image_index].theta);
                triplets.emplace_back(0, DIMENSION_2D * parameter.center_image_index    , STRONG_CONSTRAINT);
                triplets.emplace_back(1, DIMENSION_2D * parameter.center_image_index + 1, STRONG_CONSTRAINT);
                int equation = DIMENSION_2D;
                for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
                    const pair<int, int> & match_pair = images_match_graph_pair_list[i];
                    const int & m1 = match_pair.first, & m2 = match_pair.second;
                    const double guess_theta = result[m2].theta - result[m1].theta;
                    FLOAT_TYPE decision_theta, weight;
                    if(isRotationInTheRange(guess_theta,
                                            images_relative_rotation_range[m1][m2].first,
                                            images_relative_rotation_range[m1][m2].second)) {
                        decision_theta = guess_theta;
                        weight = LAMBDA_GAMMA;
                    } else {
                        decision_theta = getImagesMinimumLineDistortionRotation(m1, m2);
                        weight = 1;
                    }
                    triplets.emplace_back(equation    , DIMENSION_2D * m1    , weight *  cos(decision_theta));
                    triplets.emplace_back(equation    , DIMENSION_2D * m1 + 1, weight * -sin(decision_theta));
                    triplets.emplace_back(equation    , DIMENSION_2D * m2    ,                       -weight);
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m1    , weight *  sin(decision_theta));
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m1 + 1, weight *  cos(decision_theta));
                    triplets.emplace_back(equation + 1, DIMENSION_2D * m2 + 1,                       -weight);
                    
                    equation += DIMENSION_2D;
                }
                assert(equation == equations_count);
                A.setFromTriplets(triplets.begin(), triplets.end());
                LeastSquaresConjugateGradient<SparseMatrix<double> > lscg(A);
                VectorXd x = lscg.solve(b);
                
                for(int i = 0; i < images_data.size(); ++i) {
                    result[i].theta = atan2(x[DIMENSION_2D * i + 1], x[DIMENSION_2D * i]);
                }
            }
                break;
            default:
                printError("F(getImagesSimilarityElements) NISwGSP_ROTATION_METHOD");
                break;
        }
    }
    return result;
}

const vector<vector<pair<double, double> > > & MultiImages::getImagesRelativeRotationRange() const {
    if(images_relative_rotation_range.empty()) {
        images_relative_rotation_range.resize(images_data.size());
        for(int i = 0; i < images_relative_rotation_range.size(); ++i) {
            images_relative_rotation_range[i].resize(images_relative_rotation_range.size(), make_pair(0, 0));
        }
        const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
        const vector<vector<vector<bool> > > & apap_overlap_mask = getAPAPOverlapMask();
        const vector<vector<vector<Point2> > > & apap_matching_points = getAPAPMatchingPoints();
        for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            const vector<Edge> & m1_edges = images_data[m1].mesh_2d->getEdges();
            const vector<Edge> & m2_edges = images_data[m2].mesh_2d->getEdges();
            const vector<const vector<Edge> *> & edges = { &m1_edges, &m2_edges };
            const vector<pair<int, int> > pair_index = { make_pair(m1, m2), make_pair(m2, m1) };
            const vector<pair<const vector<Point2> *, const vector<Point2> *> > & vertices_pair = {
                make_pair(&images_data[m1].mesh_2d->getVertices(), &apap_matching_points[m1][m2]),
                make_pair(&images_data[m2].mesh_2d->getVertices(), &apap_matching_points[m2][m1])
            };
            vector<double> positive, negative;
            const vector<bool> sign_mapping = { false, true, true, false };
            for(int j = 0; j < edges.size(); ++j) {
                for(int k = 0; k < edges[j]->size(); ++k) {
                    const Edge & e = (*edges[j])[k];
                    if(apap_overlap_mask[pair_index[j].first][pair_index[j].second][e.indices[0]] &&
                       apap_overlap_mask[pair_index[j].first][pair_index[j].second][e.indices[1]]) {
                        const Point2d a = (*vertices_pair[j].first )[e.indices[0]] - (*vertices_pair[j].first )[e.indices[1]];
                        const Point2d b = (*vertices_pair[j].second)[e.indices[0]] - (*vertices_pair[j].second)[e.indices[1]];
                        const double theta = acos(a.dot(b) / (norm(a) * norm(b)));
                        const double direction = a.x * b.y - a.y * b.x;
                        int map = ((direction > 0) << 1) + j;
                        if(sign_mapping[map]) {
                            positive.emplace_back( theta);
                        } else {
                            negative.emplace_back(-theta);
                        }
                    }
                }
            }
            sort(positive.begin(), positive.end());
            sort(negative.begin(), negative.end());
            
            if(positive.empty() == false && negative.empty() == false) {
                if(positive.back() - negative.front() < M_PI) {
                    images_relative_rotation_range[m1][m2].first  = negative.front() + 2 * M_PI;
                    images_relative_rotation_range[m1][m2].second = positive.back()  + 2 * M_PI;
                    images_relative_rotation_range[m2][m1].first  = 2 * M_PI - positive.back();
                    images_relative_rotation_range[m2][m1].second = 2 * M_PI - negative.front();
                } else {
                    images_relative_rotation_range[m1][m2].first  = positive.front();
                    images_relative_rotation_range[m1][m2].second = negative.back()  + 2 * M_PI;
                    images_relative_rotation_range[m2][m1].first  =          - negative.back();
                    images_relative_rotation_range[m2][m1].second = 2 * M_PI - positive.front();

                }
            } else if(positive.empty() == false) {
                images_relative_rotation_range[m1][m2].first  =            positive.front();
                images_relative_rotation_range[m1][m2].second =            positive.back();
                images_relative_rotation_range[m2][m1].first  = 2 * M_PI - positive.back();
                images_relative_rotation_range[m2][m1].second = 2 * M_PI - positive.front();
            } else {
                images_relative_rotation_range[m1][m2].first  =  negative.front() + 2 * M_PI;
                images_relative_rotation_range[m1][m2].second =  negative.back()  + 2 * M_PI;
                images_relative_rotation_range[m2][m1].first  = -negative.back();
                images_relative_rotation_range[m2][m1].second = -negative.front();
            }
        }
    }
    return images_relative_rotation_range;
}

FLOAT_TYPE MultiImages::getImagesMinimumLineDistortionRotation(const int _from, const int _to) const {
    if(images_minimum_line_distortion_rotation.empty()) {
        images_minimum_line_distortion_rotation.resize(images_data.size());
        for(int i = 0; i < images_minimum_line_distortion_rotation.size(); ++i) {
            images_minimum_line_distortion_rotation[i].resize(images_data.size(), MAXFLOAT);
        }
    }
    if(images_minimum_line_distortion_rotation[_from][_to] == MAXFLOAT) {
        const vector<LineData> & from_lines   = images_data[_from].getLines();
        const vector<LineData> &   to_lines   = images_data[_to  ].getLines();
        const vector<Point2>   & from_project = getImagesLinesProject(_from, _to);
        const vector<Point2>   &   to_project = getImagesLinesProject(_to, _from);
        
        const vector<const vector<LineData> *> & lines    = { &from_lines,   &to_lines   };
        const vector<const vector<Point2  > *> & projects = { &from_project, &to_project };
        const vector<int> & img_indices = { _to, _from };
        const vector<int> sign_mapping = { -1, 1, 1, -1 };
        
        vector<pair<double, double> > theta_weight_pairs;
        for(int i = 0; i < lines.size(); ++i) {
            const int & rows = images_data[img_indices[i]].img.rows;
            const int & cols = images_data[img_indices[i]].img.cols;
            const vector<pair<Point2, Point2> > & boundary_edgs = {
                make_pair(Point2(0,       0), Point2(cols,    0)),
                make_pair(Point2(cols,    0), Point2(cols, rows)),
                make_pair(Point2(cols, rows), Point2(   0, rows)),
                make_pair(Point2(   0, rows), Point2(   0,    0))
            };
            for(int j = 0; j < lines[i]->size(); ++j) {
                const Point2 & p1 = (*projects[i])[EDGE_VERTEX_SIZE * j    ];
                const Point2 & p2 = (*projects[i])[EDGE_VERTEX_SIZE * j + 1];
                const bool p1_in_img = (p1.x >= 0 && p1.x <= cols && p1.y >= 0 && p1.y <= rows);
                const bool p2_in_img = (p2.x >= 0 && p2.x <= cols && p2.y >= 0 && p2.y <= rows);
                
                const bool p_in_img[EDGE_VERTEX_SIZE] = { p1_in_img, p2_in_img };
                
                Point2 p[EDGE_VERTEX_SIZE] = { p1, p2 };
            
                if(!p1_in_img || !p2_in_img) {
                    vector<double> scales;
                    for(int k = 0; k < boundary_edgs.size(); ++k) {
                        double s1;
                        if(isEdgeIntersection(p1, p2, boundary_edgs[k].first, boundary_edgs[k].second, &s1)) {
                            scales.emplace_back(s1);
                        }
                    }
                    assert(scales.size() <= EDGE_VERTEX_SIZE);
                    if(scales.size() == EDGE_VERTEX_SIZE) {
                        assert(!p1_in_img && !p2_in_img);
                        if(scales.front() > scales.back()) {
                            iter_swap(scales.begin(), scales.begin() + 1);
                        }
                        for(int k = 0; k < scales.size(); ++k) {
                            p[k] = p1 + scales[k] * (p2 - p1);
                        }
                    } else if(!scales.empty()){
                        for(int k = 0; k < EDGE_VERTEX_SIZE; ++k) {
                            if(!p_in_img[k]) {
                                p[k] = p1 + scales.front() * (p2 - p1);
                            }
                        }
                    } else {
                        continue;
                    }
                }
                const Point2d a = (*lines[i])[j].data[1] - (*lines[i])[j].data[0];
                const Point2d b = p2 - p1;
                const double theta = acos(a.dot(b) / (norm(a) * norm(b)));
                const double direction = a.x * b.y - a.y * b.x;
                const int map = ((direction > 0) << 1) + i;
                const double b_length_2 = sqrt(b.x * b.x + b.y * b.y);
                theta_weight_pairs.emplace_back(theta * sign_mapping[map],
                                                (*lines[i])[j].length * (*lines[i])[j].width * b_length_2);
            }
        }
        Point2 dir(0, 0);
        for(int i = 0; i < theta_weight_pairs.size(); ++i) {
            const double & theta = theta_weight_pairs[i].first;
            dir += (theta_weight_pairs[i].second * Point2(cos(theta), sin(theta)));
        }
        images_minimum_line_distortion_rotation[_from][_to] = acos(dir.x / (norm(dir))) * (dir.y > 0 ? 1 : -1);
        images_minimum_line_distortion_rotation[_to][_from] = -images_minimum_line_distortion_rotation[_from][_to];
    }
    return images_minimum_line_distortion_rotation[_from][_to];
}

const vector<Point2> & MultiImages::getImagesLinesProject(const int _from, const int _to) const {
    if(images_lines_projects.empty()) {
        images_lines_projects.resize(images_data.size());
        for(int i = 0; i < images_lines_projects.size(); ++i) {
            images_lines_projects[i].resize(images_data.size());
        }
    }
    if(images_lines_projects[_from][_to].empty()) {
        const vector<vector<vector<Point2> > > & feature_matches = getFeatureMatches();
        const vector<LineData> & lines = images_data[_from].getLines();
        vector<Point2> points, project_points;
        points.reserve(lines.size() * EDGE_VERTEX_SIZE);
        for(int i = 0; i < lines.size(); ++i) {
            for(int j = 0; j < EDGE_VERTEX_SIZE; ++j) {
                points.emplace_back(lines[i].data[j]);
            }
        }
        vector<Mat> not_be_used;
        APAP_Stitching::apap_project(feature_matches[_from][_to], feature_matches[_to][_from], points, images_lines_projects[_from][_to], not_be_used);
    }
    return images_lines_projects[_from][_to];
}

const vector<Mat> & MultiImages::getImages() const {
    if(images.empty()) {
        images.reserve(images_data.size());
        for(int i = 0; i < images_data.size(); ++i) {
            images.emplace_back(images_data[i].img);
        }
    }
    return images;
}

class dijkstraNode {
public:
    int from, pos;
    double dis;
    dijkstraNode(const int & _from,
                 const int & _pos,
                 const double & _dis) : from(_from), pos(_pos), dis(_dis) {
    }
    bool operator < (const dijkstraNode & rhs) const {
        return dis > rhs.dis;
    }
};
const vector<vector<double> > & MultiImages::getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma) const {
    if(_global_weight_gamma && images_polygon_space_matching_pts_weight.empty()) {
        images_polygon_space_matching_pts_weight.resize(images_data.size());
        const vector<vector<bool > > & images_features_mask = getImagesFeaturesMaskByMatchingPoints();
        const vector<vector<InterpolateVertex> > & mesh_interpolate_vertex_of_matching_pts = getInterpolateVerticesOfMatchingPoints();
        for(int i = 0; i < images_polygon_space_matching_pts_weight.size(); ++i) {
            const int polygons_count = (int)images_data[i].mesh_2d->getPolygonsIndices().size();
            vector<bool> polygons_has_matching_pts(polygons_count, false);
            for(int j = 0; j < images_features_mask[i].size(); ++j) {
                if(images_features_mask[i][j]) {
                    polygons_has_matching_pts[mesh_interpolate_vertex_of_matching_pts[i][j].polygon] = true;
                }
            }
            images_polygon_space_matching_pts_weight[i].reserve(polygons_count);
            priority_queue<dijkstraNode> que;
            
            for(int j = 0; j < polygons_has_matching_pts.size(); ++j) {
                if(polygons_has_matching_pts[j]) {
                    polygons_has_matching_pts[j] = false;
                    images_polygon_space_matching_pts_weight[i].emplace_back(0);
                    que.push(dijkstraNode(j, j, 0));
                } else {
                    images_polygon_space_matching_pts_weight[i].emplace_back(MAXFLOAT);
                }
            }
            const vector<Indices> & polygons_neighbors = images_data[i].mesh_2d->getPolygonsNeighbors();
            const vector<Point2> & polygons_center = images_data[i].mesh_2d->getPolygonsCenter();
            while(que.empty() == false) {
                const dijkstraNode now = que.top();
                const int index = now.pos;
                que.pop();
                if(polygons_has_matching_pts[index] == false) {
                    polygons_has_matching_pts[index] = true;
                    for(int j = 0; j < polygons_neighbors[index].indices.size(); ++j) {
                        const int n = polygons_neighbors[index].indices[j];
                        if(polygons_has_matching_pts[n] == false) {
                            const double dis = norm(polygons_center[n] - polygons_center[now.from]);
                            if(images_polygon_space_matching_pts_weight[i][n] > dis) {
                                images_polygon_space_matching_pts_weight[i][n] = dis;
                                que.push(dijkstraNode(now.from, n, dis));
                            }
                        }
                    }
                }
            }
            const double normalize_inv = 1. / norm(Point2i(images_data[i].img.cols, images_data[i].img.rows));
            for(int j = 0; j < images_polygon_space_matching_pts_weight[i].size(); ++j) {
                images_polygon_space_matching_pts_weight[i][j] = images_polygon_space_matching_pts_weight[i][j] * normalize_inv;
            }
        }
    }
    return images_polygon_space_matching_pts_weight;
}

void MultiImages::initialFeaturePairsSpace() const {
    feature_pairs.resize(images_data.size());
    for(int i = 0; i < images_data.size(); ++i) {
        feature_pairs[i].resize(images_data.size());
    }
}

const vector<vector<vector<pair<int, int> > > > & MultiImages::getFeaturePairs() const {
    if(feature_pairs.empty()) {
        initialFeaturePairsSpace();
        const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
        for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const vector<pair<int, int> > & initial_indices = getInitialFeaturePairs(match_pair);
            const vector<Point2> & m1_fpts = images_data[match_pair.first ].getFeaturePoints();
            const vector<Point2> & m2_fpts = images_data[match_pair.second].getFeaturePoints();
            vector<Point2> X, Y;
            X.reserve(initial_indices.size());
            Y.reserve(initial_indices.size());
            for(int j = 0; j < initial_indices.size(); ++j) {
                const pair<int, int> it = initial_indices[j];
                X.emplace_back(m1_fpts[it.first ]);
                Y.emplace_back(m2_fpts[it.second]);
            }
            vector<pair<int, int> > & result = feature_pairs[match_pair.first][match_pair.second];
            result = getFeaturePairsBySequentialRANSAC(match_pair, X, Y, initial_indices);
            assert(result.empty() == false);
#ifndef NDEBUG
            writeImageOfFeaturePairs("sRANSAC", match_pair, result);
#endif
        }
    }
    return feature_pairs;
}

const vector<vector<vector<Point2> > > & MultiImages::getFeatureMatches() const {
    if(feature_matches.empty()) {
        const vector<vector<vector<pair<int, int> > > > & feature_pairs = getFeaturePairs();
        const vector<pair<int, int> > & images_match_graph_pair_list = parameter.getImagesMatchGraphPairList();
        feature_matches.resize(images_data.size());
        for(int i = 0; i < images_data.size(); ++i) {
            feature_matches[i].resize(images_data.size());
        }
        for(int i = 0; i < images_match_graph_pair_list.size(); ++i) {
            const pair<int, int> & match_pair = images_match_graph_pair_list[i];
            const int & m1 = match_pair.first, & m2 = match_pair.second;
            feature_matches[m1][m2].reserve(feature_pairs[m1][m2].size());
            feature_matches[m2][m1].reserve(feature_pairs[m1][m2].size());
            const vector<Point2> & m1_fpts = images_data[m1].getFeaturePoints();
            const vector<Point2> & m2_fpts = images_data[m2].getFeaturePoints();
            for(int j = 0; j < feature_pairs[m1][m2].size(); ++j) {
                feature_matches[m1][m2].emplace_back(m1_fpts[feature_pairs[m1][m2][j].first ]);
                feature_matches[m2][m1].emplace_back(m2_fpts[feature_pairs[m1][m2][j].second]);
            }
        }
    }
    return feature_matches;
}

vector<pair<int, int> > MultiImages::getFeaturePairsBySequentialRANSAC(const pair<int, int> & _match_pair,
                                                                       const vector<Point2> & _X,
                                                                       const vector<Point2> & _Y,
                                                                       const vector<pair<int, int> > & _initial_indices) const {
    const int HOMOGRAPHY_MODEL_MIN_POINTS = 4;
    const int GLOBAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 - pow(GLOBAL_TRUE_PROBABILITY, HOMOGRAPHY_MODEL_MIN_POINTS));
    
    vector<char> final_mask(_initial_indices.size(), 0);
    findHomography(_X, _Y, CV_RANSAC, parameter.global_homography_max_inliers_dist, final_mask, GLOBAL_MAX_ITERATION);
    
    vector<Point2> tmp_X = _X, tmp_Y = _Y;
    
    vector<int> mask_indices(_initial_indices.size(), 0);
    for(int i = 0; i < mask_indices.size(); ++i) {
        mask_indices[i] = i;
    }
    
    while(tmp_X.size() >= HOMOGRAPHY_MODEL_MIN_POINTS &&
          parameter.local_homogrpahy_max_inliers_dist < parameter.global_homography_max_inliers_dist) {
        const int LOCAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 - pow(LOCAL_TRUE_PROBABILITY, HOMOGRAPHY_MODEL_MIN_POINTS));
        vector<Point2> next_X, next_Y;
        vector<char> mask(tmp_X.size(), 0);
        findHomography(tmp_X, tmp_Y, CV_RANSAC, parameter.local_homogrpahy_max_inliers_dist, mask, LOCAL_MAX_ITERATION);

        int inliers_count = 0;
        for(int i = 0; i < mask.size(); ++i) {
            if(mask[i]) { ++inliers_count; }
        }
        if(inliers_count < parameter.local_homography_min_features_count) {
            break;
        }
        for(int i = 0, shift = -1; i < mask.size(); ++i) {
            if(mask[i]) {
                final_mask[mask_indices[i]] = 1;
            } else {
                next_X.emplace_back(tmp_X[i]);
                next_Y.emplace_back(tmp_Y[i]);
                mask_indices[++shift] = mask_indices[i];
            }
        }
#ifndef NDEBUG
        cout << "Local true Probabiltiy = " << next_X.size() / (float)tmp_X.size() << endl;
#endif
        tmp_X = next_X;
        tmp_Y = next_Y;
    }
    vector<pair<int, int> > result;
    for(int i = 0; i < final_mask.size(); ++i) {
        if(final_mask[i]) {
            result.emplace_back(_initial_indices[i]);
        }
    }
#ifndef NDEBUG
    cout << "Global true Probabiltiy = " << result.size() / (float)_initial_indices.size() << endl;
#endif
    return result;
}

bool compareFeaturePair(const FeatureDistance & fd_1, const FeatureDistance & fd_2) {
    return
    (fd_1.feature_index[0] == fd_2.feature_index[0]) ?
    (fd_1.feature_index[1]  < fd_2.feature_index[1]) :
    (fd_1.feature_index[0]  < fd_2.feature_index[0]) ;
}

vector<pair<int, int> > MultiImages::getInitialFeaturePairs(const pair<int, int> & _match_pair) const {
    const int nearest_size = 2, pair_count = 1;
    const bool ratio_test = true, intersect = true;
    
    assert(nearest_size > 0);
    
    const int feature_size_1 = (int)images_data[_match_pair.first ].getFeaturePoints().size();
    const int feature_size_2 = (int)images_data[_match_pair.second].getFeaturePoints().size();
    const int PAIR_COUNT = 2;
    const int feature_size[PAIR_COUNT] = { feature_size_1, feature_size_2 };
    const int pair_match[PAIR_COUNT] = { _match_pair.first , _match_pair.second };
    vector<FeatureDistance> feature_pairs[PAIR_COUNT];
    
    for(int p = 0; p < pair_count; ++p) {
        const int another_feature_size = feature_size[1 - p];
        const int nearest_k = min(nearest_size, another_feature_size);
        const vector<FeatureDescriptor> & feature_descriptors_1 = images_data[pair_match[ p]].getFeatureDescriptors();
        const vector<FeatureDescriptor> & feature_descriptors_2 = images_data[pair_match[!p]].getFeatureDescriptors();
        for(int f1 = 0; f1 < feature_size[p]; ++f1) {
            set<FeatureDistance> feature_distance_set;
            feature_distance_set.insert(FeatureDistance(MAXFLOAT, p, -1, -1));
            for(int f2 = 0; f2 < feature_size[!p]; ++f2) {
                const double dist = FeatureDescriptor::getDistance(feature_descriptors_1[f1], feature_descriptors_2[f2], feature_distance_set.begin()->distance);
                if(dist < feature_distance_set.begin()->distance) {
                    if(feature_distance_set.size() == nearest_k) {
                        feature_distance_set.erase(feature_distance_set.begin());
                    }
                    feature_distance_set.insert(FeatureDistance(dist, p, f1, f2));
                }
            }
            set<FeatureDistance>::const_iterator it = feature_distance_set.begin();
            if(ratio_test) {
                const set<FeatureDistance>::const_iterator it2 = std::next(it, 1);
                if(nearest_k == nearest_size &&
                   it2->distance * FEATURE_RATIO_TEST_THRESHOLD > it->distance) {
                    continue;
                }
                it = it2;
            }
            feature_pairs[p].insert(feature_pairs[p].end(), it, feature_distance_set.end());
        }
    }
    vector<FeatureDistance> feature_pairs_result;
    if(pair_count == PAIR_COUNT) {
        sort(feature_pairs[0].begin(), feature_pairs[0].end(), compareFeaturePair);
        sort(feature_pairs[1].begin(), feature_pairs[1].end(), compareFeaturePair);
        if(intersect) {
            set_intersection(feature_pairs[0].begin(), feature_pairs[0].end(),
                             feature_pairs[1].begin(), feature_pairs[1].end(),
                             std::inserter(feature_pairs_result, feature_pairs_result.begin()),
                             compareFeaturePair);
        } else {
            set_union(feature_pairs[0].begin(), feature_pairs[0].end(),
                      feature_pairs[1].begin(), feature_pairs[1].end(),
                      std::inserter(feature_pairs_result, feature_pairs_result.begin()),
                      compareFeaturePair);
        }
    } else {
        feature_pairs_result = std::move(feature_pairs[0]);
    }
    vector<double> distances;
    distances.reserve(feature_pairs_result.size());
    for(int i = 0; i < feature_pairs_result.size(); ++i) {
        distances.emplace_back(feature_pairs_result[i].distance);
    }
    double mean, std;
    Statistics::getMeanAndSTD(distances, mean, std);
    
    const double OUTLIER_THRESHOLD = (INLIER_TOLERANT_STD_DISTANCE * std) + mean;
    vector<pair<int, int> > initial_indices;
    initial_indices.reserve(feature_pairs_result.size());
    for(int i = 0; i < feature_pairs_result.size(); ++i) {
        if(feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {
            initial_indices.emplace_back(feature_pairs_result[i].feature_index[0],
                                         feature_pairs_result[i].feature_index[1]);
        }
    }
#ifndef NDEBUG
    writeImageOfFeaturePairs("init", _match_pair, initial_indices);
#endif
    return initial_indices;
}

Mat MultiImages::textureMapping(const vector<vector<Point2> > & _vertices,
                                const Size2 & _target_size,
                                const BLENDING_METHODS & _blend_method) const {
    vector<Mat> warp_images;
    return textureMapping(_vertices, _target_size, _blend_method, warp_images);
}

Mat MultiImages::textureMapping(const vector<vector<Point2> > & _vertices,
                                const Size2 & _target_size,
                                const BLENDING_METHODS & _blend_method,
                                vector<Mat> & _warp_images) const {
    
    vector<Mat> weight_mask, new_weight_mask;
    vector<Point2> origins;
    vector<Rect_<FLOAT_TYPE> > rects = getVerticesRects<FLOAT_TYPE>(_vertices);
    
    switch (_blend_method) {
        case BLEND_AVERAGE:
            break;
        case BLEND_LINEAR:
            weight_mask = getMatsLinearBlendWeight(getImages());
            break;
        default:
            printError("F(textureMapping) BLENDING METHOD");;
    }
#ifndef NDEBUG
    for(int i = 0; i < rects.size(); ++i) {
        cout << images_data[i].file_name << " rect = " << rects[i] << endl;
    }
#endif
    _warp_images.reserve(_vertices.size());
    origins.reserve(_vertices.size());
    new_weight_mask.reserve(_vertices.size());
    
    const int NO_GRID = -1, TRIANGLE_COUNT = 3, PRECISION = 0;
    const int SCALE = pow(2, PRECISION);
    
    for(int i = 0; i < images_data.size(); ++i) {
        const vector<Point2> & src_vertices = images_data[i].mesh_2d->getVertices();
        const vector<Indices> & polygons_indices = images_data[i].mesh_2d->getPolygonsIndices();
        const Point2 origin(rects[i].x, rects[i].y);
        const Point2 shift(0.5, 0.5);
        vector<Mat> affine_transforms;
        affine_transforms.reserve(polygons_indices.size() * (images_data[i].mesh_2d->getTriangulationIndices().size()));
        Mat polygon_index_mask(rects[i].height + shift.y, rects[i].width + shift.x, CV_32SC1, Scalar::all(NO_GRID));
        int label = 0;
        for(int j = 0; j < polygons_indices.size(); ++j) {
            for(int k = 0; k < images_data[i].mesh_2d->getTriangulationIndices().size(); ++k) {
                const Indices & index = images_data[i].mesh_2d->getTriangulationIndices()[k];
                const Point2i contour[] = {
                    (_vertices[i][polygons_indices[j].indices[index.indices[0]]] - origin) * SCALE,
                    (_vertices[i][polygons_indices[j].indices[index.indices[1]]] - origin) * SCALE,
                    (_vertices[i][polygons_indices[j].indices[index.indices[2]]] - origin) * SCALE,
                };
                fillConvexPoly(polygon_index_mask, contour, TRIANGLE_COUNT, label, LINE_AA, PRECISION);
                Point2f src[] = {
                    _vertices[i][polygons_indices[j].indices[index.indices[0]]] - origin,
                    _vertices[i][polygons_indices[j].indices[index.indices[1]]] - origin,
                    _vertices[i][polygons_indices[j].indices[index.indices[2]]] - origin
                };
                Point2f dst[] = {
                    src_vertices[polygons_indices[j].indices[index.indices[0]]],
                    src_vertices[polygons_indices[j].indices[index.indices[1]]],
                    src_vertices[polygons_indices[j].indices[index.indices[2]]]
                };
                affine_transforms.emplace_back(getAffineTransform(src, dst));
                ++label;
            }
        }
        Mat image = Mat::zeros(rects[i].height + shift.y, rects[i].width + shift.x, CV_8UC4);
        Mat w_mask = (_blend_method != BLEND_AVERAGE) ? Mat::zeros(image.size(), CV_32FC1) : Mat();
        for(int y = 0; y < image.rows; ++y) {
            for(int x = 0; x < image.cols; ++x) {
                int polygon_index = polygon_index_mask.at<int>(y, x);
                if(polygon_index != NO_GRID) {
                    Point2 p_f = applyTransform2x3<FLOAT_TYPE>(x, y,
                                                               affine_transforms[polygon_index]);
                    if(p_f.x >= 0 && p_f.y >= 0 &&
                       p_f.x <= images_data[i].img.cols &&
                       p_f.y <= images_data[i].img.rows) {
                        Vec<uchar, 1> alpha = getSubpix<uchar, 1>(images_data[i].alpha_mask, p_f);
                        Vec3b c = getSubpix<uchar, 3>(images_data[i].img, p_f);
                        image.at<Vec4b>(y, x) = Vec4b(c[0], c[1], c[2], alpha[0]);
                        if(_blend_method != BLEND_AVERAGE) {
                            w_mask.at<float>(y, x) = getSubpix<float>(weight_mask[i], p_f);
                        }
                    }
                }
            }
        }
        _warp_images.emplace_back(image);
        origins.emplace_back(rects[i].x, rects[i].y);
        if(_blend_method != BLEND_AVERAGE) {
            new_weight_mask.emplace_back(w_mask);
        }
    }
    
    return Blending(_warp_images, origins, _target_size, new_weight_mask, _blend_method == BLEND_AVERAGE);
}

void MultiImages::writeResultWithMesh(const Mat & _result,
                                      const vector<vector<Point2> > & _vertices,
                                      const string & _postfix,
                                      const bool _only_border) const {
    const int line_thickness = 2;
    const Mat result(_result.size() + Size(line_thickness * 6, line_thickness * 6), CV_8UC4);
    const Point2 shift(line_thickness * 3, line_thickness * 3);
    const Rect rect(shift, _result.size());
    _result.copyTo(result(rect));
    for(int i = 0; i < images_data.size(); ++i) {
        const Scalar & color = getBlueToRedScalar((2. * i / (images_data.size() - 1)) - 1) * 255;
        const vector<Edge> & edges = images_data[i].mesh_2d->getEdges();
        vector<int> edge_indices;
        if(_only_border) {
            edge_indices = images_data[i].mesh_2d->getBoundaryEdgeIndices();
        } else {
            edge_indices.reserve(edges.size());
            for(int j = 0; j < edges.size(); ++j) {
                edge_indices.emplace_back(j);
            }
        }
        for(int j = 0; j < edge_indices.size(); ++j) {
            line(result,
                 _vertices[i][edges[edge_indices[j]].indices[0]] + shift,
                 _vertices[i][edges[edge_indices[j]].indices[1]] + shift, color, line_thickness, LINE_8);
        }
    }
    
    imwrite(parameter.debug_dir + parameter.file_name + _postfix + ".png", result(rect));
}

void MultiImages::writeImageOfFeaturePairs(const string & _name,
                                           const pair<int, int> & _match_pair,
                                           const vector<pair<int, int> > & _pairs) const {
    cout << images_data[_match_pair.first ].file_name << "-" <<
            images_data[_match_pair.second].file_name << " " << _name << " feature pairs = " << _pairs.size() << endl;
    
    const vector<Point2> & m1_fpts = images_data[_match_pair.first ].getFeaturePoints();
    const vector<Point2> & m2_fpts = images_data[_match_pair.second].getFeaturePoints();
    vector<Point2> f1, f2;
    f1.reserve(_pairs.size());
    f2.reserve(_pairs.size());
    for(int i = 0; i < _pairs.size(); ++i) {
        f1.emplace_back(m1_fpts[_pairs[i].first ]);
        f2.emplace_back(m2_fpts[_pairs[i].second]);
    }
    Mat image_of_feauture_pairs = getImageOfFeaturePairs(images_data[_match_pair.first ].img,
                                                         images_data[_match_pair.second].img,
                                                         f1, f2);
    imwrite(parameter.debug_dir +
            "feature_pairs-" + _name + "-" +
            images_data[_match_pair.first ].file_name  + "-" +
            images_data[_match_pair.second].file_name  + "-" +
            to_string(_pairs.size()) +
            images_data[_match_pair.first ].file_extension, image_of_feauture_pairs);
}