/********************************************************************
* LGT tracker - The official C++ implementation of the LGT tracker
* Copyright (C) 2013  Luka Cehovin
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
********************************************************************/
/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*- */

#include "lgt.h"
#include "common/utils/utils.h"
#include "common/canvas.h"
namespace legit {

    namespace tracker {

        typedef struct CostPair_ {
            float cost;
            int index;
        } CostPair;

        bool __random_init = false;


        int compare_cost_pair (const void* i, const void* j) {
            float c = ( ((CostPair*)j)->cost - ((CostPair*)i)->cost );
            return (c < 0) ? -1 : (c > 0) ? 1 : 0;
        }

        LGTTracker::LGTTracker(string inst) :
            patches(6, 30),
            modalities(),
            verbosity(0),
            probability_size(150),
            global_optimization(50, 300, 10, 10, 10, 0.001),
            local_optimization( 40, 40, 0, 5, 10, 0.001),
            motion(4, 2, 0) {
            patch_scale =  1.0;

            patches_max =  6;
            patches_min =  36 ;
            patches_persistence =  0.8;

            reweight_persistence =  0.5;
            reweight_similarity =  3;
            reweight_distance =  3;


            weight_remove_threshold =  0.1;
            merge_distance =  0.5;



            lambda_geometry =  0.03;
            lambda_visual =  1;

            optimization_global_M =  20;
            optimization_global_R =  0.03;
            optimization_global_S =  0.0001;


            optimization_local_M =  5;

            // TODO: probably needs rethinking
            median_size_min =  0;
            median_size_max =  INT_MAX;
            median_persistence =  0.8;

            size_constraints.min_size.width =  -1;
            size_constraints.min_size.height =  -1;
            size_constraints.max_size.width =  -1;
            size_constraints.max_size.height =  -1;

            sampling_threshold =  0.2;
            addition_distance =  3.0;

            string patch_type_string = "histogram";

            if (!__random_init) {
                RANDOM_SEED(time(NULL));
                __random_init = true;
            }



        }

        LGTTracker::~LGTTracker() {


        }

        void LGTTracker::initialize(Image& image, cv::Rect region) {

            int proposed_patch_size = 6; // ceil(sqrt((double)(region.width * region.height))*0.15) ;



            patches.set_patch_size(proposed_patch_size);
            // FIXME int count = (int) round( (region.width * region.height) / pow((double)3 * patches.get_radius(), 2));
            int count = (int) round( (float)(region.width * region.height) / std::pow( (float)(4 * 3),  2));
            count = MIN(MAX(patches_min, count), patches_max);

            /* FIXME
            patches.set_patch_size((int)ceil( (patch_scale * (image.width() * 6) / 320))); // Woodoo?
            int count = (int) round( (region.width * region.height) / pow((double)3 * patches.get_radius(), 2));// Woodoo2?
            */



            int sx, sy;

            if (region.width < region.height) {
                sx = floor(sqrt(count)) + 1;
                sy = ceil(sqrt(count)) + 1;
            } else {
                sy = floor(sqrt(count)) + 1;
                sx = ceil(sqrt(count)) + 1;
            }

            /* FIXME {
                sx = MAX(3, round(sqrt((float)count * ((float) region.width / (float) region.height))));
                sy = MAX(3, round((float)count / (float) sx));
            } else {
                sy = MAX(3, round(sqrt((float)count * ((float) region.height / (float) region.width))));
                sx = MAX(3, round((float)count / (float) sy));
            }*/

            count = sx * sy;



            int dx = (region.width - 2 * patches.get_radius()) / (sx - 1);
            int dy = (region.height - 2 * patches.get_radius()) / (sy - 1);

            Mat_<float> positions = Mat_<float>(count, 3);
            int k = 0;



            for (int i = 0; i < sx; i++) {
                for (int j = 0; j < sy; j++) {
                    positions.row(k++) << region.x + dx* i + patches.get_radius() + (i % 2), region.y + dy* j + patches.get_radius() + (j % 2), 0.5f;
                }
            }

            initialize(image, positions);

        }

        void LGTTracker::initialize(Image& image, Mat points) {

            //patches.flush();
            int psize = patches.size();
            for(int i = 0;i < psize;i++ ){
              patches.remove(0);
            }


            for (int i = 0; i < points.rows; i++) {
                patches.add(image, patch_type, Point2f(points.at<float>(i, 0), points.at<float>(i, 1)), points.at<float>(i, 2));
            }

            patches_capacity = patches.size();

            Point2f* positions = new Point2f[patches.size()];

            for (int i = 0; i < patches.size(); i++) {
                positions[i] = patches.get_position(i);
            }

            Matrix D = distances(positions, patches.size());
            vector<double> medians;

            /* FIXME int k_m = MAX( ceil(D.cols * 0.1), 3 ) ;
            float md = 0.0 ;
            for (int p = 0; p < patches.size(); p++) {
                      Matrix r = D.row(p);
                  cv::sort(r, r, CV_SORT_ASCENDING) ;

                  float dist_loc = 0.0 ;
                  for (int i_k = 1; i_k < k_m+1 ; ++i_k) {
                      dist_loc += r.at<double>(0,i_k) ;
                  }
                  dist_loc /= (float)k_m ;

                      medians.push_back(dist_loc);
              }
            median_threshold = median(medians) * 1.2 ;*/

            median_threshold =  20000;

            /* FIXME
              for (int p = 0; p < patches.size(); p++) {
                  Matrix m = D.row(p);
                  median_threshold = MAX(median_threshold, median(m));
              }*/



            modalities.flush();


            cv::Rect region = patches.region();
            Point2f mean = patches.mean_position();

            float half, third ;
            float spectral_density, meas_noise ;
            float diagonal, delta_t2, delta_t3 ;
            float delta_t = 1.0 ; // for framerate 5 fps  (for 3fps multiply by 1.6, for example)
            half = 0.5 ;
            third = 1.0 / 3.0 ;
            delta_t2 = delta_t* delta_t ;
            delta_t3 = delta_t* delta_t* delta_t ;

            diagonal = sqrt((float)region.width * region.width + (float)region.height * region.height) ;
            // set mesh deformation parameter
            //lambda_geometry = 0.002; // 1.0/(diagonal*0.08); //0.002 ; //0.0015
            // initialize cross-entropy optimization
            //optimization_global_M = 20; //  4.0* diagonal*0.15;  //20  ;
            //optimization_local_M = 5; // diagonal*0.05 ; //5
            // initialize motion model
            motion.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, delta_t, 0,
                                       0, 1, 0, delta_t,
                                       0, 0, 1, 0,
                                       0, 0, 0, 1);
            motion.measurementMatrix = (Mat_<float>(2, 4) << 1, 0, 0, 0,
                                        0, 1, 0, 0);
            // intitialize process noise
            meas_noise = diagonal * 0.1 * 10;
            meas_noise = meas_noise * meas_noise ; //10.0 ;   0.01
            spectral_density = diagonal * 0.2 * 10;
            spectral_density = spectral_density * spectral_density ; // 0.2  5.0*5.0; //
            motion.processNoiseCov = (Mat_<float>(4, 4) << third * delta_t3, 0, half * delta_t2, 0,
                                      0, third * delta_t3, 0,  half * delta_t2,
                                      half * delta_t2, 0, delta_t, 0,
                                      0, half * delta_t2, 0, delta_t) ;

            motion.processNoiseCov = motion.processNoiseCov * spectral_density ;
            // initialize measurement noise
            setIdentity(motion.measurementNoiseCov, Scalar(meas_noise));
            // initialize the posterior state
            motion.statePost = (Mat_<float>(4, 1) << mean.x, mean.y, 0, 0) ;
            motion.statePre = (Mat_<float>(4, 1) << mean.x, mean.y, 0, 0) ;
            // initialize posterior covariances
            setIdentity(motion.errorCovPost, Scalar(meas_noise * 4.0));
            setIdentity(motion.errorCovPre, Scalar(meas_noise * 4.0));

            track(image, false, false);

            delete [] positions;
        }

        void LGTTracker::update(Image& image) {

            track(image, true, true);

        }

        int LGTTracker::patch_num(){
          return patches.size();
        }

        void LGTTracker::track(Image& image, bool announce, bool push) {


            if (push) { patches.push(); } // allocate new state for patches

            Mat kalman_prediction = motion.predict();

            Point2f move;

            Point2f center = patches.mean_position();
            move.x = kalman_prediction.at<float>(0, 0) - center.x ;
            move.y = kalman_prediction.at<float>(1, 0) - center.y ;

            /*move.x = kalman_prediction.at<float>(2, 0)*3 ;
            move.y = kalman_prediction.at<float>(3, 0)*3 ;*/

            patches.move(move);


            stage_optimization(image, announce, push);

            /********************************************************************************
            *
            ****                   UPDATE WEIGHTS                                        ****
            *
            *********************************************************************************/

            stage_update_weights(image, announce, push);



            // recalculate center, update Kalman
            center = patches.mean_position();
            motion.correct((Mat_<float>(2, 1) << center.x, center.y));



            /********************************************************************************
            *
            ****                   UPDATE MODALITIES                                     ****
            *
            *********************************************************************************/

            stage_update_modalities(image, announce, push);

            /********************************************************************************
            *
            ****                   ADD PATCHES                                           ****
            *
            *********************************************************************************/

            stage_add_patches(image, announce, push);





        }

        void LGTTracker::stage_optimization(Image& image, bool announce, bool push) {

            /********************************************************************************
            ****                  GLOBAL OPTIMIZATION                                    ****
            *********************************************************************************/



            cv::Point2f center = patches.mean_position();

            OptimizationStatus status(patches);

            if (optimization_global_R < 0.00001 && optimization_global_S < 0.00001) {
                Matrix globalC = Mat::diag( (Mat_<double>(2, 1) << optimization_global_M,
                                             optimization_global_M));
                Matrix globalM = (Matrix(1, 2) << 0, 0);

                cross_entropy_global_move(image,
                                          patches, globalM, globalC, global_optimization, status);

            } else {
                Matrix globalC = Mat::diag( (Mat_<double>(5, 1) << optimization_global_M,
                                             optimization_global_M, optimization_global_R, optimization_global_S,
                                             optimization_global_S));
                Matrix globalM = (Matrix(1, 5) << 0, 0, 0, 1, 1);

                cross_entropy_global_affine(image,
                                            patches, globalM, globalC, global_optimization, size_constraints, status);

            }


            for (int i = 0; i < status.size(); i++) {
//        if (status[i].flags & OPTIMIZATION_CONVERGED) {
//printpoint(status[i].position);
                PatchStatus ps = status.get(i);
                patches.set_position(i, ps.position);
                float value = exp(-patches.response(image, i, ps.position)); //
                //  if (value > 0.8) status.set_flags(i, OPTIMIZATION_FIXED); // Zakaj mora biti vecji ravno od 0.8

//        }
            }

            /********************************************************************************
            ****                   LOCAL OPTIMIZATION                                    ****
            *********************************************************************************/

            if (patches.size() > 4) {

                DelaunayConstraints* cn = new DelaunayConstraints(patches);


                cross_entropy_local_refine(image, patches, *cn, optimization_local_M,
                                           lambda_geometry, lambda_visual, local_optimization, status);

                delete cn;

                for (int i = 0; i < status.size(); i++) {
                    PatchStatus ps = status.get(i);
                    // if (status[i].flags & OPTIMIZATION_CONVERGED) {
                    //if (announce) notify_observers(OBSERVER_CHANNEL_OPTIMIZATION, & status[i]);
                    patches.set_position(i, ps.position);
                    // }
                }

            }

        }

        void LGTTracker::stage_update_weights(Image& image, bool announce, bool push) {

            Point2f* positions = new Point2f[patches.size()];
            vector<float> similarity_score;
            vector<float> proximity_score;

            for (int i = 0; i < patches.size(); i++) {
                float similarity = exp(- patches.response(image, i, patches.get_position(i)) * reweight_similarity);
                similarity_score.push_back(similarity);
                positions[i] = patches.get_position(i);
            }

            Matrix D = distances(positions, patches.size());

            vector<double> medians;

            for (int p = 0; p < patches.size(); p++) {
                Matrix r = D.row(p);

                float m = median(r);
                medians.push_back(m);
            }

            for (int p = 0; p < patches.size(); p++) {
                proximity_score.push_back(1 / (1 + exp((medians[p] - median_threshold) * reweight_distance)));
            }

            PatchReweight reweight;

            for (int i = 0; i < patches.size(); i++) {
                reweight.id = patches.get_id(i);
                reweight.weights.clear();
                patches.set_weight(i, reweight_persistence * patches.get_weight(i) + (1 - reweight_persistence) * similarity_score[i] * proximity_score[i]);
                reweight.weights.push_back(similarity_score[i]);
                reweight.weights.push_back(proximity_score[i]);

            }

            // Merging or inhibition
            float merge_threshold = merge_distance * patches.get_radius();

            vector<int> selection;

            while (true) {
                for (int i = 0; i < patches.size(); i++)
                { positions[i] = patches.get_position(i); }


                D = distances(positions, patches.size());

                int p;

                for (p = 0; p < patches.size(); p++) {
                    selection.clear();

                    for (int m = p; m < patches.size(); m++) {
                        if (D(p, m) < merge_threshold) {
                            selection.push_back(m);
                        }
                    }

                    if (selection.size() > 1) {

                        patches.merge(image, selection, patch_type);
                        break;
                    }

                }

                if (p == patches.size())
                { break; }
            }

            delete [] positions;

            // remove patches
            WeightLowerFilter remove_filter(weight_remove_threshold);
            int removed = patches.remove(remove_filter);

        }

        void LGTTracker::stage_update_modalities(Image& image, bool announce, bool push) {


            modalities.update(image, &patches, patches.region());

        }

        void LGTTracker::stage_add_patches(Image& image, bool announce, bool push) {

            cv::Point2f center = patches.mean_position();

            cv::Rect region = intersection(cv::Rect(0, 0, image.width(), image.height()),  cv::Rect((int)center.x - probability_size / 2,
                                           (int)center.y - probability_size / 2, probability_size, probability_size));

            Image crop(image, region);

            int patches_new = MAX( MIN((int)round(patches_capacity) - (float)patches.size() + 1, patches_max - patches.size()),  patches_min - patches.size());



            Mat mask;
            patch_create(mask, (float)patches.get_patch_size() * addition_distance, (float)patches.get_patch_size() * addition_distance, PATCH_CONE, FLAG_INVERT);



            if (patches_new > 0) {

                Mat map;
                modalities.probability(crop, map);

                if (!map.empty()) {

                    double max;
                    minMaxLoc(map, NULL, &max, NULL, NULL, Mat());

                    supress_noise(map, max * sampling_threshold, 5, 1);

                    // now we mask out the positions of existing patches in the probability
                    for (int i = 0; i < patches.size(); i++) {
                        cv::Point p = patches.get_relative_position(i, region.tl());
                        patch_operation(map, mask, p, OPERATION_MULTIPLY);
                    }

                    // add new patches if possible
                    for (int i = 0; i < patches_new; i++) {

                        double total = sum(map)[0];

                        if (total < 1e-16) //TODO: hardcoded
                        { break; }

                        map /= total; // normalize masked probability

                        cv::Point p;
                        float value;

                        sample_map(map, &p, 1, &value);

                        if (p.x == -1)
                        { break; }

                        if (map.at<float>(p.y, p.x) < 0.00001)
                        { break; }



                        // mask again
                        patch_operation(map, mask, p, OPERATION_MULTIPLY);

                        patches.add(image, patch_type, p + region.tl(), 0.5); //TODO: hardcoded

                    }

                }

            }

            patches_capacity = (patches_persistence) * patches_capacity + (1 - patches_persistence) * patches.size();

        }

        cv::Rect LGTTracker::region() {

            //return bounds;
            return patches.region();

        }

        Point2f LGTTracker::position() {

            //return Point2f(bounds.x + bounds.width / 2, bounds.y + bounds.height);
            return patches.mean_position();
        }

#define VISUALIZE_MOTION_HISTORY 300

        void LGTTracker::visualize(Canvas& canvas) {

            if (verbosity < 1)
            { return; }

            int psize2 = patches.get_patch_size() / 2;
            float max_weight = 0;
            Point2f buffer[VISUALIZE_MOTION_HISTORY];

            max_weight = 1;

            for (int i = 0; i < patches.size(); i++) {
                Point2f p = patches.get_position(i);

                int tint = MIN((int) ((patches.get_weight(i) * 255) / max_weight), 255);

                canvas.rectangle(cv::Point(p.x - psize2, p.y - psize2), cv::Point(p.x + psize2, p.y + psize2), Scalar(tint, 0, tint, 10), 1);

                if (verbosity > 2) {

                    int history = patches.get_motion_history(i, buffer, VISUALIZE_MOTION_HISTORY);

                    for (int j = 0; j < history - 1; j++) {
                        canvas.line(buffer[j], buffer[j + 1], Scalar(0, 255, 0), 1);
                    }
                }

            }

            if (verbosity > 1) {

                Mat statePost = motion.statePost ;
                Mat errorCovPost = motion.errorCovPost ;
                Point2f mean, pred ;
                Matrix2f cov ;
                mean.x = statePost.at<float>(0, 0) ;
                mean.y = statePost.at<float>(1, 0) ;
                cov.m00 = errorCovPost.at<float>(0, 0) ;
                cov.m11 = errorCovPost.at<float>(1, 1) ;
                cov.m01 = errorCovPost.at<float>(0, 1) ;
                cov.m10 = errorCovPost.at<float>(1, 0) ;
                canvas.ellipse(mean, cov, Scalar(0, 0, 255));

                canvas.rectangle(region(), Scalar(0, 255, 0), 2);

                float ppx = statePost.at<float>(2, 0) ;
                float ppy = statePost.at<float>(3, 0) ;

                pred.x = mean.x + statePost.at<float>(2, 0) * motion.transitionMatrix.at<float>(0, 2) ;
                pred.y = mean.y + statePost.at<float>(3, 0) * motion.transitionMatrix.at<float>(1, 3) ;
                canvas.line(mean, pred, Scalar(100, 255, 120), 2);
            }
        }

        vector<cv::Point> LGTTracker::get_patch_positions() {

            vector<cv::Point> positions(patches.size());

            for (int i = 0; i < patches.size(); i++) {
                Point2f p = patches.get_position(i);

                positions.push_back(p);
            }

            return positions;
        }



        bool LGTTracker::is_tracking() {

            return patches.size() > 0;

        }




        string LGTTracker::get_name() {
            return "LG tracker";
        }

        void supress_noise(Mat& mat, float threshold, int window, float percent, IntegralImage* integral) {

            int window_threshold = ((float)  window * window) * percent;
            IntegralImage* temp = NULL;

            int window_offset = ceil( ((float) window) / 2);

            if (integral) {
                temp = integral;
                temp->update(mat, threshold);
            } else {
                temp = new IntegralImage(mat, threshold);
            }

            for (int i = 0; i < mat.rows; i++) {
                float* data = (float*) mat.ptr(i);

                if (i - window_offset <= 0 || i + window_offset >= mat.cols - 1) {
                    memset(data, 0, sizeof(float) * mat.cols);
                } else {

                    for (int j = 0; j < window_offset; j++) {
                        data[j] = 0;
                        data[mat.cols - 1 - j] = 0;
                    }

                    for (int j = 0; j < mat.cols - window; j++) {
                        if (temp->sum(MAX(0, j), MAX(0, i), MIN(mat.cols - 1, j + window),
                                      MIN(mat.rows - 1, i + window)) < window_threshold)
                        { data[j + window_offset] = 0; }
                    }

                }
            }

            if (!integral) {
                delete temp;
            }

        }

    }

}
