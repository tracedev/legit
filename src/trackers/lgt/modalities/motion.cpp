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

#include "motion.h"
#include <opencv2/video/tracking.hpp>

namespace legit {

    namespace tracker {

        /********************************************************************************
        *
        ****                  MOTION USING OPTICAL FLOW                              ****
        *
        *********************************************************************************/

#define OPTICALFLOW_MAX_ITERATIONS 20
#define OPTICALFLOW_MAX_RESIDUE 1e5
#define MOTION_LK_SIZE 55



        ModalityMotionLK::ModalityMotionLK(Config& config, string configbase) : Modality(config, configbase), step(2), history(step, step), motion(step, step) {

            block_size = 7;
            aperture_size = 7;

// TODO: hardcoded
            damping = 1;
            persistence = 0.7;

            levels = 2;
            window_size = 8;

            float sigma = 0.3 * (MOTION_LK_SIZE / 2 - 1) + 0.8;

//  gaussian = createGaussianFilter(CV_32F, Size(MOTION_LK_SIZE, MOTION_LK_SIZE), sigma, sigma);
//  GaussianBlur(CV_32F,CV_32F, Size(MOTION_LK_SIZE, MOTION_LK_SIZE), sigma, 0)




            flush();


        }

        ModalityMotionLK::~ModalityMotionLK() {

        }

        void ModalityMotionLK::flush() {
            history.flush();
            motion.flush();
        }

        void ModalityMotionLK::update(Image& image, PatchSet* patchSet, Rect bounds) {

            Ptr<PatchSet> patches = Ptr<PatchSet>(reliablePatchesFilter.empty() ? patchSet : patchSet->filter(*reliablePatchesFilter));

            Point2f globalMotion(0, 0);
            float w = 0;

            for (int i = 0; i < patches->size(); i++) {
                if (patches->get_age(i) < step) { continue; }

                Point2f current = patches->get_position(i, 0);
                Point2f past = patches->get_position(i, step - 1);
                globalMotion.x += (current.x - past.x) * patches->get_weight(i);
                globalMotion.y += (current.y - past.y) * patches->get_weight(i);
                w += patches->get_weight(i);
            }

            if (w != 0) {
                globalMotion.x /= w;
                globalMotion.y /= w;
                motion.push(globalMotion);

                Mat* img = new Mat();
                image.get_gray().copyTo(*img);

                history.push(Ptr<Mat>(img));
            }

            if (!usable())
            { return; }

            if (map.empty()) {
                map.create(image.height(), image.width(), CV_32F);
                map.setTo(0);
            }

            float texture_threshold = 3;

            // a simple hack (otherwise this function takes too much time)
            Rect roi = intersection(image.get_roi(), expand(patches->region(), 50));
            Mat grayscale = image.get_gray();
            vector<Point2f> points(50);
            Point2f offset = roi.tl();
            /*
                Mat eigen;
                eigen.create(Size(image.width(), image.height()), CV_32F);
                cornerMinEigenVal(grayscale(roi), eigen, block_size, aperture_size);

                for (int y = 0 ; y < eigen.rows ; y++ ) {
                    float* row = eigen.ptr<float>(y);
                    for (int x = 0 ; x < eigen.cols ; x++ ) {
                         if (row[x] > texture_threshold ) {
                            points.push_back(Point2f(x, y) + offset);
                         }
                     }
                }
            */
            goodFeaturesToTrack(grayscale(roi), points, 150, 0.05, 8, Mat(), block_size);

            for (int i = 0; i < points.size(); i++)
            { points[i] += offset; }

            TermCriteria termination =
                TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
                             OPTICALFLOW_MAX_ITERATIONS,
                             OPTICALFLOW_MAX_RESIDUE);

            Ptr<Mat> img1 = history.get(0);
            Ptr<Mat> img2 = history.get(step - 1);

            vector<Point2f> prediction;
            vector<uchar> status;
            vector<float> error;

            calcOpticalFlowPyrLK(*img1, *img2, points, prediction, status, error, Size(window_size, window_size), levels, termination);

            map *= persistence;



            //map.setTo(0);
            map += 0.000001;

            //Point2f referenceMotion = motion.get(step-1) -  motion.get(0);
            Point2f referenceMotion = motion.get(0);

            for (int i = 0; i < points.size(); i++) {
                if (!status[i]) { continue; }

                Point p = points[i];
                Point2f predictedMotion = points[i] - prediction[i];

                float norm = exp(- distance(predictedMotion - referenceMotion) / damping);


                map.at<float>(p.y, p.x) += norm * (1 - persistence);


            }

            double max, min;
            minMaxLoc(map, &min, &max, NULL, NULL, Mat());




        }

        bool ModalityMotionLK::usable() {
            return history.size() == history.limit();
        }

        void ModalityMotionLK::probability(Image& image, Mat& p) {

            if (!usable()) {
                p.setTo(1);
            } else {
                Mat c = map(image.get_roi());


                //gaussian->apply(c, p);
                //  gaussian = createGaussianFilter(CV_32F, Size(MOTION_LK_SIZE, MOTION_LK_SIZE), sigma, sigma);

                float sigma = 0.3 * (MOTION_LK_SIZE / 2 - 1) + 0.8;

                GaussianBlur(c, p, Size(MOTION_LK_SIZE, MOTION_LK_SIZE), sigma, 0);

                //p *= -1;
                //p += 1;
            }

            p /= sum(p)[0];

        }

    }

}
