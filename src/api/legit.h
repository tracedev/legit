/*******************************************************************************
* Copyright (c) 2013, Luka Cehovin
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the University of Ljubljana nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL LUKA CEHOVIN BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*- */

#ifndef LEGIT_API
#define LEGIT_API

#ifdef __cplusplus
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

using namespace cv;
using namespace std;

#define LEGIT_PROPERTY_JAVA_RESCALE_WIDTH 1001
#define LEGIT_PROPERTY_JAVA_RESCALE_HEIGHT 1002

class LegitTracker {

public:

  LegitTracker(const char* id = "default");
  ~LegitTracker();

  void clear_image();

  void update_image(Mat& image, int imagetype = -1);

  void initialize(cv::Rect region);

  void update();

  void initialize(Mat& image, cv::Rect region);

  void update(Mat& image);

  cv::Rect region();

  cv::Point2f position();

  bool is_tracking();

  void visualize(Mat& canvas);

  string get_name();

  void set_property(int code, float value);

  float get_property(int code);

  void remove_property(int code);

  bool has_property(int code);

private:

  class Impl;

  Ptr<Impl> impl;

};
#else
#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#endif
#endif
