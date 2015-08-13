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

#include "common/math/geometry.h"

#include "modalities.h"

#define __MODALITY_REGISTRATION

#include "color.h"
#include "motion.h"
#include "shape.h"

namespace legit
{

namespace tracker
{

Modalities::Modalities(Config& config)
{


        modalities.push_back(Ptr<Modality>(new ModalityColor3DHistogram(config, "")));


        modalities.push_back(Ptr<Modality>(new ModalityConvex(config, "cuename")));


        modalities.push_back(Ptr<Modality>(new ModalityMotionLK(config, "cuename")));


      //  modalities.push_back(Ptr<Modality>(new ModalityBounding(config, cuename)));

    




}

Modalities::~Modalities()
{

}

void Modalities::flush()
{

  for (int i = 0; i < modalities.size(); i++)
    {
      modalities[i]->flush();
    }

}

void Modalities::update(Image& image, PatchSet* patches, Rect bounds)
{

  for (int i = 0; i < modalities.size(); i++)
    {
      modalities[i]->update(image, patches, bounds);
    }

}

void Modalities::probability(Image& image, Mat& p)
{

  Rect region = image.get_roi();
  cv::Point modalityOffset(0, 0);

  if (region.width == 0 || region.height == 0)
    {
      p.release();
      return;
    }


  Mat pmap;
  pmap.create(region.height, region.width, CV_32FC1);

  p.create(region.height, region.width, CV_32FC1);
  p.setTo(1 / (float) (region.height * region.width));

  bool usable = false;



  for (int i = 0; i < modalities.size(); i++)
    {

      if (!modalities[i]->usable())
        {

          continue;
        }

      modalities[i]->probability(image, pmap);

      if (pmap.empty()) continue;



      p = p.mul(pmap);
      usable = true;
    }

  if (!usable)
    p.setTo(0);


}

void Modalities::probability(Image& image, Mat& p, int i)
{

  Rect region = image.get_roi();

  if (region.width == 0 || region.height == 0)
    {
      p.release();
      return;
    }

  p.create(region.height, region.width, CV_32FC1);
  if (!modalities[i]->usable())
    {
      p.setTo(1 / (float) (region.height * region.width));
      return;
    }

  modalities[i]->probability(image, p);

}

int Modalities::size()
{
  return modalities.size();

}


Modality::Modality(Config& config, string configbase)
{

  double weight = 0.5;
  int age = 0;

  reliablePatchesFilter = Ptr<ReliablePatchesFilter>(new ReliablePatchesFilter(weight, age));


}

}

}
