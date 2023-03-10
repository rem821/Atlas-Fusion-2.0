/*
 * Copyright 2020 Brno University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <precompiled_headers/PCH.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace AtlasFusion::Algorithms {

    class LidarFilter {

    public:

        static void FillterNearObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {
            auto backup = data->points;
            data->points.clear();

            for (const auto &point: backup) {
                // Used lidars have minimal range of 2m, this formula is quicker than using powers
                if (std::abs(point.x) + std::abs(point.y) < 2.82) {
                    continue;
                }
                data->points.push_back(point);
            }
            data->width = data->size();
        }

        static void FilterOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &data) {

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(data);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*data);
        }
    };

}
