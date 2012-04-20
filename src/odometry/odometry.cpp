/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ecto/ecto.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <string>
#include <numeric>

using ecto::tendrils;

namespace capture
{
  struct Odometry
  {
    static void
    declare_params(tendrils& params)
    {
      /*params.declare<double>("angle_thresh", "The angle thresh hold.", CV_PI / 36);
       params.declare<bool>("reset", "Reset observations.", false);
       params.declare<unsigned>("n_desired", "The number of desired views", std::numeric_limits<unsigned>::max());*/
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Odometry::current_image_, "image", "The current visual frame.").required(true);
      inputs.declare(&Odometry::current_depth_, "depth", "The current depth frame.").required(true);
      inputs.declare(&Odometry::K_, "K", "The camera intrinsic parameter matrix.").required(true);

      outputs.declare(&Odometry::R_, "R", "The rotation of the camera pose with respect to the previous frame.");
      outputs.declare(&Odometry::T_, "T", "The rotation of the camera pose with respect to the previous frame.");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      // Convert the current image to grayscale
      cv::Mat current_image_gray;
      if (current_image_->channels() == 3)
        cv::cvtColor(*current_image_, current_image_gray, CV_BGR2GRAY);
      else
        current_image_gray = *current_image_;

      // Change the depth to meters
      cv::Mat current_depth_meters;
      current_depth_->convertTo(current_depth_meters, CV_32FC1, 1. / 1000);

      // Odometry is only possible when we have a previous frame
      if (previous_image_gray_.empty())
      {
        current_image_gray.copyTo(previous_image_gray_);
        current_depth_meters.copyTo(previous_depth_meters_);
        return ecto::OK;
      }

      cv::TickMeter tm;
      cv::Mat Rt;

      cv::Mat cameraMatrix;
      K_->convertTo(cameraMatrix, CV_32FC1);

      std::vector<int> iterCounts(4);
      iterCounts[0] = 7;
      iterCounts[1] = 7;
      iterCounts[2] = 7;
      iterCounts[3] = 10;

      std::vector<float> minGradMagnitudes(4);
      minGradMagnitudes[0] = 12;
      minGradMagnitudes[1] = 5;
      minGradMagnitudes[2] = 3;
      minGradMagnitudes[3] = 1;

      const float minDepth = 0.f; //in meters
      const float maxDepth = 3.f; //in meters
      const float maxDepthDiff = 0.07f; //in meters

      tm.start();
      bool isFound = cv::RGBDOdometry(Rt, cv::Mat(), previous_image_gray_, previous_depth_meters_, cv::Mat(),
                                      current_image_gray, current_depth_meters, cv::Mat(), cameraMatrix, minDepth,
                                      maxDepth, maxDepthDiff, iterCounts, minGradMagnitudes, cv::RIGID_BODY_MOTION);
      tm.stop();

      std::cout << "Rt = " << Rt << std::endl;
      std::cout << "Time = " << tm.getTimeSec() << " sec." << std::endl;

      if (!isFound)
      {
        std::cout << "Rigid body motion cann't be estimated for given RGBD data." << std::endl;
        return -1;
      }

      // Keep track of the frames
      current_image_gray.copyTo(previous_image_gray_);
      current_depth_meters.copyTo(previous_depth_meters_);

      return ecto::OK;
    }

    ecto::spore<cv::Mat> K_;
    ecto::spore<cv::Mat> current_image_;
    ecto::spore<cv::Mat> current_depth_;
    cv::Mat previous_image_gray_;
    cv::Mat previous_depth_meters_;

    /** The output rotation matrix */
    ecto::spore<cv::Mat> R_;
    /** The output translation matrix */
    ecto::spore<cv::Mat> T_;
  };
}

ECTO_CELL(odometry, capture::Odometry, "Odometry", "Uses the RGBDOdometry to figure out where the camera is.")
