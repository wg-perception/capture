/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * A cell that finds features in an image and return their 2d/3d positions
 */
struct FeatureFinder {
  static void declare_params(ecto::tendrils& params) {
    params.declare(&FeatureFinder::use_fast_, "use_fast",
        "Whether to use FAST keypoints or not (otherwise, ORB is used)", false);
    params.declare(&FeatureFinder::n_features_, "n_features", "The number of keypoints to use", 1000);
    params.declare(&FeatureFinder::n_levels_, "n_levels", "The number of levels to use for ORB", 3);
    params.declare(&FeatureFinder::scale_factor_, "scale_factor", "The scale factor to use for ORB", 1.2);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs) {
    inputs.declare(&FeatureFinder::image_, "image", "The image on which to find keypoints.").required();
    inputs.declare(&FeatureFinder::points3d_in_, "points3d", "The 3d points matching the image.").required();
    inputs.declare(&FeatureFinder::mask_, "points3d_mask", "The mask of where to find keypoints in the image.");

    outputs.declare(&FeatureFinder::points_, "points", "The 2d points in the image.");
    outputs.declare(&FeatureFinder::points3d_out_, "points3d", "The 3d points");
    outputs.declare(&FeatureFinder::keypoints_, "keypoints", "The keypoints");
    outputs.declare(&FeatureFinder::descriptors_, "descriptors", "The descriptors");
  }

  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
    // case 0: more elongated that image_, case 1: morevertical, case 2: same ratio
    int ratio_case;
    float ratio2d_x, ratio2d_y;
    if (points3d_in_->cols * image_->rows > points3d_in_->rows * image_->cols) {
      ratio2d_x = float(points3d_in_->cols) / float(image_->cols);
      ratio2d_y = ratio2d_x;
      ratio_case = 0;
    } else if (points3d_in_->cols * image_->rows < points3d_in_->rows * image_->cols) {
      ratio2d_y = float(points3d_in_->rows) / float(image_->rows);
      ratio2d_x = ratio2d_y;
      ratio_case = 1;
    } else {
      ratio2d_y = float(points3d_in_->rows) / float(image_->rows);
      ratio2d_x = ratio2d_y;
      ratio_case = 2;
    }

    // Create a mask to fins the keypoints
    cv::Mat_<uchar> mask(image_->size());
    if (!mask_->empty()) {
      switch (ratio_case) {
      case 0: {
        cv::Mat sub_mask_top = mask.rowRange(0, float(mask_->rows) / ratio2d_x);
        cv::resize(*mask_, sub_mask_top, sub_mask_top.size());
        mask.rowRange(sub_mask_top.rows, mask.rows).setTo(0);
        break;
      }
      case 1: {
        cv::Mat sub_mask_left = mask.colRange(0, float(mask_->cols) / ratio2d_y);
        cv::resize(*mask_, sub_mask_left, sub_mask_left.size());
        mask.colRange(sub_mask_left.cols, mask.cols).setTo(0);
        break;
      }
      default:
        cv::resize(*mask_, mask, mask.size());
        break;
      }
    }

    // Find keypoints in the current image
#if (CV_MAJOR_VERSION ==3)
    cv::Ptr<cv::ORB> orb = cv::ORB::create(*n_features_, *scale_factor_, *n_levels_);
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION >= 4)
    cv::ORB orb = cv::ORB(*n_features_, *scale_factor_, *n_levels_);
#else
    cv::ORB::CommonParams orb_params;
    orb_params.first_level_ = 0;
    orb_params.n_levels_ = *n_levels_;
    orb_params.scale_factor_ = *scale_factor_;
    cv::ORB orb = cv::ORB(*n_features_, orb_params);
#endif

    cv::Mat descriptors;
    if (*use_fast_) {
      cv::FAST(*image_, *keypoints_, 30);
#if (CV_MAJOR_VERSION ==3)
      orb->detectAndCompute(*image_, mask, *keypoints_, descriptors, true);
#else
      orb(*image_, mask, *keypoints_, descriptors, true);
#endif
    } else {
#if (CV_MAJOR_VERSION ==3)
      orb->detectAndCompute(*image_, mask, *keypoints_, descriptors);
#else
      orb(*image_, mask, *keypoints_, descriptors);
#endif
    }

    // Remove bad keypoints
    std::vector<cv::Vec2f> points;
    std::vector<cv::Vec3f> points3d;
    points.reserve(keypoints_->size());
    points3d.reserve(keypoints_->size());
    descriptors_->create(descriptors.size(), descriptors.type());
    for (size_t i = 0; i < keypoints_->size(); ++i) {
      int x = (*keypoints_)[i].pt.x * ratio2d_x;
      int y = (*keypoints_)[i].pt.y * ratio2d_y;
      if ((x >= points3d_in_->cols) || (y >= points3d_in_->rows))
        continue;
      points.push_back(cv::Vec2f((*keypoints_)[i].pt.x, (*keypoints_)[i].pt.y));
      points3d.push_back(points3d_in_->at<cv::Vec3f>(y, x));
      descriptors.row(i).copyTo(descriptors_->row(points3d.size() - 1));
    }

    // Store the points
    if (points.empty()) {
      points_->release();
      points3d_out_->release();
      descriptors_->release();
    } else {
      *points_ = cv::Mat(points, true);
      *points3d_out_ = cv::Mat(points3d, true);
      descriptors.rowRange(0, points3d.size()).copyTo(*descriptors_);
    }

    return ecto::OK;
  }
private:
  /** Input plane coefficients */
  ecto::spore<cv::Mat> points3d_in_, points3d_out_, points_;
  /** The input image */
  ecto::spore<cv::Mat> image_;
  /** The mask of where to find keypoints in the image */
  ecto::spore<cv::Mat> mask_;
  /** ORB parameters */
  ecto::spore<int> n_features_, n_levels_;
  ecto::spore<float> scale_factor_;
  /** The resulting descriptors */
  ecto::spore<cv::Mat> descriptors_;
  /** Thekeypoints */
  ecto::spore<std::vector<cv::KeyPoint> > keypoints_;

  /** Whether we use fast key points or not */
  ecto::spore<bool> use_fast_;
};

ECTO_CELL(capture, FeatureFinder, "FeatureFinder", "Find 2dfeatures and assign  them a depth")
