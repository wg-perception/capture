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

/**
 * If the equation of the plane is ax+by+cz+d=0, the pose (R,t) is such that it takes the horizontal plane (z=0)
 * to the current equation
 */
void
getPlaneTransform(const cv::Vec4f& plane_coefficients, cv::Matx33f& rotation, cv::Vec3f& translation)
{
  double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
  // assume plane coefficients are normalized
  translation = cv::Vec3f(-a * d, -b * d, -c * d);
  cv::Vec3f z(a, b, c);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}

/**
 * A cell that takes a set of planes and returns the most central one. This is useful for the TOD tracker
 * as it needs to focus on features on a plane to find a homography.
 * If the origin is given, it is assumed that there might be a scale ambiguity and the origin is sought on
 * the ray
 */
struct PlaneFilter
{
  static void
  declare_params(ecto::tendrils& params)
  {
    params.declare(&PlaneFilter::window_size_, "size", "The edge of the central square in which to look for the plane.", 100);
  }

  static void
  declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare(&PlaneFilter::planes_, "planes",
                      "The different found planes (a,b,c,d) of equation ax+by+cz+d=0.").required();
    inputs.declare(&PlaneFilter::masks_, "masks", "The masks for each plane.").required();
    inputs.declare(&PlaneFilter::K_, "K", "The calibration matrix.");
    inputs.declare(&PlaneFilter::R_in_, "R", "The currently estimated plane rotation.");
    inputs.declare(&PlaneFilter::T_in_, "T", "The currently estimated plane origin.");

    outputs.declare(&PlaneFilter::coeffs_, "coeffs", "The coefficients of the plane.");
    outputs.declare(&PlaneFilter::R_, "R", "The rotation component of the plane pose");
    outputs.declare(&PlaneFilter::T_, "T", "The translation component of the plane pose");
    outputs.declare(&PlaneFilter::found_, "found", "Whether or not the R|T is good.");
  }

  int
  process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    // Get the origin of the plane
    cv::Point origin;
    if (T_in_) {
      cv::Vec3f T = cv::Mat((*K_) * (*T_in_));
      origin = cv::Point(T(0)/T(2), T(1)/T(2));
    } else
      origin = cv::Point(masks_->cols/2, masks_->rows/2);

    // Go over the plane masks and simply count the occurrence of each mask
    std::vector<int> occurrences(256, 0);
    for(int y = std::max(0, origin.y - *window_size_); y < std::min(masks_->rows, origin.y + *window_size_); ++y) {
      uchar *mask = masks_->ptr<uchar>(y) + std::max(0, origin.x - *window_size_);
      uchar *mask_end = masks_->ptr<uchar>(y) + std::min(masks_->cols, origin.x + *window_size_);
      for(; mask != mask_end; ++mask)
        if (*mask != 255)
          ++occurrences[*mask];
    }
    // Find the most common plane
    int best_index = -1;
    int best_count = 0;
    for(size_t i = 0; i < 255; ++i) {
      if (occurrences[i] > best_count) {
        best_index = i;
        best_count = occurrences[i];
      }
    }
    // Convert the plane coefficients to R,t
    cv::Matx33f rotation;
    cv::Vec3f translation;
    if (best_index >= 0) {
      *coeffs_ = (*planes_)[best_index];
      float a = (*coeffs_)[0], b = (*coeffs_)[1], c = (*coeffs_)[2], d = (*coeffs_)[3];

      if (T_in_) {
        rotation = *R_in_;
        // Have T_ point to the origin. Find alpha such that alpha*Kinv*origin is on the plane
        cv::Matx33f K_inv = cv::Mat(K_->inv());
        cv::Vec3f origin_inv = cv::Mat(K_inv * cv::Vec3f(origin.x, origin.y, 1));
        float alpha = -d/(a*origin_inv(0) + b*origin_inv(1) + c*origin_inv(2));
        translation = alpha*origin_inv;
        if (translation(2) < 0)
          translation = -translation;
      } else {
        getPlaneTransform(*coeffs_, rotation, translation);
        // Make sure T_ points to the center of the image
        translation = cv::Vec3f(0,0,-d/c);
      }
      *R_ = cv::Mat(rotation);
      *T_ = cv::Mat(translation);
      *found_ = true;
    } else
      *found_ = false;

    return ecto::OK;
  }
private:
  /** Input plane coefficients */
  ecto::spore<std::vector<cv::Vec4f> > planes_;
  /** Input mask of the planes */
  ecto::spore<cv::Mat> masks_;

  /** The ouput plane rotation */
  ecto::spore<cv::Mat> R_;
  /** The ouput plane translation */
  ecto::spore<cv::Mat> T_;

  /** The edge of the square at the center of the image in which to look for the biggest plane */
  ecto::spore<int> window_size_;
  ecto::spore<cv::Vec4f> coeffs_;
  /** The calibration matrix */
  ecto::spore<cv::Mat> K_;
  /** the currently estimated plane pose */
  ecto::spore<cv::Mat> R_in_;
  ecto::spore<cv::Mat> T_in_;
  /** Whether a pose was found or not */
  ecto::spore<bool> found_;
};

ECTO_CELL(capture, PlaneFilter, "PlaneFilter",
          "Takes a set of planes and returns the one overlaping most with the center of the image")
