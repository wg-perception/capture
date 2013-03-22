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

/** Returns a value that is for sure in [min, max[
 * @param min
 * @param max
 * @param val
 * @return
 */
int filterMinMax(int min, int max, int val) {
  if (max <= min)
    throw std::runtime_error("Max inferior to Min in filterMinMax");
  if (val < min)
    return min;
  else if (val >= max)
    return max - 1;
  else
    return val;
}

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
    params.declare(&PlaneFilter::do_center_, "do_center", "If set to true, the plane origin will be at the center of the image.", false);
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
    outputs.declare(&PlaneFilter::R_out_, "R", "The rotation component of the plane pose");
    outputs.declare(&PlaneFilter::T_out_, "T", "The translation component of the plane pose");
    outputs.declare(&PlaneFilter::found_, "found", "Whether or not the R|T is good.");
  }

  int
  process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    *found_ = false;
    *R_out_ = cv::Mat();
    *T_out_ = cv::Mat();

    // Get the origin of the plane
    cv::Point origin;

    cv::Matx33f K, R;
    cv::Vec3f T;
    if (*do_center_)
      origin = cv::Point(masks_->cols/2, masks_->rows/2);
    else {
      if (!K_ || K_->empty() || !R_in_ || R_in_->empty() || !T_in_ || T_in_->empty())
        return ecto::OK;
      T = *T_in_;
      if (cvIsNaN(T(0)))
        return ecto::OK;
      K = *K_;
      R = *R_in_;
      cv::Vec3f T_tmp = K*T;
      origin = cv::Point(T_tmp(0)/T_tmp(2), T_tmp(1)/T_tmp(2));
    }

    // Go over the plane masks and simply count the occurrence of each mask
    std::vector<int> occurrences(256, 0);
    for (int y = filterMinMax(0, masks_->rows, origin.y - *window_size_);
        y < filterMinMax(0, masks_->rows, origin.y + *window_size_); ++y) {
      const uchar *mask = masks_->ptr<uchar>(y)
          + filterMinMax(0, masks_->cols, origin.x - *window_size_);
      const uchar *mask_end = masks_->ptr<uchar>(y)
          + filterMinMax(0, masks_->cols, origin.x + *window_size_);
      for (; mask != mask_end; ++mask)
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

    if (best_index < 0)
      return ecto::OK;

    // Convert the plane coefficients to R,t
    cv::Matx33f rotation;
    cv::Vec3f translation;
    *coeffs_ = (*planes_)[best_index];
    float a = (*coeffs_)[0], b = (*coeffs_)[1], c = (*coeffs_)[2], d =
        (*coeffs_)[3];

    // Deal with translation
    if (*do_center_) {
      getPlaneTransform(*coeffs_, rotation, translation);
      // Make sure T_ points to the center of the image
      translation = cv::Vec3f(0, 0, -d / c);
    } else {
      // Have T_ point to the origin. Find alpha such that alpha*Kinv*origin is on the plane
      cv::Matx33f K_inv = K.inv();
      cv::Vec3f origin_inv = cv::Mat(K_inv * cv::Vec3f(origin.x, origin.y, 1));
      float alpha = -d
          / (a * origin_inv(0) + b * origin_inv(1) + c * origin_inv(2));
      translation = alpha * origin_inv;
      if (translation(2) < 0)
        translation = -translation;

      // Make the rotation fit to the plane (but as close as possible to the current estimate
      // Get the Z axis
      cv::Vec3f N(a, b, c);
      N = N / cv::norm(N);
      // Get the X, Y axes
      cv::Vec3f vecX(R(0, 0), R(1, 0), R(2, 0));
      cv::Vec3f vecY(R(0, 1), R(1, 1), R(2, 1));
      // Project them onto the plane
      vecX = vecX - vecX.dot(N) * N;
      vecY = vecY - vecY.dot(N) * N;
      vecX = vecX / cv::norm(vecX);
      vecY = vecY / cv::norm(vecY);
      // Get the median
      cv::Vec3f median = vecX + vecY;
      median = median / cv::norm(median);
      // Get a new basis
      cv::Vec3f vecYtmp = vecY - median.dot(vecY) * median;
      cv::Vec3f vecXtmp = vecX - median.dot(vecX) * median;
      vecYtmp = vecYtmp / cv::norm(vecYtmp);
      vecXtmp = vecXtmp / cv::norm(vecXtmp);
      // Get the rectified X/Y axes
      cv::Vec3f vecXnew = median + vecXtmp;
      cv::Vec3f vecYnew = median + vecYtmp;
      vecXnew = vecXnew / cv::norm(vecXnew);
      vecYnew = vecYnew / cv::norm(vecYnew);
      // Fill in the matrix
      rotation = cv::Matx33f(vecXnew(0), vecYnew(0), N(0), vecXnew(1),
                             vecYnew(1), N(1), vecXnew(2), vecYnew(2), N(2));
    }

    *R_out_ = cv::Mat(rotation);
    *T_out_ = cv::Mat(translation);
    *found_ = true;

    return ecto::OK;
  }
private:
  /** Input plane coefficients */
  ecto::spore<std::vector<cv::Vec4f> > planes_;
  /** Input mask of the planes */
  ecto::spore<cv::Mat> masks_;

  /** The ouput plane rotation */
  ecto::spore<cv::Mat> R_out_;
  /** The ouput plane translation */
  ecto::spore<cv::Mat> T_out_;

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
  /** Whether the plane origin is the center of the image or not */
  ecto::spore<bool> do_center_;
};

ECTO_CELL(capture, PlaneFilter, "PlaneFilter",
          "Takes a set of planes and returns the one overlaping most with the center of the image")
