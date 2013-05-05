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
#include <opencv2/imgproc/imgproc.hpp>

/**
 * A cell that takes a set of planes and returns the most central one. This is useful for the TOD tracker
 * as it needs to focus on features on a plane to find a homography.
 * If the origin is given, it is assumed that there might be a scale ambiguity and the origin is sought on
 * the ray
 */
struct MaskDisplay {
  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs,
      ecto::tendrils& outputs) {
    inputs.declare(&MaskDisplay::img_rgb_, "image", "The image to mask.").required();
    inputs.declare(&MaskDisplay::img_mask_, "mask",
        "The mask toapply to the image.").required();

    outputs.declare(&MaskDisplay::img_out_, "image", "The masked out image.");
  }

  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs) {
    *img_out_ = img_rgb_->clone();
    cv::Mat mask;
    if (img_rgb_->size() == img_mask_->size()) {
      mask = *img_mask_;
    } else {
      cv::resize(*img_mask_, mask, cv::Size(img_rgb_->cols, (img_mask_->rows*img_rgb_->cols)/img_mask_->cols));
      mask.resize(img_rgb_->rows, cv::Scalar(0));
    }
    cv::bitwise_not(mask, mask);
    img_out_->setTo(cv::Scalar(0, 0, 0), mask);

    return ecto::OK;
  }
private:
  /** Input image */
  ecto::spore<cv::Mat> img_rgb_;
  /** Input mask */
  ecto::spore<cv::Mat> img_mask_;
  /** Output image */
  ecto::spore<cv::Mat> img_out_;
};

ECTO_CELL(capture, MaskDisplay, "MaskDisplay",
    "Display an RGB image masked out")
