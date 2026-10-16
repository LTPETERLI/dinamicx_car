//
// Created by kaxns on 3/14/22.
//

#include "windmill_dev/image_converter.h"

namespace rm_windmill_vision
{
ColorProcessor::Scalar ColorProcessor::lower4blue_{ 26, 80, 80 };
ColorProcessor::Scalar ColorProcessor::upper4blue_{ 34, 255, 255 };
ColorProcessor::Scalar ColorProcessor::lower4red_{ 78, 80, 80 };
ColorProcessor::Scalar ColorProcessor::upper4red_{ 99, 255, 255 };

void ImageConverterBase::reConfigCB(Config& config, uint32_t __attribute__((unused)) level)
{
  *(lower4blue_.val)/* */= config.h_min4blue;
  *(lower4blue_.val + 1) = config.s_min4blue;
  *(lower4blue_.val + 2) = config.v_min4blue;

  *(upper4blue_.val)/* */= config.h_max4blue;
  *(upper4blue_.val + 1) = config.s_max4blue;
  *(upper4blue_.val + 2) = config.v_max4blue;

  *(lower4red_.val)/* */= config.h_min4red;
  *(lower4red_.val + 1) = config.s_min4red;
  *(lower4red_.val + 2) = config.v_min4red;

  *(upper4red_.val)/* */= config.h_max4red;
  *(upper4red_.val + 1) = config.s_max4red;
  *(upper4red_.val + 2) = config.v_max4red;
}

void ImageConverterBase::init(const ros::NodeHandle& nh)
{
  image_converter_cfg_srv_ = std::make_unique<dynamic_reconfigure::Server<Config>>(nh);
  image_converter_cfg_cb_ = boost::bind(&ImageConverterBase::reConfigCB, this, _1, _2);
  image_converter_cfg_srv_->setCallback(image_converter_cfg_cb_);
}
}  // namespace rm_windmill_vision