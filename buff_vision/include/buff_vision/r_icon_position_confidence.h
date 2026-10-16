//
// Created by kaxns on 3/19/22.
//

#pragma once

namespace rm_buff_vision
{
enum class RoiInfoEncodingProtocol : int
{
  INVERSE_TRANSFORM = 0,  ///< Row0 contains the inverse_trans [every ROI has]
  ACTUAL_ROI_SIZE,        ///< Row1 contains the actual size of the ROI [every ROI has]

  ROI_AMOUNT,                    ///< Row2 contains the amount of ROIs [only hammer(1st ROI) has]
  CENTRAL_R_ORIGIN_POSITION,  ///< Row3 contains the position of 'R' icon in the original frame [only hammer(1st ROI) has]
  ORIGINAL_FRAME_CENTER_POSITION,  ///< Row4 contains the position of center of original frame [only hammer(1st ROI) has]
};

enum class RIconPositionConfidence : int
{
  REALLY_TRUSTABLE = 0,
  NOT_REALLY_TRUSTABLE,
  NOT_TRUSTABLE,
};

}  // namespace rm_buff_vision