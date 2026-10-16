//
// Created by koonyu on 2022/7/26.
//

#include "../include/buff_vision/bit_flags.h"

BitFlags<scope, uint8_t> Flags::flags_{scope::TREAT_ALL,
                                       scope::PNP_MODELING_ALL,
                                       scope::EXTRACT_VERTICES_CONVENTIONAL,
                                       scope::ENABLE_ROTATION_PREDICTION};