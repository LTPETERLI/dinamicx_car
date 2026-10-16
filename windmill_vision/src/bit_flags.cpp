//
// Created by koonyu on 2022/7/26.
//

#include "windmill_dev/bit_flags.h"

BitFlags<scope, uint8_t> Flags::flags_{scope::TREAT_HAMMER_ONLY,
                                       scope::PNP_MODELING_TARGET_ONLY,
                                       scope::EXTRACT_VERTICES_CONVENTIONAL,
                                       scope::ENABLE_ROTATION_PREDICTION};