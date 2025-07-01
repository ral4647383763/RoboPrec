#ifndef RNEA_INDY7_RETURN_HPP
#define RNEA_INDY7_RETURN_HPP
#include "../../indy7_utils/config.hpp"
#include <cstdint>

typedef struct rnea_return{
    double taus[NU];
} rnea_return;

typedef struct rnea_return_float{
    float taus[NU];
} rnea_return_float;

typedef struct rnea_return_fixed32{
    int32_t taus[NU];
} rnea_return_fixed32;

typedef struct rnea_return_fixed16{
    int16_t taus[NU];
} rnea_return_fixed16;
#endif