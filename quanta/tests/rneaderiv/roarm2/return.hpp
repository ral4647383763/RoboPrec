#ifndef RNEADERIV_Roarm2_RETURN_HPP
#define RNEADERIV_Roarm2_RETURN_HPP
#include "../../roarm2_utils/config.hpp"
#include <cstdint>

typedef struct rneaderiv_return{
    double partial_da[NU][NU];
    double partial_dv[NU][NU];
    double partial_dq[NU][NU];
} rneaderiv_return;

typedef struct rneaderiv_return_float{
    float partial_da[NU][NU];
    float partial_dv[NU][NU];
    float partial_dq[NU][NU];
} rneaderiv_return_float;

typedef struct rneaderiv_return_fixed32{
    int32_t partial_da[NU][NU];
    int32_t partial_dv[NU][NU];
    int32_t partial_dq[NU][NU];
} rneaderiv_return_fixed32;

#endif