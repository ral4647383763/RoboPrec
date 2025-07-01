#ifndef RNEADERIV_Panda_RETURN_HPP
#define RNEADERIV_Panda_RETURN_HPP
#include "../../panda_utils/config.hpp"

typedef struct rneaderiv_return{
    double partial_da[NU][NU];
    double partial_dv[NU][NU];
    double partial_dq[NU][NU];
} rneaderiv_return;
#endif