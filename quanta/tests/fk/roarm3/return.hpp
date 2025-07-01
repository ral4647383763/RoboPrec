#ifndef FK_ROARM3_RETURN_HPP
#define FK_ROARM3_RETURN_HPP
#include "../../roarm3_utils/config.hpp"

typedef struct oMi{
    double rotation[3][3];
    double translation[3];
} oMi;

typedef struct fk_return{
    oMi oMis[NU];
    double v[NU][6];
    double a[NU][6];
} fk_return;
#endif