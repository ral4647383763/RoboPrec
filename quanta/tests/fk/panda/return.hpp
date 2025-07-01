#ifndef FK_PANDA_RETURN_HPP
#define FK_PANDA_RETURN_HPP
#include "../../panda_utils/config.hpp"

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