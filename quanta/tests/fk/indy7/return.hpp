#ifndef FK_INDY7_RETURN_HPP
#define FK_INDY7_RETURN_HPP
#include "../../indy7_utils/config.hpp"
#include <cstdint>

typedef struct oMi{
    double rotation[3][3];
    double translation[3];
} oMi;

typedef struct fk_return{
    oMi oMis[NU];
    double v[NU][6];
    double a[NU][6];
} fk_return;

typedef struct oMi_float{
    float rotation[3][3];
    float translation[3];
} oMi_float;

typedef struct fk_return_float{
    oMi_float oMis[NU];
    float v[NU][6];
    float a[NU][6];
} fk_return_float;

typedef struct oMi_fixed32{
    int32_t rotation[3][3];
    int32_t translation[3];
} oMi_fixed32;

typedef struct fk_return_fixed32{
    oMi_fixed32 oMis[NU];
    int32_t v[NU][6];
    int32_t a[NU][6];
} fk_return_fixed32;

typedef struct oMi_fixed16{
    int16_t rotation[3][3];
    int16_t translation[3];
} oMi_fixed16;

typedef struct fk_return_fixed16{
    oMi_fixed16 oMis[NU];
    int16_t v[NU][6];
    int16_t a[NU][6];
} fk_return_fixed16;


#endif