#ifndef PREMIUM_IMU_CPP
#define PREMIUM_IMU_CPP

#include <imu_lpms_cu3/imu_lpms_cu3_aux.hpp>

namespace drc_flexnav_sensors
{

    /* internal use, for json convertion */

    void jsonCoord3dIntStr(char *dest, const char *name, const int32_t v1, const int32_t v2, const int32_t v3)
    {
        sprintf(dest, "\"%s\":[%d, %d, %d]", name, v1, v2, v3);
    }

    void jsonCoord3dFloatStr(char *dest, const char *name, const float v1, const float v2, const float v3)
    {
        sprintf(dest, "\"%s\":[%f, %f, %f]", name, v1, v2, v3);
    }

    void jsonCoord4dIntStr(char *dest, const char *name, const int32_t v1, const int32_t v2, const int32_t v3, const int32_t v4)
    {
        sprintf(dest, "\"%s\":[%d, %d, %d, %d]", name, v1, v2, v3, v4);
    }

    void jsonCoord4dFloatStr(char *dest, const char *name, const float v1, const float v2, const float v3, const float v4)
    {
        sprintf(dest, "\"%s\":[%f, %f, %f, %f]", name, v1, v2, v3, v4);
    }

    void jsonFloatStr(char *dest, const char *name, const float v1)
    {
        sprintf(dest, "\"%s\":%f", name, v1);
    }

    void jsonIntStr(char *dest, const char *name, const int32_t v1)
    {
        sprintf(dest, "\"%s\":%d", name, v1);
    }

    void jsonHexStr(char *dest, const char *name, const int32_t v1)
    {
        sprintf(dest, "\"%s\":%04X", name, v1);
    }

    void jsonStr(char *dest, const char *name, const int value)
    {
        sprintf(dest, "\"%s\":%d", name, value);
    }
}

#endif
