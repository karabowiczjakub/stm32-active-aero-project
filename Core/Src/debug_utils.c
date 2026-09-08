/*
 * debug_utils.c
 *
 *  Created on: 7 wrz 2026
 *      Author: karab
 */


#include "debug_utils.h"

#include <stdio.h>


const char* WingState_ToString(WingState_t state)
{
    switch (state)
    {
        case WING_NORMAL:
            return "NORMAL";

        case WING_OPEN:
            return "OPEN";

        case WING_AIR_BRAKE:
            return "AIR_BRAKE";

        default:
            return "UNKNOWN";
    }
}


void Print_Signed_X10(int32_t value_x10)
{
    if (value_x10 < 0)
    {
        printf("-");
        value_x10 = -value_x10;
    }

    printf("%ld.%ld",
           (long)(value_x10 / 10),
           (long)(value_x10 % 10));
}


void Print_Signed_mG_As_G(int32_t value_mg)
{
    if (value_mg < 0)
    {
        printf("-");
        value_mg = -value_mg;
    }

    printf("%ld.%03ld",
           (long)(value_mg / 1000),
           (long)(value_mg % 1000));
}
