/*
 * wing_control.h
 *
 *  Created on: 8 wrz 2026
 *      Author: karab
 */

#ifndef WING_CONTROL_H_
#define WING_CONTROL_H_

#include <stdint.h>


void Control_Update_From_Speed(
    uint8_t speed_kmh
);



void Control_ForceNormalForCorner(void);


#endif /* WING_CONTROL_H_ */
