/*
 * debug_utils.h
 *
 *  Created on: 7 wrz 2026
 *      Author: karab
 */

#ifndef DEBUG_UTILS_H_
#define DEBUG_UTILS_H_

#include <stdint.h>
#include "app_types.h"

	const char* WingState_ToString(WingState_t state);

	void Print_Signed_X10(int32_t value_x10);

	void Print_Signed_mG_As_G(int32_t value_mg);

#endif /* DEBUG_UTILS_H_ */
