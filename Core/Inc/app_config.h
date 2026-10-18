#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#ifndef APP_LAP_SIM_ENABLED
#define APP_LAP_SIM_ENABLED 0
#endif

/* Less than one 20 ms sample: never replay several stale lateral decisions. */
#ifndef APP_LAP_SIM_MAX_LATENESS_MS
#define APP_LAP_SIM_MAX_LATENESS_MS 10U
#endif

#if APP_LAP_SIM_ENABLED != 0 && APP_LAP_SIM_ENABLED != 1
#error "APP_LAP_SIM_ENABLED must be 0 or 1"
#endif
#if APP_LAP_SIM_MAX_LATENESS_MS >= 20U
#error "Lap timing tolerance must be less than the 20 ms sample period"
#endif

#endif
