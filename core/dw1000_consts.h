/**
 * @file dw1000_consts.h
 * @brief Carrier-frequency offset constants (from Decawave API v04, used by ranging.c).
 */
#ifndef DW1000_CONSTS_H_
#define DW1000_CONSTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DWT_TIME_UNITS                  (1.0/499.2e6/128.0)
#define FREQ_OFFSET_MULTIPLIER          (998.4e6/2.0/1024.0/131072.0)
#define FREQ_OFFSET_MULTIPLIER_110KB    (998.4e6/2.0/8192.0/131072.0)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_1  (-1.0e6/3494.4e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_2  (-1.0e6/3993.6e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_3  (-1.0e6/4492.8e6)
#define HERTZ_TO_PPM_MULTIPLIER_CHAN_5  (-1.0e6/6489.6e6)

#ifdef __cplusplus
}
#endif

#endif /* DW1000_CONSTS_H_ */
