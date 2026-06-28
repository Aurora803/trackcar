#ifndef YAHBOOM_TRACKER8_IO_H
#define YAHBOOM_TRACKER8_IO_H

#include "tracker8_if.h"

#ifdef __cplusplus
extern "C" {
#endif

const tracker8_driver_t *YahboomTracker8IO_GetDriver(void);
void YahboomTracker8IO_Init(void);
tracker8_sample_t YahboomTracker8IO_Read(void);

#ifdef __cplusplus
}
#endif

#endif /* YAHBOOM_TRACKER8_IO_H */
