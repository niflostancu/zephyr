/*
 * Copyright (c) 2021 Florin Stancu <niflostancu@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ADC_CC13XX_CC26XX_H_
#define ZEPHYR_DRIVERS_ADC_CC13XX_CC26XX_H_

#include <stdint.h>
#include <kernel.h>

#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/aux_adc.h>


struct adc_cc13xx_cc26xx_sample_time_entry {
	uint16_t time_us;
	uint8_t reg_value;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_CC13XX_CC26XX_SUBG_H_ */

