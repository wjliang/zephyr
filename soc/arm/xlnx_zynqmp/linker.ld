/* linker.ld - Linker command/script file */

/*
 * Copyright (c) 2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <autoconf.h>

#ifdef CONFIG_SOC_XLNX_ZYNQMP_R5
#include <arch/arm/cortex_r/scripts/linker.ld>
#else
#error("No linker script is found for ZynqMP")
#endif
