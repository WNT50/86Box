/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Collection of the environment readings that the hardware
 *          monitor chips report to the guest: fan speeds, temperatures
 *          and voltages, including the core voltages of the active CPU.
 *
 *          A machine either fills hwm_values and lets the chips read it,
 *          or attaches a hwm_source_t that computes the readings on the
 *          fly; the second form is what dynamic models will use.
 *
 * Authors: RichardG, <richardg867@gmail.com>
 *          WNT50
 *
 *          Copyright 2020 RichardG.
 *          Copyright 2026 WNT50.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/machine.h>
#include <86box/hwm.h>
#include "cpu.h"

/* Refer to specific hardware monitor implementations for the meaning of hwm_values. */
hwm_values_t hwm_values;

/* Supplier of the readings; NULL means "report hwm_values as they are". */
static const hwm_source_t *hwm_source;

void
hwm_attach_source(const hwm_source_t *source)
{
    hwm_source = source;
}

void
hwm_detach_source(const hwm_source_t *source)
{
    if (hwm_source == source)
        hwm_source = NULL;
}

uint16_t
hwm_get_fan(unsigned idx)
{
    if (idx >= HWM_FANS)
        return 0;

    if ((hwm_source != NULL) && (hwm_source->fan_rpm != NULL))
        return hwm_source->fan_rpm(hwm_source->priv, idx);

    return hwm_values.fans[idx];
}

uint16_t
hwm_get_vcore(void)
{
    /* Determine Vcore for the active CPU. */
    return (cpu_s == NULL) ? 0x0000 : cpu_s->voltage;
}

uint16_t
hwm_get_voltage(unsigned idx)
{
    if (idx >= HWM_VOLTAGES)
        return 0;

    if ((hwm_source != NULL) && (hwm_source->voltage_mv != NULL))
        return hwm_source->voltage_mv(hwm_source->priv, idx);

    return hwm_values.voltages[idx];
}

uint8_t
hwm_get_temperature(unsigned idx)
{
    if (idx >= HWM_TEMPERATURES)
        return 0;

    if ((hwm_source != NULL) && (hwm_source->temperature != NULL))
        return hwm_source->temperature(hwm_source->priv, idx);

    return hwm_values.temperatures[idx];
}
