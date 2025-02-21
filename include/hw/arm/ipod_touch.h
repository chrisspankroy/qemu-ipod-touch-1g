#ifndef HW_ARM_IPOD_TOUCH_H
#define HW_ARM_IPOD_TOUCH_H

#include "exec/hwaddr.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"

#define TYPE_IPOD_TOUCH "iPod-Touch"
#define TYPE_IPOD_TOUCH_MACHINE MACHINE_TYPE_NAME(TYPE_IPOD_TOUCH)
#define IPOD_TOUCH_MACHINE(obj) \
    OBJECT_CHECK(IPodTouchMachineState, (obj), TYPE_IPOD_TOUCH_MACHINE)

typedef struct {
    MachineClass parent;
} IPodTouchMachineClass;

typedef struct {
    /* Define memory regions, CPU pointers, peripheral state, etc. */
    MemoryRegion sysmem;
    CPUState *cpu;
} IPodTouchMachineState;

#endif
