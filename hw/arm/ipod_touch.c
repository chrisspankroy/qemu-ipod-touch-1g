#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/arm/ipod_touch.h"

static void ipod_touch_machine_init(MachineState *machine)
{
    IPodTouchMachineState *nms = IPOD_TOUCH_MACHINE(machine);

    /* Initialize a memory region for system RAM */
    memory_region_init_ram(&nms->sysmem, NULL, "sysmem", 0x10000000, &error_fatal);

    /* Map the RAM into the address space */
    memory_region_add_subregion(get_system_memory(), 0x00000000, &nms->sysmem);

    /* Initialize and attach the CPU (or multiple CPUs) */
    ARMCPU **cpu;
    Object *cpuobj = object_new(machine->cpu_type);
    *cpu = ARM_CPU(cpuobj);
    CPUState *cs = CPU(*cpu);
    nms->cpu = cpu;

    /* Additional CPU configuration can be performed here */

    /* Add additional peripheral initialization here */
}

static void ipod_touch_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->init = ipod_touch_machine_init;
    mc->desc = "iPod Touch";
}

static void ipod_touch_instance_init(Object *obj)
{
}

static const TypeInfo ipod_touch_machine_info = {
    .name          = TYPE_IPOD_TOUCH_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(IPodTouchMachineState),
    .class_size    = sizeof(IPodTouchMachineClass),
    .class_init    = ipod_touch_machine_class_init,
    .instance_init = ipod_touch_instance_init
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_machine_info);
}

type_init(ipod_touch_machine_types)
