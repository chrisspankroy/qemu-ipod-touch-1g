#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/arm/ipod_touch.h"

static void ipod_touch_instance_init(Object *obj)
{

}

// Initialize the iPod Touch machine
static void ipod_touch_machine_init(MachineState *machine)
{
	IPodTouchMachineState *nms = IPOD_TOUCH_MACHINE(machine); // Instantiate our state

	// initialize our cpu and update our state to use it
	ARMCPU *cpu;
	Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("arm1176"));
	cpu = ARM_CPU(cpuobj);
	nms->cpu = cpu;
}

static void ipod_touch_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "iPod Touch";
    mc->init = ipod_touch_machine_init;
}

static const TypeInfo ipod_touch_machine_info = {
    .name          = TYPE_IPOD_TOUCH_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(IPodTouchMachineState),
    .class_size    = sizeof(IPodTouchMachineClass),
    .class_init    = ipod_touch_machine_class_init,
    .instance_init = ipod_touch_instance_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_machine_info);
}

type_init(ipod_touch_machine_types)
