#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "sysemu/reset.h"
#include "hw/arm/xnu.h"
#include "hw/arm/xnu_mem.h"
#include "hw/arm/ipod_touch.h"

static void ipod_touch_instance_init(Object *obj)
{

}

static void reset_cpu(void *opaque)
{
    IPodTouchMachineState *nms = IPOD_TOUCH_MACHINE((MachineState *)opaque);
    ARMCPU *cpu = nms->cpu;
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);

    cpu_set_pc(CPU(cpu), 0xe59ff060);
    printf("CPU reset complete. PC set to 0xe59ff060\n");
}

// Initialize the iPod Touch machine
static void ipod_touch_machine_init(MachineState *machine)
{
	IPodTouchMachineState *nms = IPOD_TOUCH_MACHINE(machine); // Instantiate our state

	// initialize cpu
	ARMCPU *cpu;
	Object *cpuobj = object_new(ARM_CPU_TYPE_NAME("arm1176"));
	cpu = ARM_CPU(cpuobj);
	AddressSpace* nsas;
	CPUState *cs = CPU(cpu);

        // initialize memory
        MemoryRegion *sysmem = get_system_memory();
        object_property_set_link(cpuobj, "memory", OBJECT(sysmem), &error_abort); // link our memory to CPU

	object_property_set_bool(cpuobj, "realized", true, &error_fatal); // mark CPU as ready

	nms->cpu = cpu;
	nsas = cpu_get_address_space(cs, ARMASIdx_NS);

	// map our file into memory

	arm_load_securerom("/root/Documents/s5l8900_securerom_rev2", nsas, sysmem, "securerom", 0x0);

	// register callback for CPU reset
	qemu_register_reset(reset_cpu, nms);
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
    .instance_init = ipod_touch_instance_init
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_machine_info);
}

type_init(ipod_touch_machine_types)
