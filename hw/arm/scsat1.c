#include "qemu/osdep.h"
#include "qemu/units.h"
#include "exec/address-spaces.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/clock.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"


struct SCSat1State {
	SysBusDevice	parent_obj;

	Clock*	sysclk;
};
#define TYPE_SCSAT1_SOC	"scsat1-soc"
OBJECT_DECLARE_SIMPLE_TYPE(SCSat1State, SCSAT1_SOC)

static void
scsat1_soc_reset_enter(Object* obj, ResetType type)
{
	// do nothing
}

static void scsat1_soc_reset_hold(Object* obj)
{
	SCSat1State*	soc = SCSAT1_SOC(obj);

	clock_set_ns(soc->sysclk, 1000 / 48);  // 48MHz
	clock_propagate(soc->sysclk);
}

static void
scsat1_soc_reset_exit(Object* obj)
{
	// do nothing
}

static void
scsat1_soc_class_init(ObjectClass* klass, void* data)
{
	ResettableClass* rc = RESETTABLE_CLASS(klass);

	rc->phases.enter = scsat1_soc_reset_enter;
	rc->phases.hold  = scsat1_soc_reset_hold;
	rc->phases.exit  = scsat1_soc_reset_exit;
}

static void
scsat1_soc_instance_init(Object* obj)
{
	SCSat1State* soc = SCSAT1_SOC(obj);

	soc->sysclk = qdev_init_clock_out(DEVICE(soc), "SYSCLK");
}

static const TypeInfo scsat1_soc_info = {
	.name = TYPE_SCSAT1_SOC,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(SCSat1State),
	.instance_init = scsat1_soc_instance_init,
	.class_init = scsat1_soc_class_init,
};

static void scsat1_register_types(void)
{
	type_register_static(&scsat1_soc_info);
}

type_init(scsat1_register_types)


static void
scsat1_init(MachineState* ms)
{
	MemoryRegion* sram = g_new(MemoryRegion, 1);
	MemoryRegion* system_memory = get_system_memory();
	DeviceState*	soc;
	DeviceState*	cpu;
	DeviceState*	uart;

	memory_region_init_ram(sram, NULL,
		"scsat1.sram", 32 * KiB, &error_fatal);  // on chip RAM
	memory_region_add_subregion(system_memory, 0x00000000, sram);

	soc = qdev_new(TYPE_SCSAT1_SOC);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(soc), &error_fatal);

	cpu = qdev_new(TYPE_ARMV7M);
	qdev_prop_set_string(cpu, "cpu-type", ms->cpu_type);
	qdev_connect_clock_in(
		cpu, "cpuclk", qdev_get_clock_out(soc, "SYSCLK"));
	object_property_set_link(
		OBJECT(cpu), "memory", OBJECT(get_system_memory()), &error_abort);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(cpu), &error_fatal);

	uart = qdev_new("xlnx.xps-uartlite");
	qdev_prop_set_chr(uart, "chardev", serial_hd(0));
	sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
	sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, 0x4f010000);

	armv7m_load_kernel(ARM_CPU(first_cpu), ms->kernel_filename, 0, 32 * MiB);
}

static void
scsat1_main_init(MachineState* machine)
{
	scsat1_init(machine);
}

static void
scsat1_main_machine_init(MachineClass* mc)
{
	mc->desc = "Space Cubics Sat1-Main (Cortex-M3)";
	mc->init = &scsat1_main_init;
	mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
	mc->default_ram_id   = "ram";
	mc->default_ram_size = 32 * KiB;
	mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("scsat1-main", scsat1_main_machine_init)
