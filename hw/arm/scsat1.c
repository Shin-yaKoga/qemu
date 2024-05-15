#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "exec/address-spaces.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/clock.h"
#include "hw/i2c/sc_obc_i2c.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/sensor/tmp105.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"

#include <stdlib.h>


#define NUM_IRQ_LINES   64
#define NUM_PRIO_BITS   3


struct SCSat1State {
    SysBusDevice    parent_obj;

    Clock*    sysclk;
};
#define TYPE_SCSAT1_SOC    "scsat1-soc"
OBJECT_DECLARE_SIMPLE_TYPE(SCSat1State, SCSAT1_SOC)

static void scsat1_soc_reset_enter(Object* obj, ResetType type)
{
    /* do nothing */
}

static void scsat1_soc_reset_hold(Object* obj)
{
    SCSat1State*    soc = SCSAT1_SOC(obj);

    clock_set_ns(soc->sysclk, 1000 / 48);  /* 48MHz */
    clock_propagate(soc->sysclk);
}

static void scsat1_soc_reset_exit(Object* obj)
{
    /* do nothing */
}

static void scsat1_soc_class_init(ObjectClass* klass, void* data)
{
    ResettableClass* rc = RESETTABLE_CLASS(klass);

    rc->phases.enter = scsat1_soc_reset_enter;
    rc->phases.hold  = scsat1_soc_reset_hold;
    rc->phases.exit  = scsat1_soc_reset_exit;
}

static void scsat1_soc_instance_init(Object* obj)
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


static void scsat1_init(MachineState* ms)
{
    MemoryRegion* sram = g_new(MemoryRegion, 1);
    MemoryRegion* system_memory = get_system_memory();
    Object*       soc_container;
    DeviceState*  soc;
    DeviceState*  cpu;
    DeviceState*  uart;
    DeviceState*  i2c;
    DeviceState*  i2c_slave;
    I2CBus* i2c_bus;

    memory_region_init_ram(sram, NULL,
        "scsat1.sram", 32 * KiB, &error_fatal);  /* on chip RAM */
    memory_region_add_subregion(system_memory, 0x00000000, sram);

    soc_container = object_new("container");
    object_property_add_child(OBJECT(ms), "soc_container", soc_container);

    soc = qdev_new(TYPE_SCSAT1_SOC);
    object_property_add_child(soc_container, "soc", OBJECT(soc));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(soc), &error_fatal);

    cpu = qdev_new(TYPE_ARMV7M);
    object_property_add_child(soc_container, "v7m", OBJECT(cpu));
    qdev_prop_set_uint32(cpu, "num-irq", NUM_IRQ_LINES);
/*    qdev_prop_set_uint8(cpu, "num-prio-bits", NUM_PRIO_BITS); */ /* enabled after v9.x */
    qdev_prop_set_string(cpu, "cpu-type", ms->cpu_type);
//    qdev_prop_set_bit(cpu, "enable-bitband", true);
    object_property_set_link(
        OBJECT(cpu), "memory", OBJECT(system_memory), &error_abort);
    qdev_connect_clock_in(
        cpu, "cpuclk", qdev_get_clock_out(soc, "SYSCLK"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cpu), &error_fatal);

    uart = qdev_new("xlnx.xps-uartlite");
    //    object_property_add_child(soc_container, "uart", OBJECT(uart));
    object_property_add_child(soc_container, "uart[*]", OBJECT(uart));
    qdev_prop_set_chr(uart, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, 0x4f010000);
    sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0, qdev_get_gpio_in(cpu, 0));

    i2c = sysbus_create_simple(TYPE_SCOBC_I2C, 0x4F030000, qdev_get_gpio_in(cpu, 7));

    /* create and attach the I2C temperature sensor */
    i2c_bus = (I2CBus*)qdev_get_child_bus(i2c, "i2c");
    i2c_slave = DEVICE(i2c_slave_create_simple(i2c_bus, TYPE_TMP105, 0x48));
    object_property_set_int(OBJECT(i2c_slave), "temperature", 25000, &error_abort);  /* 25Ž */
//    object_property_set_int(OBJECT(i2c_slave), "temperature", 20000, &error_abort);  /* 25Ž */

    armv7m_load_kernel(ARM_CPU(first_cpu), ms->kernel_filename, 0, 32 * MiB);
}

static void scsat1_main_init(MachineState* machine)
{
    scsat1_init(machine);
}

static void scsat1_main_machine_init(MachineClass* mc)
{
    mc->desc = "Space Cubics Sat1-Main (Cortex-M3)";
//    mc->init = &scsat1_main_init;
    mc->init = scsat1_main_init;  // !?
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
    mc->default_ram_id   = "ram";
    mc->default_ram_size = 32 * KiB;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("scsat1-main", scsat1_main_machine_init)
