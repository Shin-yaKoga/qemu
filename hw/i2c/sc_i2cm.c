#include "qemu/osdep.h"
#include "hw/i2c/sc_i2cm.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "migration/vmstate.h"


#define SC_I2CM_FIFO_SIZE 16

typedef struct I2cFifo {
    uint8_t     ringbuf[SC_I2CM_FIFO_SIZE];
    uint32_t    rpos;
    uint32_t    wpos;
} I2cFifo;

struct SC_I2CM_State {
    SysBusDevice    parent_obj;

    I2CBus* bus;
    qemu_irq    irq;
    MemoryRegion    iomem;
    uint32_t    ier_shadow_mask;
    I2cFifo     tx_fifo;  /* actually, never used */
    I2cFifo     rx_fifo;
    uint32_t    i2cm_status;
    uint8_t     curr_slave_addr;
    uint32_t    requested_bytes;
    uint32_t    received_bytes;
    bool        stop_after_recv;
    uint32_t    enr;
    uint32_t    txfifor;
    uint32_t    bsr;
    uint32_t    isr;
    uint32_t    ier;
    uint32_t    fifosr;
    uint32_t    fiforr;
    uint32_t    ftlsr;
    uint32_t    scltsr;
    uint32_t    thdstar;
    uint32_t    tsustor;
    uint32_t    tsustar;
    uint32_t    thighr;
    uint32_t    thddatr;
    uint32_t    tsudatr;
    uint32_t    tbufr;
    uint32_t    tbsmplr;
    uint32_t    bfsr;
};
OBJECT_DECLARE_SIMPLE_TYPE(SC_I2CM_State, SC_I2CM)


static void i2c_fifo_init(I2cFifo* fifo)
{
    memset(fifo->ringbuf, 0, sizeof(fifo->ringbuf));
    fifo->rpos = fifo->wpos = 0;
}

static uint32_t i2c_fifo_count(I2cFifo* fifo)
{
    return (fifo->wpos - fifo->rpos);
}

static bool i2c_fifo_is_full(I2cFifo* fifo)
{
    return (i2c_fifo_count(fifo) >= SC_I2CM_FIFO_SIZE);
}

static uint8_t  i2c_fifo_read(I2cFifo* fifo)
{
    uint8_t val = fifo->ringbuf[fifo->rpos % SC_I2CM_FIFO_SIZE];

    if (i2c_fifo_count(fifo) > 0) {
        fifo->rpos += 1;
    }

    return val;
}

static bool i2c_fifo_write(I2cFifo* fifo, uint8_t val)
{
    if (i2c_fifo_count(fifo) == SC_I2CM_FIFO_SIZE) {
        return false;  /* buffer is full */
    } else {
        fifo->ringbuf[fifo->wpos++ % SC_I2CM_FIFO_SIZE] = val;
        return true;
    }
}

/* I2CM_ISR, I2CM_IER register field's bits */
#define SC_I2CM_INTR_SCLTO       12
#define SC_I2CM_INTR_RXFIFOUDF   11
#define SC_I2CM_INTR_TXFIFOOVF   10
#define SC_I2CM_INTR_RXFIFOOTH    5
#define SC_I2CM_INTR_TXFIFOUTH    4
#define SC_I2CM_INTR_COMP         0

/* I2CM_BSR register field's bits */
#define SC_I2CM_BSR_OTHERBUSY    1
#define SC_I2CM_BSR_SELFBUSY     0

/* I2CM_TXFIFOR register field's bits */
#define SC_I2CM_RESTART  9
#define SC_I2CM_STOP     8

/* ScObcI2cState.i2cm_status */
typedef enum {
    I2CM_STAT_IDLE,
    I2CM_STAT_IN_SENDING,
    I2CM_STAT_RECV_STARTING,
    I2CM_STAT_IN_RECEIVING,
} EI2cmStatus;

static void sc_i2cm_do_try_irq_raise(SC_I2CM_State* s)
{
    if (((s->isr & s->ier) & s->ier_shadow_mask) != 0) {
        qemu_irq_raise(s->irq);
    }
}

static uint8_t  sc_i2cm_do_read_and_recv(SC_I2CM_State* s)
{
    uint8_t val = i2c_fifo_read(&s->rx_fifo);

    if (s->received_bytes < s->requested_bytes) {
        i2c_fifo_write(&s->rx_fifo, i2c_recv(s->bus));
        s->received_bytes += 1;
        if (s->received_bytes == s->requested_bytes) {
            i2c_end_transfer(s->bus);
            s->i2cm_status = I2CM_STAT_IDLE;
            if (s->stop_after_recv) {
                s->isr |= (1UL << SC_I2CM_INTR_COMP);
                sc_i2cm_do_try_irq_raise(s);
            }
        }
    }

    return val;
}

static void sc_i2cm_do_write_and_send(SC_I2CM_State* s, uint32_t value)
{
    switch (s->i2cm_status) {
    case I2CM_STAT_IDLE:
        s->curr_slave_addr = (uint8_t)(value >> 1);
        if (0 != i2c_start_transfer(s->bus, s->curr_slave_addr, (0 != (value & 0x1)))) {
            // エラー
            // xxx
            return;
        } else {
            if (0 != (value & 0x1)) {
                s->i2cm_status = I2CM_STAT_RECV_STARTING;
                s->requested_bytes = s->received_bytes = 0;
            } else {
                s->i2cm_status = I2CM_STAT_IN_SENDING;
            }
        }
        break;
    case I2CM_STAT_IN_SENDING:
        // 1Byte 送出
        if (0 != i2c_send(s->bus, (uint8_t)value)) {
            // エラー
            // xxx
            return;
        }
        if (value & (1UL << SC_I2CM_STOP)) {
            i2c_end_transfer(s->bus);
            s->i2cm_status = I2CM_STAT_IDLE;
            s->isr |= (1UL << SC_I2CM_INTR_COMP);
        } else if (value & (1UL << SC_I2CM_RESTART)) {
            i2c_end_transfer(s->bus);
            s->i2cm_status = I2CM_STAT_IDLE;
        }
        break;
    case I2CM_STAT_RECV_STARTING:
        s->requested_bytes = (uint8_t)value + 1;
        s->received_bytes = 0;
        i2c_fifo_init(&s->rx_fifo);
        s->stop_after_recv = (0 != (value & (1UL << SC_I2CM_STOP)));
        s->i2cm_status = I2CM_STAT_IN_RECEIVING;
        for (uint32_t i = 0; i < SC_I2CM_FIFO_SIZE; ++i) {
            (void)i2c_fifo_write(&s->rx_fifo, i2c_recv(s->bus));
            if (++(s->received_bytes) == s->requested_bytes) {
                i2c_end_transfer(s->bus);
                s->i2cm_status = I2CM_STAT_IDLE;
                if (s->stop_after_recv) {
                    s->isr |= (1UL << SC_I2CM_INTR_COMP);
                    sc_i2cm_do_try_irq_raise(s);
                }
                break;
            }
        }
        break;
    case I2CM_STAT_IN_RECEIVING:
        // 実機についての見解:
        // ---
        // そのデータはFIFOに書き込まれるため、次の転送のアドレスと思って、
        // 受信動作が完了した後にすぐにデータを出してしまうと思います。
        // (少し自信がないですが、多分正しいです)
        // ---
        /* simply ignore in emulation */
        break;
    default:
        // 予期しない値（エラー）
        // xxx
        break;
    }
}

static void sc_i2cm_update(SC_I2CM_State* s)
{
    int level = ((s->isr & s->ier) & s->ier_shadow_mask) != 0;

    qemu_set_irq(s->irq, level);
}

static uint64_t sc_i2cm_read(void* opaque, hwaddr offset, unsigned size)
{
    SC_I2CM_State* s = (SC_I2CM_State*)opaque;

    switch (offset) {
    case 0x0000: /* I2CM_ENR */
        return s->enr;
    case 0x0004: /* I2CM_TXFIFOR */
        return 0;  /* WO */
    case 0x0008: /* I2CM_RXFIFOR */
        if (0 == i2c_fifo_count(&s->rx_fifo)) {
            s->isr |= (1UL << SC_I2CM_INTR_RXFIFOUDF);
            sc_i2cm_do_try_irq_raise(s);
            return 0;  /* as Undefined value */
        } else {
            return sc_i2cm_do_read_and_recv(s);
        }
    case 0x000C: /* I2CM_BSR */
        s->bsr = 0;
        if (s->i2cm_status != I2CM_STAT_IDLE) {
            s->bsr |= (1UL << SC_I2CM_BSR_SELFBUSY);
        }
        if (i2c_bus_busy(s->bus)) {
            s->bsr |= (1UL << SC_I2CM_BSR_OTHERBUSY);
        }
        return s->bsr;
    case 0x0010: /* I2CM_ISR */
        return s->isr;
    case 0x0014: /* I2CM_IER */
        return s->ier;
    case 0x0018: /* I2CM_FIFOSR */
        /* read RXFIFO count & TXFIFO count */
        return (
            (i2c_fifo_count(&s->rx_fifo) << 16) |
            (i2c_fifo_count(&s->tx_fifo) << 0));
    case 0x001C: /* I2CM_FIFORR */
        return 0;  /* WO */
    case 0x0020: /* I2CM_FTLSR */
        return s->ftlsr;
    case 0x0024: /* I2CM_SCLTSR */
        return s->scltsr;
    case 0x0030: /* I2CM_THDSTAR */
        return s->thdstar;
    case 0x0034: /* I2CM_TSUSTOR */
        return s->tsustor;
    case 0x0038: /* I2CM_TSUSTAR */
        return s->tsustar;
    case 0x003C: /* I2CM_THIGHR */
        return s->thighr;
    case 0x0040: /* I2CM_THDDATR */
        return s->thddatr;
    case 0x0044: /* I2CM_TSUDATR */
        return s->tsudatr;
    case 0x0048: /* I2CM_TBUFR */
        return s->tbufr;
    case 0x004C: /* I2CM_TBSMPLR */
        return s->tbsmplr;
    case 0x0050: /* I2CM_BFSR */
        return s->bfsr;
    case 0xF000: /* I2CM_VER */
        return 0x01000000;  /* 1.0-p0 */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
            "sc_i2cm_read: read at bad offset 0x%lu\n", offset);
        return 0;
    }
}

static void sc_i2cm_write(void* opaque, hwaddr offset, uint64_t value, unsigned size)
{
    SC_I2CM_State* s = (SC_I2CM_State*)opaque;

    switch (offset) {
    case 0x0000: /* I2CM_ENR */
        s->enr = value & 0x1;
        break;
    case 0x0004: /* I2CM_TXFIFOR */
        if (i2c_fifo_is_full(&s->tx_fifo)) {
            s->isr |= (1UL << SC_I2CM_INTR_TXFIFOOVF);
            sc_i2cm_do_try_irq_raise(s);
        } else {
            sc_i2cm_do_write_and_send(s, (uint32_t)value);
        }
        break;
    case 0x0008: /* I2CM_RXFIFOR */
    case 0x000C: /* I2CM_BSR */
        return; /* RO */
    case 0x0010: /* I2CM_ISR */
        s->isr &= ~value;
        break;
    case 0x0014: /* I2CM_IER */
        s->ier = value & 0x00001F33;
        break;
    case 0x0018: /* I2CM_FIFOSR */
        return; /* RO */
    case 0x001C: /* I2CM_FIFORR */
        if (value & 0x10000) {
            /* clear RXFIFO */
            i2c_fifo_init(&s->rx_fifo);
            s->isr &= ~(
                (1UL << SC_I2CM_INTR_RXFIFOUDF) |
                (1UL << SC_I2CM_INTR_RXFIFOOTH));
        }
        if (value & 0x1) {
            /* clear TXFIFO */
            i2c_fifo_init(&s->tx_fifo);
            s->isr &= ~(
                (1UL << SC_I2CM_INTR_TXFIFOOVF) |
                (1UL << SC_I2CM_INTR_TXFIFOUTH));
        }
        break;
    case 0x0020: /* I2CM_FTLSR */
        s->ftlsr = (value & 0x001F001F);
        if (((s->ftlsr & 0x001F0000) == 0) ||
            ((s->ftlsr & 0x001F0000) == 0x001F0000)) {
            /* disable I2CM_RXFIFOOTH */
            s->ier_shadow_mask &= ~(1UL << SC_I2CM_INTR_RXFIFOOTH);
        }
        if (((s->ftlsr & 0x0000001F) == 0) ||
            ((s->ftlsr & 0x0000001F) == 0x0000001F)) {
            /* disable I2CM_TXFIFOUTH */
            s->ier_shadow_mask &= ~(1UL << SC_I2CM_INTR_TXFIFOUTH);
        }
        break;
    case 0x0024: /* I2CM_SCLTSR */
        s->scltsr = value & 0x0000FFFF;
        if (s->scltsr == 0) {
            /* disable I2CM_SCLTO */
            s->ier_shadow_mask &= ~(1UL << SC_I2CM_INTR_SCLTO);
        }
        break;
    case 0x0030: /* I2CM_THDSTAR */
        if (s->enr != 0) {
            return;
        }
        s->thdstar = (uint16_t)value;
        break;
    case 0x0034: /* I2CM_TSUSTOR */
        if (s->enr != 0) {
            return;
        }
        s->tsustor = (uint16_t)value;;
        break;
    case 0x0038: /* I2CM_TSUSTAR */
        if (s->enr != 0) {
            return;
        }
        s->tsustar = (uint16_t)value;;
        break;
    case 0x003C: /* I2CM_THIGHR */
        if (s->enr != 0) {
            return;
        }
        s->thighr = (uint16_t)value;;
        break;
    case 0x0040: /* I2CM_THDDATR */
        if (s->enr != 0) {
            return;
        }
        s->thddatr = (uint16_t)value;
        break;
    case 0x0044: /* I2CM_TSUDATR */
        if (s->enr != 0) {
            return;
        }
        s->tsudatr = (uint16_t)value;
        break;
    case 0x0048: /* I2CM_TBUFR */
        if (s->enr != 0) {
            return;
        }
        s->tbufr = (uint16_t)value;
        break;
    case 0x004C: /* I2CM_TBSMPLR */
        s->tbsmplr = (uint16_t)value;
        break;
    case 0x0050: /* I2CM_BFSR */
        s->bfsr = (uint8_t)value;
        break;
    case 0xF000: /* I2CM_VER */
        return;  /* RO */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
            "sc_i2cm_write: write at bad offset 0x%lu\n", offset);
        return;
    }
    sc_i2cm_update(s);
}

static const MemoryRegionOps    sc_i2cm_ops = {
    .read = sc_i2cm_read,
    .write = sc_i2cm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_scobc_i2c = {
    .name = "scobc_i2c",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(enr,       SC_I2CM_State),
        VMSTATE_UINT32(txfifor,   SC_I2CM_State),
        VMSTATE_UINT32(bsr,       SC_I2CM_State),
        VMSTATE_UINT32(isr,       SC_I2CM_State),
        VMSTATE_UINT32(ier,       SC_I2CM_State),
        VMSTATE_UINT32(fifosr,    SC_I2CM_State),
        VMSTATE_UINT32(fiforr,    SC_I2CM_State),
        VMSTATE_UINT32(ftlsr,     SC_I2CM_State),
        VMSTATE_UINT32(scltsr,    SC_I2CM_State),
        VMSTATE_UINT32_V(thdstar, SC_I2CM_State, 0x000000EF),
        VMSTATE_UINT32_V(tsustor, SC_I2CM_State, 0x000000EF),
        VMSTATE_UINT32_V(tsustar, SC_I2CM_State, 0x00000117),
        VMSTATE_UINT32_V(thighr,  SC_I2CM_State, 0x000000E5),
        VMSTATE_UINT32_V(thddatr, SC_I2CM_State, 0x00000013),
        VMSTATE_UINT32_V(tsudatr, SC_I2CM_State, 0x000000E5),
        VMSTATE_UINT32_V(tbufr,   SC_I2CM_State, 0x00000117),
        VMSTATE_UINT32(tbsmplr,   SC_I2CM_State),
        VMSTATE_UINT32_V(bfsr,    SC_I2CM_State, 0x0000002F),
        VMSTATE_END_OF_LIST()
    }
};

static void sc_i2cm_init(Object* obj)
{
    DeviceState* dev = DEVICE(obj);
    SC_I2CM_State* s = SC_I2CM(obj);
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);
    I2CBus* bus;

    sysbus_init_irq(sbd, &s->irq);
    bus = i2c_init_bus(dev, "i2c");
    s->bus = bus;
    memory_region_init_io(
        &s->iomem, obj, &sc_i2cm_ops, s, "i2c", 0x10000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void sc_i2cm_reset_enter(Object* obj, ResetType type)
{
    SC_I2CM_State* s = SC_I2CM(obj);

    if (s->i2cm_status != I2CM_STAT_IDLE) {
        i2c_end_transfer(s->bus);
        s->i2cm_status = I2CM_STAT_IDLE;
    }
}

static void sc_i2cm_reset_hold(Object* obj)
{
    SC_I2CM_State* s = SC_I2CM(obj);

    s->ier_shadow_mask = 0xFFFFFFFF;
    i2c_fifo_init(&s->tx_fifo);
    i2c_fifo_init(&s->rx_fifo);
    s->i2cm_status = I2CM_STAT_IDLE;
    s->requested_bytes = s->received_bytes = 0;
    s->enr = 0;
    s->txfifor = 0;
    s->bsr = 0;
    s->isr = 0;
    s->ier = 0;
    s->fifosr = 0;
    s->fiforr = 0;
    s->ftlsr = 0;
    s->scltsr = 0;
    s->thdstar = 0x000000EF;
    s->tsustor = 0x000000EF;
    s->tsustar = 0x00000117;
    s->thighr = 0x000000E5;
    s->thddatr = 0x00000013;
    s->tsudatr = 0x000000E5;
    s->tbufr = 0x00000117;
    s->tbsmplr = 0;
    s->bfsr = 0x0000002F;
}

static void sc_i2cm_reset_exit(Object* obj)
{
    SC_I2CM_State* s = SC_I2CM(obj);

    sc_i2cm_update(s);
}

static void sc_i2cm_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);
    ResettableClass* rc = RESETTABLE_CLASS(klass);

    rc->phases.enter = sc_i2cm_reset_enter;
    rc->phases.hold  = sc_i2cm_reset_hold;
    rc->phases.exit  = sc_i2cm_reset_exit;
    dc->vmsd = &vmstate_scobc_i2c;
}

static const TypeInfo   sc_i2cm_info = {
    .name = TYPE_SC_I2CM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SC_I2CM_State),
    .instance_init = sc_i2cm_init,
    .class_init = sc_i2cm_class_init,
};


static void sc_i2cm_register_types(void)
{
    type_register_static(&sc_i2cm_info);
}
type_init(sc_i2cm_register_types)

/*
 * End of File
 */
