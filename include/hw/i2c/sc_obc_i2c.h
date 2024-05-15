/*
 * Space Cubics OBC FPGA's I2C Controller register difinition
 */

#ifndef SC_OBC_I2C_H
#define SC_OBC_I2C_H

#ifndef QEMU_I2C_H
#include "hw/i2c/i2c.h"
#endif
#ifndef HW_SYSBUS_H
#include "hw/sysbus.h"
#endif


#define SCOBC_I2C_FIFO_SIZE 16

typedef struct I2cFifo {
    uint8_t     ringbuf[SCOBC_I2C_FIFO_SIZE];
    uint32_t    rpos;
    uint32_t    wpos;
} I2cFifo;

struct SCObcI2cState {
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
#define TYPE_SCOBC_I2C  "scobc-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(SCObcI2cState, SCOBC_I2C)

#endif /* SC_OBC_I2C_H */
