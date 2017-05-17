/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization and API Support.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <rtems/irq-extension.h>
#include <rtems/counter.h>
#include <bsp/bbb-gpio.h>
#include <rtems/score/assert.h>


static void flush_fifo(i2c_bus *base);

static void am335x_i2c0_pinmux(bbb_i2c_bus *bus)
{
  REG(bus->regs + AM335X_CONF_I2C0_SDA) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN);

  REG(bus->regs + AM335X_CONF_I2C0_SCL) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN); 
}

static void I2C0ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_I2C0_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
          CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_I2C0_GFCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_I2C0_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_I2C0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_IDLEST));
}

/*
static bool am335x_i2c_busbusy(volatile bbb_i2c_regs *regs)
{
  bool status;
printf("begin am335x_i2c_busbusy\n");
  if (REG(&regs->BBB_I2C_IRQSTATUS_RAW) & AM335X_I2C_IRQSTATUS_RAW_BB)
  {
    status = true; 
  } else {
    status = false;
  }
  return status; 
}
*/

static bool am335x_i2c_busbusy(volatile bbb_i2c_regs *regs)
{
  bool status;
  int stat;
  int timeout=I2C_TIMEOUT;

printf("begin am335x_i2c_busbusy\n");
   while ((stat = readw(&regs->BBB_I2C_IRQSTATUS_RAW) &
    AM335X_I2C_IRQSTATUS_RAW_BB) && timeout--) {
   printf("am335x_i2c_busbusy test1\n");
    writew(stat, &regs->BBB_I2C_IRQSTATUS);
    udelay(10);
  }
printf("am335x_i2c_busbusy test2\n");
  if (timeout <= 0) {
    printf("Timed out in wait_for_bb: status=%04x\n",
           stat);
   status = true;
  }
  else
  {
  writew(0xFFFF, &regs->BBB_I2C_IRQSTATUS);   
   status = false;
  }
printf("end am335x_i2c_busbusy\n");
  return status; 
}




static void am335x_i2c_reset(bbb_i2c_bus *bus)
{
  volatile bbb_i2c_regs *regs = bus->regs;
   int timeout = I2C_TIMEOUT; 
 
   if (readw(&regs->BBB_I2C_CON) & I2C_CON_EN) {
    writew(0, &regs->BBB_I2C_CON);
    udelay(50000);
  }

  writew(0x2, &regs->BBB_I2C_SYSC); /* for ES2 after soft reset */
  udelay(1000);

  writew(I2C_CON_EN, &regs->BBB_I2C_CON);
  while (!(readw(&regs->BBB_I2C_SYSS) & I2C_SYSS_RDONE) && timeout--) {
    if (timeout <= 0) {
      puts("ERROR: Timeout in soft-reset\n");
      return;
    }
    udelay(1000);
  }


}

/*
Possible values for msg->flag 
   * - @ref I2C_M_TEN,
   * - @ref I2C_M_RD,
   * - @ref I2C_M_STOP,
   * - @ref I2C_M_NOSTART,
   * - @ref I2C_M_REV_DIR_ADDR,
   * - @ref I2C_M_IGNORE_NAK,
   * - @ref I2C_M_NO_RD_ACK, and
   * - @ref I2C_M_RECV_LEN.
*/

static void am335x_i2c_set_address_size(const i2c_msg *msgs,volatile bbb_i2c_regs *regs)
{
    /*can be configured multiple modes here. Need to think about own address modes*/
  if ((msgs->flags & I2C_M_TEN) == 0)  {/* 7-bit mode slave address mode*/
  mmio_write(&regs->BBB_I2C_CON,(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN)); 
  } else { /* 10-bit slave address mode*/
  mmio_write(&regs->BBB_I2C_CON,(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  }
  }

static void am335x_i2c_next_byte(bbb_i2c_bus *bus)
{
  i2c_msg *msg;

  ++bus->msgs;
  --bus->msg_todo;

  msg = &bus->msgs[0];

  bus->current_msg_todo = msg->len;
  bus->current_msg_byte = msg->buf;
}

static unsigned int am335x_i2c_intrawstatus(volatile bbb_i2c_regs *regs)
{
  return (REG(&regs->BBB_I2C_IRQSTATUS_RAW));
}

static void am335x_i2c_masterint_enable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  REG(&regs->BBB_I2C_IRQENABLE_SET) |= flag;
}

static void am335x_i2c_masterint_disable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
 REG(&regs->BBB_I2C_IRQENABLE_CLR) = flag;
}

static void am335x_int_clear(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  REG(&regs->BBB_I2C_IRQSTATUS) = flag;
}


static void am335x_clean_interrupts(volatile bbb_i2c_regs *regs)
{
  am335x_i2c_masterint_enable(regs,0x7FFF);
  am335x_int_clear(regs,0x7FFF);
  am335x_i2c_masterint_disable(regs,0x7FFF); 
}


static void am335x_i2c_setup_read_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs, const i2c_msg *msgs, bool send_stop)
{ 
  volatile unsigned int no_bytes;
  int status;
    REG(&regs->BBB_I2C_CNT) = bus->current_msg_todo;
//  printf("am335x_i2c_setup_read_transfer\n");
  // I2C Controller in Master Mode
  REG(&regs->BBB_I2C_CON) = AM335X_I2C_CFG_MST_RX | AM335X_I2C_CON_I2C_EN | AM335X_I2C_CON_MST;
    am335x_clean_interrupts(regs);
  // receive interrupt is enabled
  am335x_i2c_masterint_enable(regs, AM335X_I2C_INT_RECV_READY | AM335X_I2C_INT_STOP_CONDITION);

// printk("middle am335x_i2c_setup_read_transfer\n");
//    printk("irqstatus:%x\n",REG(&regs->BBB_I2C_IRQSTATUS) );
  if (send_stop) {
    // stop condition
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP; 
  } else {
    // start condition
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_START;
  }
 // printf("end am335x_i2c_setup_read_transfer\n");
 //  printk("irqstatus:%x\n",REG(&regs->BBB_I2C_IRQSTATUS) );
  //printf("end emd am335x_i2c_setup_read_transfer\n");
 // printk("irqstatus:%x\n",REG(&regs->BBB_I2C_IRQSTATUS) );
  //while(am335x_i2c_busbusy(regs));
 
}


static void am335x_i2c_continue_read_transfer(
  bbb_i2c_bus *bus,
  volatile bbb_i2c_regs *regs
)
{
  bus->current_msg_byte[bus->already_transferred] = REG(&regs->BBB_I2C_DATA);
//  printf("begin am335x_i2c_continue_read_transfer irqstatus:%x\n", REG(&regs->BBB_I2C_IRQSTATUS) );
  bus->already_transferred++;
  am335x_int_clear(regs,AM335X_I2C_INT_RECV_READY);

 //  printf("middle am335x_i2c_continue_read_transfer irqstatus:%x\n", REG(&regs->BBB_I2C_IRQSTATUS) );
  
  if (bus->already_transferred == REG(&regs->BBB_I2C_CNT)) {
    

    am335x_i2c_masterint_disable(regs, AM335X_I2C_INT_RECV_READY);
    
    REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP;
    
  }
}


static void am335x_i2c_continue_write(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{ 

 // printf("am335x_i2c_continue_write\n");
//  writeb(0x0,&regs->BBB_I2C_DATA);
//writew(AM335X_I2C_IRQSTATUS_XRDY, &regs->BBB_I2C_IRQSTATUS);

//  printf("bus->already_transferred:%d\n",bus->already_transferred );
//  printf("bus->msg_todo:%d\n",bus->msg_todo );
if (bus->already_transferred == bus->msg_todo) {
 //    printk("finished transfer \n");
     writeb(bus->current_msg_byte[bus->already_transferred],&regs->BBB_I2C_DATA);
     writew(AM335X_I2C_IRQSTATUS_XRDY, &regs->BBB_I2C_IRQSTATUS);
     am335x_i2c_masterint_disable(regs, AM335X_I2C_IRQSTATUS_XRDY);
     REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_STOP;
   } else { 
 //    printk("remaining byte1111 \n");
 //    printf("bus->current_msg_byte[bus->already_transferred]:%x\n",bus->current_msg_byte[bus->already_transferred] );
   writeb(bus->current_msg_byte[bus->already_transferred],&regs->BBB_I2C_DATA);
   // REG(&regs->BBB_I2C_DATA) = bus->current_msg_byte[bus->already_transferred];
   //  printk("%x\n",REG(&regs->BBB_I2C_DATA));
  //  writew(AM335X_I2C_IRQSTATUS_XRDY, &regs->BBB_I2C_IRQSTATUS);
//   am335x_int_clear(regs,AM335X_I2C_IRQSTATUS_XRDY);
     bus->already_transferred++;   
   }

}

static void am335x_i2c_setup_write_transfer(bbb_i2c_bus *bus,volatile bbb_i2c_regs *regs, const i2c_msg *msgs)
{
  volatile unsigned int no_bytes; 
// printf(" begin am335x_i2c_setup_write_transfer\n");

  // Following data count specify bytes to be transmitted
  REG(&regs->BBB_I2C_CNT) = bus->current_msg_todo;
  no_bytes = REG(&regs->BBB_I2C_CNT);
  REG(&regs->BBB_I2C_SA) = msgs->addr;

  // I2C Controller in Master transmitter Mode
  REG(&regs->BBB_I2C_CON) = AM335X_I2C_CFG_MST_TX | AM335X_I2C_CON_I2C_EN;

  am335x_clean_interrupts(regs);
  
  // transmit interrupt is enabled
  am335x_i2c_masterint_enable(regs,AM335X_I2C_IRQSTATUS_XRDY );
  
 // printf(" middle am335x_i2c_setup_write_transfer\n");

  //start condition 
  REG(&regs->BBB_I2C_CON) |= AM335X_I2C_CON_START;

 // while(am335x_i2c_busbusy(regs)==0);


 // rtems_counter_delay_nanoseconds(1000000);
 // while( !((am335x_i2c_intrawstatus(regs)) & (AM335X_I2C_INT_RECV_READY)));
 //   printf(" end am335x_i2c_setup_write_transfer\n");
}


static void am335x_i2c_setup_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{
  const i2c_msg *msgs = bus->msgs;
  uint32_t msg_todo = bus->msg_todo;
  bool send_stop = false;
  uint32_t i;

  bus->current_todo = msgs[0].len;
  for (i = 1; i < msg_todo && (msgs[i].flags & I2C_M_NOSTART) != 0; ++i) {
    bus->current_todo += msgs[i].len;
  }

  regs = bus->regs;
  
  REG(&bus->regs->BBB_I2C_BUF) |= AM335X_I2C_BUF_TXFIFO_CLR;
  REG(&bus->regs->BBB_I2C_BUF) |= AM335X_I2C_BUF_RXFIFO_CLR;
  am335x_i2c_set_address_size(msgs,regs);

 
  bus->read = (msgs->flags & I2C_M_RD) != 0;
    
  bus->already_transferred = (bus->read == true) ? 0 : 1;

  if (bus->read) {
    if (REG(&regs->BBB_I2C_CNT) == 1) {
     
      send_stop = true;
    }
 //printf("read\n");
    am335x_i2c_setup_read_transfer(bus,regs,msgs,send_stop);
  } else {
    
    am335x_i2c_setup_write_transfer(bus,regs,msgs);
  }
  
}



static void am335x_i2c_interrupt(void *arg)
{
  bbb_i2c_bus *bus = arg;
  volatile bbb_i2c_regs *regs = bus->regs;
  /* get status of enabled interrupts */
  uint32_t irqstatus = REG(&regs->BBB_I2C_IRQSTATUS);
 //printk("irqstatus:%x\n",irqstatus );
  bool done = false;
  /* Clear all enabled interrupt except receive ready and transmit ready interrupt in status register */ 
  REG(&regs->BBB_I2C_IRQSTATUS) = (irqstatus & ~( AM335X_I2C_IRQSTATUS_RRDY | AM335X_I2C_IRQSTATUS_XRDY));
//printk("irqstatus:%x\n",irqstatus );
 if (irqstatus & AM335X_I2C_INT_RECV_READY ) {
  //  printk("AM335X_I2C_INT_RECV_READY\n");
    //rtems_counter_delay_nanoseconds(1000000);

    am335x_i2c_continue_read_transfer(bus, regs);
  }
 
 else if (irqstatus & AM335X_I2C_IRQSTATUS_XRDY) {
// printk("AM335X_I2C_IRQSTATUS_XRDY\n");
    am335x_i2c_continue_write(bus,regs);
  }
 


 else if (irqstatus & AM335X_I2C_IRQSTATUS_NACK) {
    done = true;
   
    am335x_i2c_masterint_disable(regs,AM335X_I2C_IRQSTATUS_NACK);
  }

  else if (irqstatus & AM335X_I2C_IRQSTATUS_ARDY) {
  //  printk("AM335X_I2C_IRQSTATUS_ARDY\n");
  //  printk("irqstatus:%x\n",REG(&regs->BBB_I2C_IRQSTATUS) );
   done = true;
  writew(I2C_STAT_ARDY, &regs->BBB_I2C_IRQSTATUS);
  }

 
  else if (irqstatus & AM335X_I2C_IRQSTATUS_BF) {
    done = true;
   
  }

  if (done) {
    uint32_t err = irqstatus & BBB_I2C_IRQ_ERROR;
//printf("22222\n");
    am335x_i2c_next_byte(bus);

    if (bus->msg_todo == 0 ) {
    rtems_status_code sc;
//printf("11111\n");
    am335x_i2c_masterint_disable(regs, (AM335X_I2C_IRQSTATUS_RRDY | AM335X_I2C_IRQSTATUS_XRDY | AM335X_I2C_IRQSTATUS_BF));

    REG(&regs->BBB_I2C_IRQSTATUS) = err;
  
    sc = rtems_event_transient_send(bus->task_id);
    _Assert(sc == RTEMS_SUCCESSFUL);
    (void) sc;
    } else {
 //   printf("else\n");
      am335x_i2c_setup_transfer(bus, regs);
    }
  }
}

static int am335x_i2c_transfer(i2c_bus *base, i2c_msg *msgs, uint32_t msg_count)
{
  rtems_status_code sc;
  bbb_i2c_bus *bus = (bbb_i2c_bus *)base;
  volatile bbb_i2c_regs *regs;
  uint32_t i;
  rtems_task_wake_after(1);
  

  if (msg_count < 1){
    return 1;
  }
 
  for (i=0; i<msg_count;++i) {
      if ((msgs[i].flags & I2C_M_RECV_LEN) != 0) {
        return -EINVAL;
      }
  }
  
  bus->msgs = &msgs[0];
  bus->msg_todo = msg_count;
 
  
  bus->current_msg_todo = msgs[0].len;// current data size
  printf("bus->current_msg_todo:%d\n",bus->current_msg_todo);
  bus->current_msg_byte = msgs[0].buf;// current data
  
  bus->task_id = rtems_task_self();

  regs = bus->regs;
  am335x_i2c_setup_transfer(bus,regs);
  REG(&regs->BBB_I2C_IRQENABLE_SET) = BBB_I2C_IRQ_USED;

  sc = rtems_event_transient_receive(RTEMS_WAIT, bus->base.timeout);
  // If timeout then return timeout error
  if (sc != RTEMS_SUCCESSFUL) {
    am335x_i2c_reset(bus);

    rtems_event_transient_clear();

    return -ETIMEDOUT;
  }
  return 0;
}

static int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t prescaler,divider;

  
  prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;

  REG(&bus->regs->BBB_I2C_PSC) = prescaler;
  
  divider = BBB_I2C_INTERNAL_CLK/(2*clock);
  
  REG(&bus->regs->BBB_I2C_SCLL) = (divider - 7);
  
  REG(&bus->regs->BBB_I2C_SCLH) = (divider - 5);

  return 0;
}

static void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
 
  sc = rtems_interrupt_handler_remove(bus->irq, am335x_i2c_interrupt, bus);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;
 
  i2c_bus_destroy_and_free(&bus->base);
}

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
)
{
  
  bbb_i2c_bus *bus;
  rtems_status_code sc;
  int err;
  /*check bus number is >0 & <MAX*/

  bus = (bbb_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus));
  
  if (bus == NULL) {
    return -1;
  }

  bus->regs = (volatile bbb_i2c_regs *) register_base;
 
// 1. Enable clock for I2CX
  I2C0ModuleClkConfig();
// 2. pinmux setup
  am335x_i2c0_pinmux(bus);
// 3. RESET : Disable Master, autoideal 
  am335x_i2c_reset(bus);
// 4. configure bus speed  
  bus->input_clock = input_clock; // By default 100KHz. Normally pass 100KHz as argument 
 
  
  err = am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);
 
  if (err != 0) {
    (*bus->base.destroy)(&bus->base);
    
    rtems_set_errno_and_return_minus_one(-err);
  }
   bus->irq = irq;
  
  //bring I2C out of reset

   udelay(1000);
  flush_fifo(&bus->base);
  writew(0xFFFF, &bus->regs->BBB_I2C_IRQSTATUS);


 
  // 5. Start interrupt service routine & one interrupt at a time 
  sc  = rtems_interrupt_handler_install(
    irq,
    "BBB I2C",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_i2c_interrupt,
    bus
   );
  
  if (sc != RTEMS_SUCCESSFUL) {
    (*bus->base.destroy)(&bus->base);
 
    rtems_set_errno_and_return_minus_one(EIO);
  }
  // 6. start transfer for reading and writing 
  bus->base.transfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  
  return i2c_bus_register(&bus->base,bus_path);
}


static void flush_fifo(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;

 
  int stat;

  /*
   * note: if you try and read data when its not there or ready
   * you get a bus error
   */
  while (1) {
    stat = readw(&bus->regs->BBB_I2C_IRQSTATUS);
    if (stat == I2C_STAT_RRDY) {
      readb(&bus->regs->BBB_I2C_DATA);
      writew(I2C_STAT_RRDY, &bus->regs->BBB_I2C_IRQSTATUS);
      udelay(1000);
    } else
      break;
  }
}
