/*
 * Copyright (c) 2015 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include "tmacros.h"

#include <libfdt.h>
#include <string.h>

#include "some.h"

/*
 * To generate the FDT blob use:
 *
 * dtc some.dts -O asm > some.s
 * as some.s -o some.o
 * objcopy -O binary some.o some.bin
 * rtems-bin2c some.bin some.c
 */

const char rtems_test_name[] = "LIBFDT 1";

static void test(void)
{
  const void *fdt = bsp_fdt_get();
  int status;
  uint32_t size;
  const char *alias;
  int root;
  int cells;
  const char *prop;
  fdt32_t *prop_32;
  int len;
  int d;
  int m;
  int t;
  int node;
  uint32_t phandle;
  uint32_t cell;

  status = fdt_check_header(fdt);
  printf("status:%d\n",status );
  rtems_test_assert(status == 0);

  root = fdt_path_offset(fdt, "/");
  rtems_test_assert(root >= 0);
}

static void Init(rtems_task_argument arg)
{
  TEST_BEGIN();

  test();

  TEST_END();
  rtems_test_exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS 1

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
