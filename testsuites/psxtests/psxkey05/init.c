/*
 *  Copyright (c) 2012 Zhongwei Yao.
 *  COPYRIGHT (c) 1989-2014.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include <errno.h>
#include "tmacros.h"
#include "pmacros.h"

const char rtems_test_name[] = "PSXKEY 5";

/* forward declarations to avoid warnings */
rtems_task Init( rtems_task_argument ignored );

rtems_task Init( rtems_task_argument ignored )
{
  pthread_key_t    key1, key2;
  int              sc, *value;
  int Data_array[2] = {1, 2};

  TEST_BEGIN();

  puts( "Init - pthread key1 create - OK" );
  sc = pthread_key_create( &key1, NULL );
  rtems_test_assert( !sc );

  puts( "Init - pthread key2 create - OK" );
  sc = pthread_key_create( &key2, NULL );
  rtems_test_assert( !sc );

  puts( "Init - key1 pthread_setspecific - OK" );
  sc = pthread_setspecific( key1, &Data_array[0] );
  rtems_test_assert( !sc );

  puts( "Init - key2 pthread_setspecific - OK" );
  sc = pthread_setspecific( key2, &Data_array[1] );
  rtems_test_assert( !sc );

  puts( "Init - key1 pthread_getspecific - OK" );
  value = pthread_getspecific( key1 );
  rtems_test_assert( *value == Data_array[0] );

  puts( "Init - key2 pthread_getspecific - OK" );
  value = pthread_getspecific( key2 );
  rtems_test_assert( *value == Data_array[1] );

  puts( "Init - key1 pthread_setspecific - OK" );
  sc = pthread_setspecific( key1, &Data_array[1] );
  rtems_test_assert( !sc );

  puts( "Init - key1 pthread_getspecific - OK" );
  value = pthread_getspecific( key1 );
  rtems_test_assert( *value == Data_array[1] );

  puts( "Init - pthread key1 delete - OK" );
  sc = pthread_key_delete( key1 );
  rtems_test_assert( sc == 0 );

  puts( "Init - pthread key2 delete - OK" );
  sc = pthread_key_delete( key2 );
  rtems_test_assert( sc == 0 );

  TEST_END();
  rtems_test_exit(0);
}

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_MAXIMUM_TASKS          1
#define CONFIGURE_MAXIMUM_POSIX_KEYS     2

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE


#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/* global variables */
