/*  or16.h
 *
 *  This file sets up basic CPU dependency settings based on 
 *  compiler settings.  For example, it can determine if
 *  floating point is available.  This particular implementation
 *  is specified to the OPENCORES.ORG OR16 port.
 *
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.OARcorp.com/rtems/license.html.
 *
 *  $Id$
 *
 */

#ifndef _INCLUDE_OR16_h
#define _INCLUDE_OR16_h

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  This file contains the information required to build
 *  RTEMS for a particular member of the OPENCORES.ORG OR16 family.
 *  It does this by setting variables to indicate which
 *  implementation dependent features are present in a particular
 *  member of the family.
 *
 *  This is a good place to list all the known CPU models
 *  that this port supports and which RTEMS CPU model they correspond
 *  to.
 */
 
#if defined(rtems_multilib)
/*
 *  Figure out all CPU Model Feature Flags based upon compiler 
 *  predefines. 
 */

#define CPU_MODEL_NAME  "rtems_multilib"
#define OR16_HAS_FPU     1

#elif defined(or16)
 
#define CPU_MODEL_NAME  "or16_model"
#define OR16_HAS_FPU     1
 
#else
 
#error "Unsupported CPU Model"
 
#endif

/*
 *  Define the name of the CPU family.
 */

#define CPU_NAME "OPENCORES.ORG OR16"

#ifdef __cplusplus
}
#endif

#endif /* ! _INCLUDE_OR16_h */
/* end of include file */
