/**********************************************************************************/
/* File:         types.h                                                          */
/* Date:         Feb 23, 2010                                                     */
/* Description:  Common definitions for both the C and C++ APIs                   */
/* Authors:      Olivier Michel, Yvan Bourquin                                    */
/*                                                                                */
/* Copyright (c) 2010 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#if defined (WB_USING_CPP_API) && defined (WB_USING_C_API) && !defined (WB_ALLOW_MIXING_C_AND_CPP_API)
#ifdef _MSC_VER
#pragma message("warning: mixing the C and C++ APIs in the same controller is not supported.")
#else
#warning "mixing the C and C++ APIs in the same controller is not supported."
#endif
#endif

#ifndef WB_TYPES_H
#define WB_TYPES_H

/* There can be a maximum of 65535 devices on a robot */
typedef unsigned short WbDeviceTag; /* identifier of a device */

/* Opaque type definitions */
typedef struct WbImageStructPrivate  *WbImageRef;
typedef struct WbMotionStructPrivate *WbMotionRef;
typedef struct WbNodeStructPrivate   *WbNodeRef;
typedef struct WbFieldStructPrivate  *WbFieldRef;

/* define "bool" type for C controllers */
/* C++ code will use the standard definition of "bool" */
#ifndef __cplusplus

#ifndef bool
#define bool char
#endif

#ifndef true
#define true ((bool)1)
#endif

#ifndef false
#define false ((bool)0)
#endif

#ifndef INFINITY
#define INFINITY (1.0 / 0.0)
#endif

#endif

/* Allow us to mark functions as 'deprecated' and have gcc emit a nice warning for each use. */
/* Usage: int foo(char) WB_DEPRECATED; */
/* and then gcc will emit a warning for each usage of the function. */
#ifndef WB_DEPRECATED
#if __GNUC__ >= 3 && !defined WB_MATLAB_LOADLIBRARY
#define WB_DEPRECATED __attribute__((deprecated))
#else
#define WB_DEPRECATED
#endif
#endif

#endif /* WB_TYPES_H */
