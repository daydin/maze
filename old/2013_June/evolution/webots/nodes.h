/**********************************************************************************/
/* File:         nodes.h                                                          */
/* Date:         June 10, 2010                                                    */
/* Description:  Webots C programming interface                                   */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2010 Cyberbotics - www.cyberbotics.com                           */
/*                                                                                */
/**********************************************************************************/

#ifndef WB_NODES_H
#define WB_NODES_H

typedef int WbNodeType;

// IMPORTANT: any modification of this file must also be propagated to:
//  1. include/controller/cpp/webots/Node.hpp
//  2. lib/matlab/mgenerate.c

#define WB_NODE_NO_NODE              0

#define WB_NODE_APPEARANCE           1 /* VRML nodes */
#define WB_NODE_BACKGROUND           2
#define WB_NODE_BOX                  3
#define WB_NODE_COLOR                4
#define WB_NODE_CONE                 5
#define WB_NODE_COORDINATE           6
#define WB_NODE_CYLINDER             7
#define WB_NODE_DIRECTIONAL_LIGHT    8
#define WB_NODE_ELEVATION_GRID       9
#define WB_NODE_EXTRUSION           10
#define WB_NODE_FOG                 11
#define WB_NODE_GROUP               12
#define WB_NODE_IMAGE_TEXTURE       13
#define WB_NODE_INDEXED_FACE_SET    14
#define WB_NODE_INDEXED_LINE_SET    15
#define WB_NODE_MATERIAL            16
#define WB_NODE_POINT_LIGHT         17
#define WB_NODE_SHAPE               18
#define WB_NODE_SPHERE              19
#define WB_NODE_SPOT_LIGHT          20
#define WB_NODE_SWITCH              21 /* not supported */
#define WB_NODE_TEXTURE_COORDINATE  22
#define WB_NODE_TEXTURE_TRANSFORM   23
#define WB_NODE_TRANSFORM           24
#define WB_NODE_VIEWPOINT           25
#define WB_NODE_WORLD_INFO          26

#define WB_NODE_CAPSULE             30 /* non-vrml geometry */
#define WB_NODE_PLANE               31

#define WB_NODE_ROBOT               40 /* robots */
#define WB_NODE_SUPERVISOR          41
#define WB_NODE_DIFFERENTIAL_WHEELS 42

#define WB_NODE_ANCHOR_PARAMETER    50 /* misc */
#define WB_NODE_BALL_JOINT          51 
#define WB_NODE_CAMERA_ZOOM         52 
#define WB_NODE_CHARGER             53
#define WB_NODE_CONTACT_PROPERTIES  54
#define WB_NODE_DAMPING             55
#define WB_NODE_HINGE_JOINT         56
#define WB_NODE_HINGE_2_JOINT         57
#define WB_NODE_JOINT_PARAMETERS    58
#define WB_NODE_PHYSICS             59
#define WB_NODE_SLIDER_JOINT        60
#define WB_NODE_SOLID               61 
#define WB_NODE_SOLID_REFERENCE     62


#define WB_NODE_ACCELEROMETER       70 /* devices */
#define WB_NODE_ANGULAR_MOTOR       71
#define WB_NODE_CAMERA              72
#define WB_NODE_COMPASS             73
#define WB_NODE_CONNECTOR           74
#define WB_NODE_DISPLAY             75
#define WB_NODE_DISTANCE_SENSOR     76
#define WB_NODE_EMITTER             77
#define WB_NODE_GPS                 78
#define WB_NODE_GYRO                79
#define WB_NODE_INERTIAL_UNIT       80
#define WB_NODE_LED                 81
#define WB_NODE_LIGHT_SENSOR        82
#define WB_NODE_LINEAR_MOTOR        83
#define WB_NODE_MICROPHONE          84
#define WB_NODE_MOTOR               85
#define WB_NODE_PEN                 86
#define WB_NODE_POSITION_SENSOR     87
#define WB_NODE_RADIO               88
#define WB_NODE_RECEIVER            89
#define WB_NODE_SERVO               90
#define WB_NODE_SPEAKER             91
#define WB_NODE_TOUCH_SENSOR        92

#endif /* WB_NODES_H */
