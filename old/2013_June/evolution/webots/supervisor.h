/**********************************************************************************/
/* File:         supervisor.h                                                     */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Supervisor node           */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_SUPERVISOR_H
#define WB_SUPERVISOR_H

#define WB_USING_C_API
#include "types.h"
#include "nodes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int WbFieldType;
#define WB_SF_BOOL        0x01
#define WB_SF_INT32       0x02
#define WB_SF_FLOAT       0x03
#define WB_SF_VEC2F       0x04
#define WB_SF_VEC3F       0x05
#define WB_SF_ROTATION    0x06
#define WB_SF_COLOR       0x07
#define WB_SF_STRING      0x08
#define WB_SF_NODE        0x09
#define WB_MF             0x10
#define WB_MF_INT32       (WB_MF|WB_SF_INT32)
#define WB_MF_FLOAT       (WB_MF|WB_SF_FLOAT)
#define WB_MF_VEC2F       (WB_MF|WB_SF_VEC2F)
#define WB_MF_VEC3F       (WB_MF|WB_SF_VEC3F)
#define WB_MF_COLOR       (WB_MF|WB_SF_COLOR)
#define WB_MF_STRING      (WB_MF|WB_SF_STRING)
#define WB_MF_NODE        (WB_MF|WB_SF_NODE)

void          wb_supervisor_simulation_quit(int status);
void          wb_supervisor_simulation_revert();
void          wb_supervisor_simulation_physics_reset();
void          wb_supervisor_export_image(const char *file,int quality);
void          wb_supervisor_start_movie(const char *file,int width,int height,int type,int quality);
void          wb_supervisor_stop_movie();
void          wb_supervisor_set_label(int id,const char *string,double xpos,double ypos,
                                      double size,int color,double transparency);

WbNodeRef     wb_supervisor_node_get_root();
WbNodeRef     wb_supervisor_node_get_from_def(const char *def);
WbNodeType    wb_supervisor_node_get_type(WbNodeRef node);
const char   *wb_supervisor_node_get_type_name(WbNodeRef);
WbFieldRef    wb_supervisor_node_get_field(WbNodeRef node, const char *field_name);

const double *wb_supervisor_node_get_center_of_mass(WbNodeRef node);
const double *wb_supervisor_node_get_contact_points(WbNodeRef node, int *number_of_contact_points);
const double *wb_supervisor_node_get_orientation(WbNodeRef node);
const double *wb_supervisor_node_get_position(WbNodeRef node);
bool          wb_supervisor_node_get_static_balance(WbNodeRef node);
WbFieldType   wb_supervisor_field_get_type(WbFieldRef field);
const char   *wb_supervisor_field_get_type_name(WbFieldRef field);
int           wb_supervisor_field_get_count(WbFieldRef field);

bool          wb_supervisor_field_get_sf_bool(WbFieldRef field);
int           wb_supervisor_field_get_sf_int32(WbFieldRef field);
double        wb_supervisor_field_get_sf_float(WbFieldRef field);
const double *wb_supervisor_field_get_sf_vec2f(WbFieldRef field);
const double *wb_supervisor_field_get_sf_vec3f(WbFieldRef field);
const double *wb_supervisor_field_get_sf_rotation(WbFieldRef field);
const double *wb_supervisor_field_get_sf_color(WbFieldRef field);
const char   *wb_supervisor_field_get_sf_string(WbFieldRef field);
WbNodeRef     wb_supervisor_field_get_sf_node(WbFieldRef field);

int           wb_supervisor_field_get_mf_int32(WbFieldRef field, int index);
double        wb_supervisor_field_get_mf_float(WbFieldRef field, int index);
const double *wb_supervisor_field_get_mf_vec2f(WbFieldRef field, int index);
const double *wb_supervisor_field_get_mf_vec3f(WbFieldRef field, int index);
const double *wb_supervisor_field_get_mf_color(WbFieldRef field, int index);
const char   *wb_supervisor_field_get_mf_string(WbFieldRef field, int index);
WbNodeRef     wb_supervisor_field_get_mf_node(WbFieldRef field, int index);

void          wb_supervisor_field_set_sf_bool(WbFieldRef field, bool value);
void          wb_supervisor_field_set_sf_int32(WbFieldRef field, int value);
void          wb_supervisor_field_set_sf_float(WbFieldRef field, double value);
void          wb_supervisor_field_set_sf_vec2f(WbFieldRef field, const double values[2]);
void          wb_supervisor_field_set_sf_vec3f(WbFieldRef field, const double values[3]);
void          wb_supervisor_field_set_sf_rotation(WbFieldRef field, const double values[4]);
void          wb_supervisor_field_set_sf_color(WbFieldRef field, const double values[3]);
void          wb_supervisor_field_set_sf_string(WbFieldRef field, const char *value);

void          wb_supervisor_field_set_mf_int32(WbFieldRef field, int index, int value);
void          wb_supervisor_field_set_mf_float(WbFieldRef field, int index, double value);
void          wb_supervisor_field_set_mf_vec2f(WbFieldRef field, int index, const double values[2]);
void          wb_supervisor_field_set_mf_vec3f(WbFieldRef field, int index, const double values[3]);
void          wb_supervisor_field_set_mf_color(WbFieldRef field, int index, const double values[3]);
void          wb_supervisor_field_set_mf_string(WbFieldRef field, int index, const char *value);

void          wb_supervisor_field_import_mf_node(WbFieldRef field, int position, const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* SUPERVISOR_H */
