#include "webots/supervisor.h"
#include "webots/robot.h"
#include "webots/emitter.h"
#include "webots/led.h"
#include "webots/gps.h"
#include "webots/distance_sensor.h"
#include "webots/differential_wheels.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>

#define NB_DIST_SENS 8
#define NB_IN 4
#define NB_LEDS 8
#define NB_FLOOR_SENS 3
const int off=0;
const int on=1;

static int time_step;
static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag gps;

WbDeviceTag fs[NB_FLOOR_SENS];
WbDeviceTag ps[NB_DIST_SENS];
WbDeviceTag led[NB_LEDS];
FILE *f[NB_DIST_SENS];
FILE *fps[NB_FLOOR_SENS];
char fName[20];

void reset(void);
void save_data(void);
void close_files(void);