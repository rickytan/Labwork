
#ifndef REQUIRED_H
#define REQUIRED_H

#pragma once

#include "standard_types.h"
#include "gp_types.h"
#include "step.h"
#include "ais_api.h"
#include "brep_api.h"
#include "geom_func.h"
#include "geom_types.h"
#include "topo_func.h"
#include "topo_types.h"
#include "v3d_api.h"
#include "select_api.h"


#ifdef _WIN32
#pragma comment(lib, "TKernel.lib")
#pragma comment(lib, "TKBRep.lib")
#pragma comment(lib, "TKG2d.lib")
#pragma comment(lib, "TKG3d.lib")
#pragma comment(lib, "TKMath.lib")
#pragma comment(lib, "TKOpenGl.lib")
#pragma comment(lib, "TKPShape.lib")
#pragma comment(lib, "TKService.lib")
#pragma comment(lib, "TKShapeSchema.lib")
#pragma comment(lib, "TKShapeSchema.lib")
#pragma comment(lib, "TKPrim.lib")

#pragma comment(lib, "TKV3d.lib")
#pragma comment(lib, "TKTopAlgo.lib")
#pragma comment(lib, "TKSTEP.lib")
#pragma comment(lib, "TKSTEP209.lib")
#pragma comment(lib, "TKSTEPAttr.lib")
#pragma comment(lib, "TKSTEPBase.lib")
#pragma comment(lib, "TKXSBase.lib")
#endif // _WIN32

#endif