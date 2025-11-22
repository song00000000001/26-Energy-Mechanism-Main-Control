#pragma once

#ifdef __cplusplus

#include <assert.h>
#include "stdlib_imports.h"
#ifdef __PX4_QURT
#include "dspal_PX4_math.h"
#endif
#include "helper_functions.h"
#include "Matrix.h"
#include "SquareMatrix.h"
#include "Slice.h"
#include "Vector.h"
#include "Vector2.h"
#include "Vector3.h"
#include "Euler.h"
#include "Dcm.h"
#include "Scalar.h"
#include "Quaternion.h"
#include "AxisAngle.h"
#include "LeastSquaresSolver.h"
#include "Dual.h"
#include "PseudoInverse.h"
#include "SparseVector.h"

#endif /* __cplusplus */