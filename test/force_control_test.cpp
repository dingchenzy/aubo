#include "mock_common.h"
#define _FUNC(m, n, method, ...) EQ##n(m, method)
#define _INST(m, n, method, ...) EQ##n(m, method)
ForceControl_DECLARES
