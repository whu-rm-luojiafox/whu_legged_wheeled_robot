#include "arm_math.h"
#include <assert.h>
#include <stdlib.h>
unsigned allocation_count;
void *velocity_test_alloc(size_t size)
{
    void *p = malloc(size);
    assert(p);
    ++allocation_count;
    return p;
}
void arm_mat_init_f32(arm_matrix_instance_f32 *m, uint16_t r, uint16_t c, float *p)
{
    m->numRows = r;
    m->numCols = c;
    m->pData = p;
}
static void same(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                 const arm_matrix_instance_f32 *c)
{
    assert(a->numRows == b->numRows && a->numCols == b->numCols && a->numRows == c->numRows &&
           a->numCols == c->numCols);
}
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *c)
{
    same(a, b, c);
    for (unsigned i = 0; i < a->numRows * a->numCols; ++i)
        c->pData[i] = a->pData[i] + b->pData[i];
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *c)
{
    same(a, b, c);
    for (unsigned i = 0; i < a->numRows * a->numCols; ++i)
        c->pData[i] = a->pData[i] - b->pData[i];
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                            arm_matrix_instance_f32 *c)
{
    assert(a->numCols == b->numRows && c->numRows == a->numRows && c->numCols == b->numCols);
    for (unsigned r = 0; r < c->numRows; ++r)
        for (unsigned s = 0; s < c->numCols; ++s)
        {
            float v = 0;
            for (unsigned k = 0; k < a->numCols; ++k)
                v += a->pData[r * a->numCols + k] * b->pData[k * b->numCols + s];
            c->pData[r * c->numCols + s] = v;
        }
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b)
{
    assert(a->numRows == b->numCols && a->numCols == b->numRows);
    for (unsigned r = 0; r < a->numRows; ++r)
        for (unsigned c = 0; c < a->numCols; ++c)
            b->pData[c * b->numCols + r] = a->pData[r * a->numCols + c];
    return ARM_MATH_SUCCESS;
}
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b)
{
    assert(a->numRows == 1 && a->numCols == 1 && b->numRows == 1 && b->numCols == 1);
    if (a->pData[0] == 0) return ARM_MATH_SINGULAR;
    b->pData[0] = 1.0f / a->pData[0];
    return ARM_MATH_SUCCESS;
}
