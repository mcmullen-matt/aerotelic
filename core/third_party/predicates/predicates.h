/* C interface to Jonathan Shewchuk's public-domain adaptive-precision
   geometric predicates (predicates.c, 1996, double version). Call
   exactinit() once before any predicate; predicates.c must be compiled
   with strict IEEE rounding (no fast-math). */
#ifndef AEROTELIC_PREDICATES_H
#define AEROTELIC_PREDICATES_H

#ifdef __cplusplus
extern "C" {
#endif

void exactinit(void);
/* Positive when pa, pb, pc are counter-clockwise. */
double orient2d(const double* pa, const double* pb, const double* pc);
/* Positive when pd is below the plane of pa, pb, pc ("below" such that
   pa, pb, pc appear counter-clockwise from above), i.e. the determinant of
   the rows pa-pd, pb-pd, pc-pd. */
double orient3d(const double* pa, const double* pb, const double* pc, const double* pd);
/* Positive when pd lies inside the circle through pa, pb, pc (given
   counter-clockwise). */
double incircle(const double* pa, const double* pb, const double* pc, const double* pd);
/* Positive when pe lies inside the sphere through pa, pb, pc, pd, which
   must be positively oriented by orient3d(). */
double insphere(const double* pa, const double* pb, const double* pc, const double* pd, const double* pe);

#ifdef __cplusplus
}
#endif

#endif
