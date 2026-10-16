/*
 * The few C math functions the firmware needs, in our own build: images link with -nostdlib, and the host and
 * the target must compute the same bits.
 *
 * asin, atan and atan2 follow fdlibm (Sun Microsystems), which is also what the stock firmware's Keil library
 * uses, so the attitude filter can match the stock firmware bit for bit. sqrt is correctly rounded, so any
 * correct implementation gives the same bits.
 */
#ifndef EUC_MATH_LIBM_H
#define EUC_MATH_LIBM_H

double euc_sqrt(double x);
double euc_asin(double x);
double euc_atan(double x_in);
double euc_atan2(double y, double x);

#endif
