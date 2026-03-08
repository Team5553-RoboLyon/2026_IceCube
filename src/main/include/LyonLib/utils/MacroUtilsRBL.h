/*******************************************************************************
 * 
 * File        : MacroUtilsRBL.h (v1.2)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : mathematical and bit manipulation macros
 * 
 * Authors     : AKA (2025), last update by VTT (2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

#pragma once
#include <cmath>

/* =============================================================================
 *  MATHEMATICAL CONSTANTS
 * ========================================================================== */

// -- PI ------------------------------------------------------------------------
#define NF64_PI        3.14159265358979323846     // PI
#define NF64_2PI       6.28318530717958647693     // 2*PI
#define NF64_PI_2      1.57079632679489661923     // PI/2
#define NF64_PI_3      1.04719755119659774615     // PI/3
#define NF64_PI_4      0.78539816339744830962     // PI/4
#define NF64_1_PI      0.31830988618379067154     // 1/PI
#define NF64_1_2PI     0.15915494309189533577     // 1/(2*PI)
#define NF64_2_PI      0.63661977236758134308     // 2/PI
#define NF64_1_SQRTPI  0.56418958354775628695     // 1/sqrt(PI)
#define NF64_2_SQRTPI  1.12837916709551257390     // 2/sqrt(PI)

// -- SQRT(2) -------------------------------------------------------------------
#define NF64_SQRT2     1.41421356237309504880     // sqrt(2)
#define NF64_1_SQRT2   0.70710678118654752440     // 1/sqrt(2)
#define NF64_SQRT2_2   0.70710678118654752440     // sqrt(2)/2


/* =============================================================================
 *  MACROS : CONVERTIONS
 * ========================================================================== */

// -- Deg ↔ Rad -----------------------------------------------------------------
#define NDEGtoRAD(deg)  ((NF64_PI / 180.0) * (deg))      // Deg -> Radian
#define NRADtoDEG(rad)  ((180.0 / NF64_PI) * (rad))      // Radian -> Deg

// -- GENERIC ------------------------------------------------------
#define NABS(a)         (((a) < 0) ? -(a) : (a))
#define NMAX(a, b)      (((a) > (b)) ? (a) : (b))
#define NMIN(a, b)      (((a) < (b)) ? (a) : (b))
#define NSIGN(a)        (((a) > 0) - ((a) < 0))           // 1, -1 or 0
#define NHYPOT(a, b)    (std::sqrt((a)*(a) + (b)*(b)))

#define NDEADBAND(a, t) (((a) > -(t) && (a) < (t)) ? 0 : (a))
#define NCLAMP(mn, a, mx) (((a) < (mn)) ? (mn) : ((a) > (mx)) ? (mx) : (a))
#define NEPSILON_EQUALS(a, b, epsilon) (((a) - (epsilon)) <= (b)) && (((a) + (epsilon)) >= (b))
#define IS_IN_RANGE(val, target, tolerance)    ((val <= (target + tolerance)) && (val >= (target - tolerance)))

#define NLERP(a, b, t)  ((a) + ((b) - (a)) * (t))

// -- ANGLES ----------------------------------------------------
#define WRAP_ANGLE_0_TO_2PI(angle_rad) \
    (((std::fmod((angle_rad), NF64_2PI)) < 0) ? \
        (std::fmod((angle_rad), NF64_2PI) + NF64_2PI) : \
        (std::fmod((angle_rad), NF64_2PI)))

#define WRAP_ANGLE_0_TO_360(angle_deg) \
    (((std::fmod((angle_deg), 360.0)) < 0) ? \
        (std::fmod((angle_deg), 360.0) + 360.0) : \
        (std::fmod((angle_deg), 360.0)))

#define WRAP_ANGLE_NEG_PI_TO_PI(angle_rad) \
(((std::fmod((angle_rad), NF64_2PI)) >= NF64_PI) ? \
 (std::fmod((angle_rad), NF64_2PI) - NF64_2PI) : \
 ((std::fmod((angle_rad), NF64_2PI)) <= -NF64_PI) ? \
 (std::fmod((angle_rad), NF64_2PI) + NF64_2PI) : \
 (std::fmod((angle_rad), NF64_2PI)))

static double PIPI(double angle_rad)
{
    double result = std::fmod(angle_rad,NF64_2PI);
    if (result < 0)
    {
        result += NF64_2PI;
    }
    if (result > NF64_PI)
    {
        result -= NF64_2PI;
    }
    return result;
}


/* =============================================================================
 *  MACROS : TABLES DE BITES
 * ========================================================================== */

#define BITSET(val, bit_id)     ((val) |=  (1 << (bit_id)))
#define BITCLEAR(val, bit_id)   ((val) &= ~(1 << (bit_id)))
#define BITGET(val, bit_id)     (((val) >> (bit_id)) & 1)

#define BITSSET(bits, setVal)   ((bits) |  (setVal))
#define BITSGET(val, mask)      ((val)  &  (mask))
