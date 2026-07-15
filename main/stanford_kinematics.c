/* stanford_kinematics.c  -  see stanford_kinematics.h for the full rationale.
 *
 * EXACT port of pupper/Kinematics.py : leg_explicit_inverse_kinematics()
 * and four_legs_inverse_kinematics(), with the mini_pupper_2pro_bsp geometry.
 * The only non-BSP part is the final joint-angle -> servo-degree mapping, which
 * is expressed RELATIVE to the resting stance so that servo-centre == your
 * calibrated "Ini" pose (no horn re-flash, no dip at gait start).
 */
#include "stanford_kinematics.h"
#include "driver_board.h"   /* db_phys: role id -> connector id (see main.c) */
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SK_RAD2DEG (180.0f / (float)M_PI)

/* ABDUCTION_OFFSETS = [-off, +off, -off, +off]  (FR, FL, RR, RL) */
static const float sk_abd_off[4] = {
    -SK_ABDUCTION_OFFSET, +SK_ABDUCTION_OFFSET,
    -SK_ABDUCTION_OFFSET, +SK_ABDUCTION_OFFSET
};

/* Servo id per (axis, leg).  axis 0=abduction 1=hip 2=knee.  leg FR,FL,RR,RL.
 * Same layout as the BSP servo_ids and your firmware. */
static const int sk_servo_id[3][4] = {
    { 1, 4,  7, 10 },   /* abduction */
    { 2, 5,  8, 11 },   /* hip       */
    { 3, 6,  9, 12 },   /* knee      */
};

/* ------------------------------------------------------------------------
 * Servo direction signs.  Applied to the joint-angle deviation from neutral.
 *
 * The BSP ServoParams.servo_multipliers (2pro) are
 *     abduction : [ 1,  1, -1, -1]
 *     hip       : [-1,  1, -1,  1]
 *     knee      : [-1,  1, -1,  1]
 * BUT Stanford's IK measures theta with atan2(-x, ...), i.e. its hip/knee
 * angle deltas are sign-INVERTED relative to this firmware's servo_write
 * polarity. Applying the raw BSP multipliers drives hip+knee BACKWARDS vs your
 * proven fRIK/fLIK/rRIK/rLIK walk (verified for all 4 legs). So the working
 * signs for THIS firmware are the BSP multipliers negated on every row:
 *     abduction : [-1, -1,  1,  1]
 *     hip       : [ 1, -1,  1, -1]
 *     knee      : [ 1, -1,  1, -1]
 * Hip & knee are VERIFIED to match the legacy forward walk direction
 * (tools/verify_stanford_ik.py). Abduction is not exercised by forward walk
 * (it stays at neutral, y=0); confirm its direction on the robot when you
 * first strafe/turn, and flip the abduction row here if a hip yaws the wrong
 * way. This is the ONLY hardware knob.
 * ---------------------------------------------------------------------- */
static const float SK_SIGN[3][4] = {
    { -1.0f, -1.0f, +1.0f, +1.0f },   /* abduction */
    { +1.0f, -1.0f, +1.0f, -1.0f },   /* hip       */
    { +1.0f, -1.0f, +1.0f, -1.0f },   /* knee      */
};

/* ==== EXACT leg IK  (pupper/Kinematics.py leg_explicit_inverse_kinematics) ==
 * Input: foot position relative to the LEG ORIGIN, Stanford signs (z<0 below).
 * Output: [abduction, hip, knee] in radians. */
void stanford_leg_ik_rad(float x, float y, float z, int leg, float out_rad[3])
{
    const float L1  = SK_L1;
    const float L2  = SK_L2;
    const float OFF = SK_ABDUCTION_OFFSET;

    /* Distance from leg origin to foot, projected into the y-z plane */
    float R_body_foot_yz = sqrtf(y * y + z * z);

    /* Distance from the fwd/back rotation point to the foot */
    float t = R_body_foot_yz * R_body_foot_yz - OFF * OFF;
    if (t < 0.0f) t = 0.0f;
    float R_hip_foot_yz = sqrtf(t);

    /* Interior y-z triangle angle (clip exactly like np.clip(-0.99,0.99)) */
    float arg = sk_abd_off[leg] / R_body_foot_yz;
    if (arg >  0.99f) arg =  0.99f;
    if (arg < -0.99f) arg = -0.99f;
    float phi = acosf(arg);

    float hip_foot_angle = atan2f(z, y);
    float abduction_angle = phi + hip_foot_angle;

    /* theta: tilted -z axis vs hip-to-foot vector (note the -x, per the ref) */
    float theta = atan2f(-x, R_hip_foot_yz);

    float R_hip_foot = sqrtf(R_hip_foot_yz * R_hip_foot_yz + x * x);

    arg = (L1 * L1 + R_hip_foot * R_hip_foot - L2 * L2)
        / (2.0f * L1 * R_hip_foot);
    if (arg >  0.99f) arg =  0.99f;
    if (arg < -0.99f) arg = -0.99f;
    float trident = acosf(arg);

    float hip_angle = theta + trident;

    arg = (L1 * L1 + L2 * L2 - R_hip_foot * R_hip_foot)
        / (2.0f * L1 * L2);
    if (arg >  0.99f) arg =  0.99f;
    if (arg < -0.99f) arg = -0.99f;
    float beta = acosf(arg);

    float knee_angle = hip_angle - ((float)M_PI - beta);

    out_rad[0] = abduction_angle;
    out_rad[1] = hip_angle;
    out_rad[2] = knee_angle;
}

/* Reconstruct the BODY-frame foot position from the gait's per-hip output and
 * hand back the LEG-ORIGIN-relative position four_legs_inverse_kinematics uses.
 *   body = LEG_ORIGIN(gait) + local ;  ik_in = body - LEG_ORIGINS
 * With the gait origin y = 49.5 mm and LEG_ORIGINS y = 23.5 mm, the resting
 * ik_in y comes out to exactly +/-26 mm = ABDUCTION_OFFSET (foot straight
 * below the hip in the leg plane), which is the intended neutral. */
static void foot_to_legorigin_frame(const sg_foot_t *f, int leg,
                                    float *lx, float *ly, float *lz)
{
    float sign_x = (leg == 0 || leg == 1) ? +1.0f : -1.0f;  /* front +59 */
    float sign_y = (leg == 1 || leg == 3) ? +1.0f : -1.0f;  /* left  +   */

    float body_x = sign_x * SG_ORIGIN_X + f->x;
    float body_y = sign_y * SG_ORIGIN_Y + f->y;
    float body_z = -f->z;                       /* downward-positive -> Stanford */

    *lx = body_x - sign_x * SK_LEG_FB;          /* == f->x                       */
    *ly = body_y - sign_y * SK_LEG_LR;          /* == sign_y*26 + f->y           */
    *lz = body_z;
}

/* Neutral joint angles (radians) at the resting stance, computed once. */
static float sk_neutral[4][3];
static int   sk_neutral_ready = 0;

static void sk_init_neutral(void)
{
    sg_foot_t rest = { 0.0f, 0.0f, SK_NEUTRAL_HEIGHT_MM };
    for (int leg = 0; leg < 4; leg++) {
        float lx, ly, lz;
        foot_to_legorigin_frame(&rest, leg, &lx, &ly, &lz);
        stanford_leg_ik_rad(lx, ly, lz, leg, sk_neutral[leg]);
    }
    sk_neutral_ready = 1;
}

void stanford_kinematics_servo_deg(const sg_foot_t feet[4], float servo_deg[13])
{
    if (!sk_neutral_ready) sk_init_neutral();

    for (int i = 0; i <= 12; i++) servo_deg[i] = 0.0f;

    for (int leg = 0; leg < 4; leg++) {
        float lx, ly, lz, jr[3];
        foot_to_legorigin_frame(&feet[leg], leg, &lx, &ly, &lz);
        stanford_leg_ik_rad(lx, ly, lz, leg, jr);

        for (int axis = 0; axis < 3; axis++) {
            float dev_rad = jr[axis] - sk_neutral[leg][axis];
            float deg = dev_rad * SK_RAD2DEG * SK_SIGN[axis][leg];
            /* sk_servo_id holds ROLE ids (tuning-era numbering); db_phys()
             * converts to the user-facing CONNECTOR id. The driver applies
             * db_phys() again (self-inverse), so the wire command is
             * identical to the proven walk for any board variant. */
            servo_deg[ db_phys(sk_servo_id[axis][leg]) ] = deg;
        }
    }
}