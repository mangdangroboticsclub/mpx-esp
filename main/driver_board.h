/* driver_board.h
 *
 * SPI backend for the Mini Pupper "controller driver board" (4x AT32F413).
 *
 * This replaces the direct Feetech serial-bus servo control (SCServo / ftservo)
 * used previously. Instead of bit-banging the SCS bus, the ESP32 now talks to
 * four AT32F413 servo-driver boards over SPI (one board per leg, 3 servos each).
 *
 * Each SPI frame (host_SMS_t) carries, per servo: { mode, position, torque, kp, kd }.
 * IMPORTANT: there is NO "speed" field. In MODE_POSITION the AT32 firmware reads
 * the "torque" field as a MAX CURRENT LIMIT in milliamps (max_current_mA), NOT a
 * speed. So the legacy "goal_speed" value is translated into a current limit.
 *
 * Wiring (matches minipupper2pro/esp32 reference, official Mangdang board):
 *   SPI2_HOST  MOSI=11  MISO=13  CLK=12
 *   CS: FR=9  FL=10  RR=21  RL=14
 *   Power enable: GPIO 8 (high = servo bus powered)
 *
 * Servo ID -> leg mapping (unchanged from the gait code):
 *   1,2,3   = Front Right (board 0)
 *   4,5,6   = Front Left  (board 1)
 *   7,8,9   = Rear  Right (board 2)
 *   10,11,12= Rear  Left  (board 3)
 */
#ifndef DRIVER_BOARD_H
#define DRIVER_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------
 * SERVO BOARD VARIANT SELECT
 *   1 = normal  (black board - this robot, as wired now)
 *   2 = swapped (the other build: in each leg the HIP and CALF servos are
 *                plugged the other way round -> physical channels swapped
 *                1<->3, 4<->6, 7<->9, 10<->12; thighs 2,5,8,11 unchanged)
 *   3 = purple board  (same design as board 1, but the 1-2-3 group sits where
 *                4-5-6 is and 7-8-9 where 10-11-12 is. The leg groups swap
 *                straight across with NO within-leg reversal. Net remap,
 *                verified on hardware: 1<->4, 2<->5, 3<->6, 7<->10, 8<->11,
 *                9<->12.)
 *
 * db_phys() maps a LOGICAL servo id (what the gait / IK / CLI / calibration
 * all use, 1..12) to the PHYSICAL channel on the driver boards. Doing the
 * remap HERE - at the single hardware boundary - means every path agrees:
 * the walk, the `pos` command, sweep/swalk, position feedback and the
 * offset[] calibration all address the same servo by the same id. Set
 * SERVO_BOARD and re-flash to switch builds.
 * ---------------------------------------------------------------------- */
#define SERVO_BOARD 1

static inline int db_phys(int logical){
#if SERVO_BOARD == 2
    switch(logical){
        case 1:  return 3;   case 3:  return 1;
        case 4:  return 6;   case 6:  return 4;
        case 7:  return 9;   case 9:  return 7;
        case 10: return 12;  case 12: return 10;
        default: return logical;
    }
#elif SERVO_BOARD == 3
    /* purple board: 1-2-3 group swapped with 4-5-6, 7-8-9 with 10-11-12,
     * straight across with NO within-leg reversal. Net (verified on
     * hardware): 1<->4, 2<->5, 3<->6, 7<->10, 8<->11, 9<->12. */
    switch(logical){
        case 1:  return 4;   case 4:  return 1;
        case 2:  return 5;   case 5:  return 2;
        case 3:  return 6;   case 6:  return 3;
        case 7:  return 10;  case 10: return 7;
        case 8:  return 11;  case 11: return 8;
        case 9:  return 12;  case 12: return 9;
        default: return logical;
    }

#elif SERVO_BOARD == 4  //White pupper  -   Already 5 ports are broken
    /* abduction <-> knee (calf) swap: calf servo plugged into abduction port.
     * 1<->3, 4<->6, 7<->9, 10<->12; hips 2,5,8,11 unchanged. */
    switch(logical){
        // 8 servos connected: 4 calves (3,6,9,12) + 4 thighs (2,5,8,11).
        // 4 abductions (1,4,7,10) on broken ports (3,12,9,8,6) — no servo.
        // Port 1→Calf3  Port 2→Thigh2  Port 4→Calf6  Port 5→Thigh5
        // Port 6→Thigh8  Port 7→Calf9  Port 10→Calf12  Port 11→Thigh11
        case 1:  return 3;   // FR Abd   → physical 3 (broken)
        case 2:  return 2;   // FR Thigh → physical 2
        case 3:  return 1;   // FR Calf  → physical 1
        case 4:  return 12;  // FL Abd   → physical 12 (broken)
        case 5:  return 5;   // FL Thigh → physical 5
        case 6:  return 4;   // FL Calf  → physical 4
        case 7:  return 9;   // RR Abd   → physical 9 (broken)
        case 8:  return 6;   // RR Thigh → physical 6 (cross-board: FL board CS10)
        case 9:  return 7;   // RR Calf  → physical 7
        case 10: return 8;   // RL Abd   → physical 8 (broken)
        case 11: return 11;  // RL Thigh → physical 11
        case 12: return 10;  // RL Calf  → physical 10
        default: return logical;
    }

    #elif SERVO_BOARD == 5  // White pupper (ports 4,5,6,8 broken; abduction servos unused)
    switch(logical){
        case 6:  return 3;   // logical  6 (calf)  → physical 3
        case 5:  return 2;   // logical  5 (thigh) → physical 2
        case 3:  return 1;   // logical  3 (calf)  → physical 1
        case 2:  return 7;   // logical  2 (thigh) → physical 7
        case 8:  return 11;  // logical  8 (thigh) → physical 11
        case 9:  return 9;   // logical  9 (calf)  → physical 9
        case 11: return 12;  // logical 11 (thigh) → physical 12
        case 12: return 10;  // logical 12 (calf)  → physical 10
        case 1:  return 4;   // logical  1 (abd)   → physical 4  (broken)
        case 4:  return 5;   // logical  4 (abd)   → physical 5  (broken)
        case 7:  return 6;   // logical  7 (abd)   → physical 6  (broken)
        case 10: return 8;   // logical 10 (abd)   → physical 8  (broken)
        default: return logical;
    }

#else
    return logical;
#endif
}

/* Inverse: given a PHYSICAL channel (1..12), return the LOGICAL servo (1..12)
 * that is wired to it. Used by driver_board_sync_write() to route pos[] data.
 * Unlike db_phys(), this is NOT required to be symmetric — it handles
 * cross-board wiring (e.g. RR servo plugged into the FL board). */
static inline int db_phys_inv(int physical){
#if SERVO_BOARD == 5
    switch(physical){
        case 1:  return 3;   // physical 1  → logical  3 (calf)
        case 2:  return 5;   // physical 2  → logical  5 (thigh)
        case 3:  return 6;   // physical 3  → logical  6 (calf)
        case 4:  return 1;   // physical 4  → logical  1 (abd, broken)
        case 5:  return 4;   // physical 5  → logical  4 (abd, broken)
        case 6:  return 7;   // physical 6  → logical  7 (abd, broken)
        case 7:  return 2;   // physical 7  → logical  2 (thigh)
        case 8:  return 10;  // physical 8  → logical 10 (abd, broken)
        case 9:  return 9;   // physical 9  → logical  9 (calf)
        case 10: return 12;  // physical 10 → logical 12 (calf)
        case 11: return 8;   // physical 11 → logical  8 (thigh)
        case 12: return 11;  // physical 12 → logical 11 (thigh)
        default: return physical;
    }
#else
    // original mapping for SERVO_BOARD 1 (default)
    switch(physical){
        // FR board (CS9): ports 1,2,3
        case 1:  return 1;   // FR Calf  -> logical 3
        case 2:  return 2;   // FR Thigh -> logical 2
        case 3:  return 3;   // broken   -> logical 1 (FR Abd, no servo)
        // FL board (CS10): ports 4,5,6
        case 4:  return 4;   // FL Calf  -> logical 6
        case 5:  return 5;   // FL Thigh -> logical 5
        case 6:  return 6;   // RR Thigh -> logical 8 (cross-board!)
        // RR board (CS21): ports 7,8,9
        case 7:  return 7;   // RR Calf  -> logical 9
        case 8:  return 8;  // broken   -> logical 10 (RL Abd, no servo)
        case 9:  return 9;   // broken   -> logical 7 (RR Abd, no servo)
        // RL board (CS14): ports 10,11,12
        case 10: return 10;  // RL Calf  -> logical 12
        case 11: return 11;  // RL Thigh -> logical 11
        case 12: return 12;   // broken   -> logical 4 (FL Abd, no servo)
        default: return physical;
    }
#endif
}

/* Initialise the SPI bus, the 4 driver-board devices, and the power-enable pin.
 * Leaves servo power ON (GPIO8 high). Call once at start-up. */
void driver_board_init(void);

/* Enable / disable servo bus power (GPIO 8). */
void driver_board_power(bool on);

/* Push all 12 setpoints to the four driver boards in one pass (4 SPI frames).
 *   pos[12]   : per-servo goal position, SCS scale 0..1023 (511 = centre),
 *               index 0 == servo ID 1 ... index 11 == servo ID 12.
 *   cur_mA[12]: per-servo current/torque limit in milliamps (the "torque" field).
 * Feedback (present position + present current) is captured on the same
 * transaction and cached for driver_board_present_*().
 */
void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12]);

/* Cached feedback from the last sync_write. ch = 1..12. */
int16_t  driver_board_present_current(int ch);   /* motor current, mA (signed) */
uint16_t driver_board_present_position(int ch);  /* SCS scale 0..1023          */

/* ---- NTC servo temperature ---------------------------------------------
 * The AT32 firmware samples one 10k NTC per servo (MTA10103F3380F00, 10k
 * pull-up to 3V3, NTC to GND) at ~22 Hz and ships the reading in the
 * feedback frame's reserved1 field as a SIGNED value in 0.1 degC
 * (253 => 25.3 degC).
 *
 * That field rides along on EVERY feedback frame, so this cache is
 * refreshed FOR FREE by driver_board_sync_write() while the gait runs -
 * zero extra SPI traffic. When the gait is parked (CLI mode) refresh it
 * with driver_board_poll() / driver_board_poll_board().
 *
 * Returns degC, or DB_TEMP_INVALID if that servo has never answered.     */
#define DB_TEMP_INVALID  (-273.0f)

float driver_board_present_temperature(int ch);  /* degC, ch = 1..12 */

/* Refresh the whole feedback cache (position / current / temperature) for
 * ONE board by resending its last commanded frame. board = 0..3 in
 * PHYSICAL board order (FR, FL, RR, RL). Setpoints are unchanged, so this
 * is safe while parked in CLI mode - an idle servo stays idle. */
bool driver_board_poll_board(int board /*0..3*/);

/* ---- AT32 sms_config parameter access over SPI (web CLI) ----------------
 * Parameter ids match the AT32 UART CLI table order.
 * NOTE: do not run these concurrently with driver_board_sync_write() —
 * pause the gait loop first (CLI mode).                                    */
enum {
    DB_PARAM_REVERSE_POSITION_SENSOR = 0,
    DB_PARAM_MIN_POSITION_ADC,      /* 1 */
    DB_PARAM_MAX_POSITION_ADC,      /* 2 */
    DB_PARAM_RANGE_POSITION_DEG,    /* 3 */
    DB_PARAM_REVERSE_MOTOR,         /* 4 */
    DB_PARAM_KP_POSITION,           /* 5 */
    DB_PARAM_KD_POSITION,           /* 6 */
    DB_PARAM_KP_CURRENT,            /* 7 */
    DB_PARAM_KFF_CURRENT,           /* 8 */
    DB_PARAM_MAX_PWM_DUTY_CYCLE,    /* 9 */
    DB_PARAM_COUNT
};

/* Parameter name for id, or NULL if out of range. */
const char *driver_board_param_name(int param_id);
/* Name -> id, or -1 if unknown. */
int driver_board_param_id(const char *name);

/* servo = 1..12 (global id). Return true on success. */
bool driver_board_set_param(int servo, int param_id, float value);
bool driver_board_get_param(int servo, int param_id, float *out);

/* board = 0..3, or -1 for all boards. */
bool driver_board_save_config(int board);      /* commit sms_config to flash */
bool driver_board_factory_restore(int board);  /* factory defaults (RAM only) */

/* ---- direct single-servo control (web CLI pos/tor/stop) -----------------
 * Sends one servo command frame to the servo's board; the OTHER two servos
 * on that board keep their last commanded values (shadow of the last
 * sync_write / direct write). Angle is the RAW AT32 angle in degrees,
 * 0..270 (135 = centre) - same units as the AT32 UART CLI 'pos' command. */
#define DB_MODE_IDLE     0
#define DB_MODE_POSITION 1
#define DB_MODE_TORQUE   2

bool driver_board_direct(int servo /*1..12*/, uint16_t mode,
                         float pos_deg, int16_t current_mA);

/* Refresh feedback for this servo's board by resending the last commanded
 * frame (setpoints unchanged). Use driver_board_present_*() afterwards.
 * Used by the web CLI 'trace' live view.                                  */
bool driver_board_poll(int servo /*1..12*/);

/* ---- live control-loop values (web trace, needs new AT32 firmware) ------
 * Same set the AT32 uart_trace() prints:                                   */
enum {
    DB_LIVE_POS_ADC = 0,        /* position sensor, raw ADC     */
    DB_LIVE_CUR_ADC,            /* current sense, raw ADC       */
    DB_LIVE_SETPOINT_POS_DEG,   /* commanded position, deg      */
    DB_LIVE_PRESENT_POS_DEG,    /* measured position, deg       */
    DB_LIVE_ERROR_POS_DEG,      /* position error, deg          */
    DB_LIVE_MAX_CURRENT_MA,     /* current cap (position mode)  */
    DB_LIVE_SETPOINT_CUR_MA,    /* current setpoint (torque)    */
    DB_LIVE_PRESENT_CUR_MA,     /* measured motor current, mA   */
    DB_LIVE_ERROR_CUR_MA,       /* current error, mA            */
    DB_LIVE_PWM_DUTY,           /* PWM duty cycle 0..1          */
    DB_LIVE_MODE,               /* 0 idle 1 position 2 torque   */
    DB_LIVE_LOOP_COUNTER,       /* control loop tick counter    */
    DB_LIVE_TEMPERATURE_C,      /* NTC servo temperature, degC  */
    DB_LIVE_COUNT
};

bool driver_board_get_live(int servo /*1..12*/, int live_id, float *out);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BOARD_H */
