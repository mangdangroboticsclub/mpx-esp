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

/* ------------------------------------------------------------------------
 * WHICH DRIVER BOARDS ARE FITTED  (one bit per board, not a count)
 *
 *   bit 0 = board 0 : servos 1-3   Front Right   CS = GPIO 9
 *   bit 1 = board 1 : servos 4-6   Front Left    CS = GPIO 10
 *   bit 2 = board 2 : servos 7-9   Rear  Right   CS = GPIO 21
 *   bit 3 = board 3 : servos 10-12 Rear  Left    CS = GPIO 14
 *
 *   0x0F = full robot (all four boards)
 *   0x02 = single-board bench test on CS GPIO 10 -> servos 4,5,6
 *
 * This used to be a plain count, which could only ever mean "the first N
 * boards". That is wrong for bench work: `scan` found the one wired board
 * answering on CS GPIO 10, i.e. board 1, so a count of 1 addressed board 0
 * and every frame went to a chip select with nothing on it.
 *
 * Absent boards are skipped silently instead of timing out on every gait
 * tick. Their servos keep reporting the last cached feedback (zeros at
 * start-up) and commands addressed to them are a no-op.
 *
 * NOTE: the "Triple_SMS_3IN1" board speaks the SAME 36-byte SPI frame as
 * the old 4IN1, so nothing else on the ESP side changes.
 * ---------------------------------------------------------------------- */
#define DB_BOARD_MASK 0x02

/* number of fitted boards (popcount of the mask) */
static inline int db_board_count(void){
    int n = 0;
    for (int b = 0; b < 4; b++) if ((DB_BOARD_MASK >> b) & 1) n++;
    return n;
}

/* ------------------------------------------------------------------------
 * POWER-ONLY MODE  (bench bring-up)
 *
 *   1 = the ESP32 does nothing but hold the servo power rail on (GPIO 8).
 *       The SPI bus is never initialised and no frames are sent, so the
 *       AT32's UART CLI has EXCLUSIVE control of the servos - your `pos`,
 *       `tor` and `trace` commands will not be overwritten by the gait task
 *       every 5 ms.
 *   0 = normal operation (ESP drives the servos over SPI).
 *
 * Flip this one line and re-flash to switch between bench and robot.
 *
 * While this is 1:
 *   - MOSI/MISO/CLK/CS stay high-Z, so nothing is injected into the AT32's
 *     SPI slave pins. PB12 (NSS) idles high on its own pull-up = deselected.
 *   - gait/web buttons still "work" but move nothing.
 *   - the web CLI's parameter get/set and live trace return failures - use
 *     the AT32 UART CLI for those instead.
 * ---------------------------------------------------------------------- */
#define DB_POWER_ONLY 0

/* true if board index 0..3 is physically present */
static inline bool db_board_present(int board){
#if DB_POWER_ONLY
    (void)board;
    return false;              /* nothing is reachable over SPI in this mode */
#else
    return board >= 0 && board < 4 && ((DB_BOARD_MASK >> board) & 1);
#endif
}

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
#else
    return logical;
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

/* Servo temperature in degrees Celsius, from the NTC next to each motor.
 * Comes back inside the normal feedback frame (the old `reserved1` field), so
 * it costs no extra SPI traffic - just call it after driver_board_sync_write()
 * or driver_board_poll(). Requires the 3IN1 board + matching AT32 firmware;
 * on the old 4IN1 boards there is no NTC and this reads garbage. */
float    driver_board_present_temperature(int ch);

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
    DB_LIVE_TEMPERATURE,        /* NTC servo temperature, degC  */
    DB_LIVE_COUNT
};

bool driver_board_get_live(int servo /*1..12*/, int live_id, float *out);

/* ---- bring-up scan ------------------------------------------------------
 * Probes all four chip selects with a harmless IDLE frame and reports the
 * raw 36 bytes each one returns on MISO, with a verdict. Pass a printf-style
 * sink (the CLI's cli_printf wrapper) or NULL to log via ESP_LOG.
 * Safe to call at any time - it commands IDLE mode with a zero current
 * limit, so no servo moves.                                              */
void driver_board_scan(void (*out)(const char *line));

/* ---- pin walk for multimeter tracing ------------------------------------
 * Drives SCK, MOSI and the four CS lines as plain GPIO, one at a time:
 * everything idles HIGH, then the pin under test is held LOW for hold_s
 * seconds (1..30, default 3) so a multimeter can catch it.
 *
 * DESTRUCTIVE: this detaches those pins from the SPI peripheral, so the
 * driver boards are unreachable afterwards. Reboot to restore.          */
void driver_board_pintest(void (*out)(const char *line), int hold_s);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BOARD_H */
