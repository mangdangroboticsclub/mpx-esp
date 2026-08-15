#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "driver/gpio.h"
#include "driver_board.h"   // SPI controller-driver-board backend (replaces SCServo)
#include "stanford_gait.h"  // Stanford Pupper trot gait port (forward walk)
#include "stanford_kinematics.h"  // exact BSP 2pro leg IK - Stanford walk uses this

/* The Stanford walk ALWAYS uses the exact mini_pupper_2pro_bsp inverse
 * kinematics (true geometry L2=60mm + 26mm abduction offset, from
 * stanford_kinematics.c). Every OTHER mode keeps the legacy planar
 * fRIK/fLIK/rRIK/rLIK. Nothing to toggle. */
#include "mqtt_client.h"
#include "mp2_calib.h"           // BF_STAND[]/BF_SIGN[] -- shared servo calibration
#include "mp2_backflip_data.h"   // optimized backflip keyframes (auto-generated)
#include "mp2_backflip2_data.h"  // hand-crafted backflip keyframes (backflip_edit.py)
#include "mp2_caltest_data.h"    // calibration test: lift one leg at a time
#include "hardcode_backflip_angle.h" // "Backflip 3": hand-taught SCS keyframes
#include "mp2_bf_v4.h"           // "Backflip v4": optimizer trajectory (mp2_backflip_v4.hdf5)
/* The "current flip" slot. Regenerated in place by tools/newflip.py, so a new
 * HDF5 trajectory never requires a code change here — same filename, same
 * CURRENT_FLIP_* names, every time. */
#include "current_flip.h"
#include "backflip20.h"          // Backflip20 button: linear-interp playback
#include "hdf5_traj1.h"          // HDF5 trajectory #1: delta-format, auto-generated

#define TAG "PUPPER"
#define PI 3.14159265358979f

/* ---- legacy "speed" -> current(mA) limit mapping (new driver board) ----
 * The driver board does POSITION control with a current/torque CAP; it has no
 * speed field. Feetech convention used by the gait code: goal_speed 0 == "max
 * speed". We translate every legacy speed value into a current limit:
 *   speed 0      -> CUR_MAX_MA   (full torque / snap)
 *   speed 1..ref -> CUR_MIN_MA .. CUR_MAX_MA  (gentler/slower = lower cap)
 * NOTE: this is a CAP, not a forced draw. A lightly loaded leg only sources the
 * current it needs to hold its position; the cap just limits peak/stall current.
 * Tune these three constants for your SCS0009 servos. */
#define CUR_MAX_MA   1500

static inline uint16_t speed_to_current_mA(uint16_t spd){ (void)spd; return CUR_MAX_MA; }

// ---- WiFi (station) + MQTT configuration ----
// Fill these in for your network / broker before flashing.
// ENABLE_MQTT 0 = MQTT fully off (no reconnect error spam in the serial
// CLI). Set to 1 when you have a broker running again.
#define ENABLE_MQTT      0
#define WIFI_SSID        "Mangdang"
#define WIFI_PASS        "mangdang"
#define MQTT_BROKER_URI  "mqtt://192.168.1.117:1883"
#define MQTT_CMD_TOPIC   "minipupper/cmd"
#define MQTT_STATE_TOPIC "minipupper/state"

static nvs_handle_t nvs;

static float offset[13] = {0};
static float L1 = 50, L2 = 56;

static int Ini=0, Step=0, Roll=0, Pitch=0, Stretch=0;
static int Advance=0, Back=0, Left=0, Right=0, TurnL=0, TurnR=0;
static int Twerk=0, Jump=0, JumpFwd=0, TestSpeed=0, Mate=0, Stanford=0;
static int Backflip=0;   // optimized backflip playback (bench demo, see gait loop)
static int Backflip20=0; // backflip20.h playback, linear interp (see backflip20_run)
static int sg_started=0;   // Stanford gait state initialised for this activation

// CLI mode: pauses the gait loop so the HTTP task has exclusive SPI access
// to the AT32 driver boards for parameter get/set (kp_position, kd_position,
// kp_current, kff_current, ...). Servos hold their last position.
static int  CliMode = 0;
static char cli_out[6144];   // last CLI command output, shown in the web UI

// Power-on behaviour (matches mini_pupper_2_bsp): hold the NEUTRAL pose
// (all servos centred = the Ini pose) until the user starts any motion.
// With the neutral-angle calibration (see ik_neutral_init) the Ini pose
// equals the IK stand at (x=0, z=NEUTRAL_Z), so every motion starts right
// from the startup pose - no repositioning move first.
static int started_once = 0;         // set on the first motion command
static float pose_x = 0, pose_z = 70;    // last neutral pose, IK coords (mm)

// ---- web joystick (mini_pupper_web_controller style) -------------------
// Left pad: forward/strafe, right pad: turn. Values are set by /js and
// consumed by the Stanford gait. Timeout -> stand (step in place).
// EXACT Mini Pupper 2 (Pro) Config.py limits (mangdangroboticsclub/
// mini_pupper_2_bsp, mini_pupper_2pro_bsp branch): max_x_velocity=0.20 m/s,
// max_y_velocity=0.20 m/s, max_yaw_rate=2 rad/s. Full stick == full speed,
// same scaling as JoystickInterface.py/HardwareInterface.py.
#define JOY_VX_MAX   200.0f   // mm/s forward
#define JOY_VY_MAX   200.0f   // mm/s strafe
#define JOY_WZ_MAX     2.0f   // rad/s yaw
#define JOY_TIMEOUT_MS 600
static volatile float    js_vx = 0, js_vy = 0, js_wz = 0;
static volatile uint32_t js_last_ms = 0;

static int period=80, height=70, upHeight=10, stride=10, tilt=10;
static int sgspeed=100;   // Stanford walk speed, mm/s (max 200 = reference full stick)

static uint16_t goal[13];
static uint16_t goal_speed[13];   // legacy "speed" units; converted to mA on flush

// Manual override for any servo. When manual_ovr[id] is set, the gait task
// holds a neutral stand but drives that servo to manual_ovr_deg[id] degrees.
// Indexed 1..12 (1-indexed, like goal[]).
static int       manual_ovr[13] = {0};      // non-zero = override active
static float     manual_ovr_deg[13] = {0};  // angle in degrees (0-270)

/* ---- teach / record / playback (hand-pose keyframes) ------------------
 * Teach mode relaxes the legs to a LOW-TORQUE follow so you can pose them by
 * hand; Record snapshots all 12 present angles into a keyframe; Verify steps
 * to one keyframe at low speed / normal torque so you can confirm it before
 * committing; Play runs the whole trace ONCE from the initial (Ini) pose.
 * Mirror is the optional optimizer: copy the RIGHT legs onto the LEFT (same
 * foot height, mirrored joint angles). The trace can be saved to flash (NVS)
 * so the recorded angles survive a reboot ("hardcoded"). */
#define MAX_FRAMES   128
static int teach_cur = 50;       /* teach-mode current cap (mA); lower = limper */
static uint16_t rec_frames[MAX_FRAMES][13];  /* [frame][servo 1..12] SCS 0..1023 */
static int rec_count   = 0;      /* number of recorded keyframes                */
static int Relax       = 0;      /* teach (hand-pose) mode                      */
static int Play        = 0;      /* play the whole trace once                   */
static int Goto        = 0;      /* move to one keyframe (verify)               */
static int goto_frame  = 0;      /* which keyframe Goto targets                 */
static int verify_idx  = 0;      /* Verify Prev/Next cursor                     */
static int HoldPose    = 0;      /* hold a fixed pose after play/verify         */
static uint16_t hold_frame[13];  /* the pose HoldPose holds                     */
static int GotoPose    = 0;      /* move to a typed-in SCS pose ('pose' cmd)    */
static uint16_t pose_target[13]; /* the 12 SCS values to move to                */
static volatile int rec_request = 0;      /* /rec sets this; the gait task captures (volatile: cross-core flag) */
static int play_ms     = 1000;   /* move time per pose (ms); low speed default  */
static int play_delay_ms = 0;    /* dwell/hold at each pose after the move (ms)  */
/* Optional PER-FRAME timing (used by Backflip 3). When use_frame_timing==1 the
 * Play loop uses frame_move_ms[f]/frame_delay_ms[f] instead of the globals
 * above, so every transition can have its own speed and pause. Taught traces
 * leave this 0 and keep using the global play_ms/play_delay_ms. */
static int frame_move_ms[MAX_FRAMES];    /* per-frame move time (ms)            */
static int frame_delay_ms[MAX_FRAMES];   /* per-frame dwell time (ms)           */
static int use_frame_timing = 0; /* 1 = use the per-frame arrays above          */
/* SLOW-MOTION SAFETY OVERRIDE.
 * The HDF5 trajectories carry per-frame move times of ~24 ms. Played at that
 * speed the servos slam pose-to-pose and can strip the gears, and it is far
 * too fast to see whether the motion is even correct. When SlowMo is set the
 * Play loop IGNORES frame_move_ms[]/frame_delay_ms[] and uses the global
 * play_ms/play_delay_ms instead, so the same frames run at whatever speed the
 * web UI is set to. Defaults ON — turn it off deliberately once the motion has
 * been verified frame by frame. */
static int SlowMo = 1;
/* How many values load_current_flip() had to clamp at a servo limit. Declared
 * up here because send_root() renders it long before load_current_flip() is
 * defined further down. Non-zero means the trajectory does not fit and should
 * be regenerated with a lower --scale. */
static int current_flip_clamped = 0;
static uint16_t cur_override_mA = 0;  /* 0 = normal cap; else force this cap    */

// Backflip-specific recording (exports REF + DELTA for hardcode_backflip_angle.h)
static uint16_t bf_ref[13] = {0};          /* reference frame (absolute SCS)    */
static uint16_t bf_frames[MAX_FRAMES][13]; /* all recorded frames (absolute SCS)*/
static int bf_count = 0;                   /* number of recorded frames         */
/* Per-frame move time for the taught frames, editable from the web UI. Kept
 * separate from frame_move_ms[] because that belongs to whatever trace is
 * currently loaded; this survives loading and re-loading the taught set. */
static uint16_t bf_move_ms[MAX_FRAMES];
static uint16_t bf_delay_ms[MAX_FRAMES];
static int bf_ref_idx = 0;                 /* which taught frame is the REF     */

static inline void servo_speed(int ch, uint16_t spd){
    goal_speed[ch] = spd;
}
static inline void servo_speed_all(uint16_t spd){
    for(int i=1;i<=12;i++) goal_speed[i] = spd;
}

static void servo_flush(void){
    static int64_t last_us = 0;
    while(esp_timer_get_time() - last_us < 5000){
        vTaskDelay(1);
    }
    last_us = esp_timer_get_time();

    uint16_t pos[12], cur[12];
    for(int i=0; i<12; i++){
        pos[i] = goal[i+1];
        // teach/hold modes force a fixed current cap via cur_override_mA;
        // otherwise use the legacy speed->current mapping (full torque).
        cur[i] = cur_override_mA ? cur_override_mA
                                 : speed_to_current_mA(goal_speed[i+1]);
    }
    driver_board_sync_write(pos, cur);
}

static inline uint32_t millis(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }

static void reset_all_modes(void){
    Ini=Step=Roll=Pitch=Stretch=0;
    Advance=Back=Left=Right=TurnL=TurnR=Twerk=Jump=JumpFwd=TestSpeed=Mate=Stanford=0; // <-- add TestSpeed here
    Backflip=0;
    Backflip20=0;
    for(int i=1;i<=12;i++) manual_ovr[i]=0;
    sg_started=0;
    Relax=Play=Goto=HoldPose=GotoPose=0;   // stop teach / playback modes too
    cur_override_mA=0;            // back to normal torque cap
}

// Toggle a motion flag the same way the web buttons do: pressing the
// active motion's button turns it off, pressing any other turns that
// one on (and everything else off).
static void toggle_motion(int *flag){
    started_once = 1;    // leave the power-on (Ini) hold
    if(*flag){ *flag=0; reset_all_modes(); }
    else     { reset_all_modes(); *flag=1; }
}

/* ---------------- web CLI for the AT32 driver boards -------------------
 * Same command style as the AT32 UART CLI, but with GLOBAL servo ids 1..12
 * (board = (id-1)/3 is addressed automatically over SPI):
 *   7 set kp_position 30     7 get kp_position     7 dump
 *   dump | save [id] | restore [id] | help                              */
static void urldecode(char *s){
    char *o = s;
    while(*s){
        if(*s=='+'){ *o++=' '; s++; }
        else if(*s=='%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])){
            char h[3]={s[1],s[2],0};
            *o++ = (char)strtol(h,NULL,16); s+=3;
        }else *o++ = *s++;
    }
    *o = 0;
}

static void cli_printf(const char *fmt, ...){
    size_t len = strlen(cli_out);
    if(len >= sizeof(cli_out)-2) return;
    va_list ap; va_start(ap, fmt);
    vsnprintf(cli_out+len, sizeof(cli_out)-len, fmt, ap);
    va_end(ap);
}

static void cli_dump_servo(int id){
    for(int p=0; p<DB_PARAM_COUNT; p++){
        float v;
        if(driver_board_get_param(id, p, &v))
            cli_printf("%2d set %-24s = %g\n", id, driver_board_param_name(p), v);
        else
            cli_printf("%2d     %-24s   <no reply>\n", id, driver_board_param_name(p));
    }
}

static void cli_list_params(void){
    cli_printf("params:\n");
    for(int p=0; p<DB_PARAM_COUNT; p++)
        cli_printf("  %s\n", driver_board_param_name(p));
}

static void cli_exec(char *cmd){
    cli_out[0]=0;
    // echo (sanitised for the <pre> block)
    for(char *p=cmd; *p; p++) if(*p=='<'||*p=='>') *p='.';
    cli_printf("> %s\n", cmd);
    for(char *p=cmd; *p; p++) if(*p=='=') *p=' ';   // allow "set kp = 30"

    char *sp=NULL;
    char *t0 = strtok_r(cmd, " \t", &sp);
    if(!t0){ cli_printf("empty command (try help)\n"); return; }

    if(!strcmp(t0,"help")){
        cli_printf("commands:\n"
                   "  <id 1-12> get <param>\n"
                   "  <id 1-12> set <param> <value>\n"
                   "  <id 1-12> dump\n"
                   "  <id 1-12> pos <deg 0-270> [current_mA]   (default 130mA)\n"
                   "  <id 1-12> tor <current_mA>   torque mode, +/- direction\n"
                   "  <id 1-12> stop               idle, motor off\n"
                   "  <id 1-12> fb                 present position + current\n"
                   "  dump              all 12 servos\n"
                   "  trace <id>        live position/current view (trace off = stop)\n"
                   "  sweep [id low high hold_ms cycles hz]  PID step test -> CSV\n"
                   "                    (serial only, needs 'cli on'; default: 2 100 200 800 3 50)\n"
                   "  swalk [secs hz vx id...]  Stanford-walk PID trace -> CSV\n"
                   "                    (serial only, needs 'cli on'; default: 6 50 <sgspeed> 2 3)\n"
                   "  sstretch [secs hz id...]  Stretch-bob PID trace -> CSV (all legs)\n"
                   "                    (serial only, needs 'cli on'; default: 6 50 2 3)\n"
                   "  sjump [hz reps id...]     in-place Jump PID trace -> CSV (all legs)\n"
                   "                    (serial only, needs 'cli on'; default: 50 1 2 3)\n"
                   "  offsets           print all 12 servo calibration offsets (serial only)\n"
                   "  save [id]         save board config to flash\n"
                   "  restore [id]      factory defaults (RAM, then save)\n"
                   "note: pos uses RAW AT32 angle, 135 = centre, no direction flip\n");
        cli_list_params();

    }else if(!strcmp(t0,"dump")){
        char *t1 = strtok_r(NULL," \t",&sp);
        if(t1){
            int id=atoi(t1);
            if(id>=1 && id<=12) cli_dump_servo(id);
            else cli_printf("bad servo id '%s'\n", t1);
        }else{
            for(int id=1; id<=12; id++){ cli_dump_servo(id); cli_printf("\n"); }
        }

    }else if(!strcmp(t0,"save")){
        char *t1 = strtok_r(NULL," \t",&sp);
        int board = -1;
        if(t1){ int id=atoi(t1); if(id>=1&&id<=12) board=(id-1)/3; }
        cli_printf(driver_board_save_config(board)
            ? "OK. saved to flash (%s)\n" : "save FAILED (%s)\n",
            board<0 ? "all boards" : "one board");

    }else if(!strcmp(t0,"trace")){
        // Live view: the browser polls /tracepoll and updates the console.
        char *t1 = strtok_r(NULL," \t",&sp);
        if(t1 && !strcmp(t1,"off")){
            cli_printf("TRACE OFF\n");
        }else{
            int id = t1 ? atoi(t1) : 0;
            if(id>=1 && id<=12)
                cli_printf("TRACE ON servo %d - live view in the blue box.\n"
                           "Other commands (pos/tor/set/...) keep working while\n"
                           "it runs. Stop with 'trace off'.\n", id);
            else cli_printf("usage: trace <id 1-12> | trace off\n");
        }

    }else if(!strcmp(t0,"restore") || !strcmp(t0,"factory_restore")){
        char *t1 = strtok_r(NULL," \t",&sp);
        int board = -1;
        if(t1){ int id=atoi(t1); if(id>=1&&id<=12) board=(id-1)/3; }
        cli_printf(driver_board_factory_restore(board)
            ? "OK. factory defaults loaded (RAM). Run 'save' to keep them.\n"
            : "restore FAILED\n");

    }else{
        int id = atoi(t0);
        if(id<1 || id>12){ cli_printf("unknown command '%s' (try help)\n", t0); return; }
        char *t1 = strtok_r(NULL," \t",&sp);
        if(!t1){ cli_printf("missing operator: get/set/dump/pos/tor/stop/fb (try help)\n"); return; }

        if(!strcmp(t1,"dump")){
            cli_dump_servo(id);

        }else if(!strcmp(t1,"pos")){
            char *t2 = strtok_r(NULL," \t",&sp);
            if(!t2){ cli_printf("usage: %d pos <deg 0-270> [current_mA]\n", id); return; }
            float deg = strtof(t2,NULL);
            char *t3 = strtok_r(NULL," \t",&sp);
            int cur = t3 ? atoi(t3) : 130;   /* same default as AT32 CLI */
            if(driver_board_direct(id, DB_MODE_POSITION, deg, (int16_t)cur))
                cli_printf("OK. %d pos %.1f deg, max %d mA\n", id, deg, cur);
            else cli_printf("pos FAILED\n");

        }else if(!strcmp(t1,"tor")){
            char *t2 = strtok_r(NULL," \t",&sp);
            if(!t2){ cli_printf("usage: %d tor <current_mA>\n", id); return; }
            int cur = atoi(t2);
            if(driver_board_direct(id, DB_MODE_TORQUE, 135, (int16_t)cur))
                cli_printf("OK. %d torque %d mA\n", id, cur);
            else cli_printf("tor FAILED\n");

        }else if(!strcmp(t1,"stop")){
            if(driver_board_direct(id, DB_MODE_IDLE, 135, 0))
                cli_printf("OK. %d idle (motor off)\n", id);
            else cli_printf("stop FAILED\n");

        }else if(!strcmp(t1,"fb")){
            cli_printf("%2d present: pos=%u (SCS 0-1023), cur=%d mA\n",
                       id, driver_board_present_position(id),
                       driver_board_present_current(id));

        }else if(!strcmp(t1,"get") || !strcmp(t1,"set")){
            char *t2 = strtok_r(NULL," \t",&sp);
            int p = t2 ? driver_board_param_id(t2) : -1;
            if(p<0){ cli_printf("unknown parameter '%s'\n", t2?t2:""); cli_list_params(); return; }

            if(!strcmp(t1,"get")){
                float v;
                if(driver_board_get_param(id,p,&v))
                    cli_printf("%2d set %s = %g\n", id, driver_board_param_name(p), v);
                else cli_printf("get FAILED (no reply from board)\n");
            }else{
                char *t3 = strtok_r(NULL," \t",&sp);
                if(!t3){ cli_printf("missing value\n"); return; }
                float v = strtof(t3,NULL), rb=0;
                if(driver_board_set_param(id,p,v) && driver_board_get_param(id,p,&rb))
                    cli_printf("%2d set %s = %g  (readback OK)\n"
                               "RAM only - run 'save %d' to keep after power off\n",
                               id, driver_board_param_name(p), rb, id);
                else cli_printf("set FAILED (no reply from board)\n");
            }
        }else cli_printf("unknown operator '%s' (get/set/dump/pos/tor/stop/fb)\n", t1);
    }
}

/* ---------------- serial CLI (idf.py monitor), no HTTP ------------------
 * Same commands as the web CLI, plus:
 *   cli on / cli off   toggle CLI mode (pause/resume the gait) from serial
 *   trace <id>         background live stream at 10 Hz; other commands keep
 *                      working while it runs. 'trace off' or 'q' stops it.
 * Runs as its own task; stdin is polled non-blocking (works on the UART
 * console and on USB-Serial-JTAG).                                        */
static int TraceId = 0;                 /* 0 = off, 1-12 = servo being traced */

static void trace_print_line(int id){
    static const char *mn[] = {"IDLE","POS ","TOR ","IK  "};
    float lv[DB_LIVE_COUNT];
    bool ok = true;
    /* temperature (the last id) is fetched separately below, so that a board
     * on older AT32 firmware still prints the rest of the trace */
    for(int i=0; i<DB_LIVE_TEMPERATURE_C && ok; i++)
        ok = driver_board_get_live(id, i, &lv[i]);
    if(ok){
        int m = (int)lv[DB_LIVE_MODE];
        float t;
        if(!driver_board_get_live(id, DB_LIVE_TEMPERATURE_C, &t) || t <= DB_TEMP_INVALID)
            t = driver_board_present_temperature(id);
        printf("pos s/n/e %6.1f/%6.1f/%5.1f deg | cur c/s/n/e %4.0f/%4.0f/%4.0f/%4.0f mA"
               " | duty %5.1f%% | adc %4.0f/%4.0f | %s | %4.1fC | loop %lu\n",
               lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
               lv[DB_LIVE_ERROR_POS_DEG],
               lv[DB_LIVE_MAX_CURRENT_MA], lv[DB_LIVE_SETPOINT_CUR_MA],
               lv[DB_LIVE_PRESENT_CUR_MA], lv[DB_LIVE_ERROR_CUR_MA],
               lv[DB_LIVE_PWM_DUTY]*100.0f,
               lv[DB_LIVE_POS_ADC], lv[DB_LIVE_CUR_ADC],
               (m>=0&&m<4)?mn[m]:"?", t, (unsigned long)lv[DB_LIVE_LOOP_COUNTER]);
    }else if(driver_board_poll(id)){
        printf("pos %4u SCS  cur %5d mA  (basic - old AT32 fw)\n",
               driver_board_present_position(id), driver_board_present_current(id));
    }else printf("SPI poll failed\n");
}

/* ---- automated step-response sweep for PID tuning (serial CLI) ----------
 * Steps ONE servo low<->high for N cycles and streams CSV over the console,
 * so you can capture it in `idf.py monitor` and plot it. Blocks until done
 * (runs in the console task). Requires 'cli on' so the gait doesn't fight
 * the SPI bus. Output is framed by #SWEEP_BEGIN / #SWEEP_END so a plot
 * script can pick the block straight out of the monitor log.              */
static void run_sweep(int id, float low, float high,
                      int hold_ms, int cycles, int hz){
    if(hz < 1)   hz = 1;
    if(hz > 200) hz = 200;
    int period_ms = 1000 / hz;
    const int cur = 900;                 /* max current cap during the test */

    printf("#SWEEP_BEGIN id=%d low=%.1f high=%.1f hold_ms=%d cycles=%d hz=%d\n",
           id, low, high, hold_ms, cycles, hz);
    printf("t_ms,cmd_deg,set_deg,now_deg,err_deg,cur_mA,duty_pct\n");

    /* settle at the low angle first (not logged) */
    driver_board_direct(id, DB_MODE_POSITION, low, (int16_t)cur);
    vTaskDelay(pdMS_TO_TICKS(600));

    uint32_t t0 = millis();
    float targets[2] = { high, low };
    for(int c = 0; c < cycles; c++){
        for(int leg = 0; leg < 2; leg++){
            float tgt = targets[leg];
            driver_board_direct(id, DB_MODE_POSITION, tgt, (int16_t)cur);
            uint32_t seg_end = millis() + (uint32_t)hold_ms;
            while((int32_t)(seg_end - millis()) > 0){
                uint32_t ls = millis();
                float lv[DB_LIVE_COUNT];
                bool ok = true;
                /* only the values this CSV prints - do NOT add temperature
                 * here, it is one more SPI round-trip per sample and the
                 * sweep runs at up to 200 Hz */
                for(int i=0; i<DB_LIVE_TEMPERATURE_C && ok; i++)
                    ok = driver_board_get_live(id, i, &lv[i]);
                if(ok){
                    printf("%lu,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f\n",
                        (unsigned long)(millis()-t0), tgt,
                        lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
                        lv[DB_LIVE_ERROR_POS_DEG], lv[DB_LIVE_PRESENT_CUR_MA],
                        lv[DB_LIVE_PWM_DUTY]*100.0f);
                }else if(driver_board_poll(id)){
                    printf("%lu,%.1f,,%.1f,,%d,\n",
                        (unsigned long)(millis()-t0), tgt,
                        (float)driver_board_present_position(id)*270.0f/1024.0f,
                        driver_board_present_current(id));
                }
                uint32_t el = millis() - ls;
                if((int)el < period_ms) vTaskDelay(pdMS_TO_TICKS(period_ms - (int)el));
            }
        }
    }
    /* leave the servo idle so it isn't straining after the test */
    driver_board_direct(id, DB_MODE_IDLE, low, 0);
    printf("#SWEEP_END\n");
}

/* defined later (needs the IK helpers + NEUTRAL_Z); runs the Stanford gait
 * while streaming per-servo tracking CSV so you can PID-tune during the walk */
static void run_swalk(const int *ids, int nids, int secs, int hz, float vx);

/* teach/playback optimizer: mirror the recorded RIGHT legs onto the LEFT.
 * Defined after the IK helpers; forward-declared here for the serial CLI. */
static void mirror_RL(void);

/* likewise: runs the all-leg Stretch bob (up/down) while streaming per-servo
 * tracking CSV, so you can PID-tune the vertical stretch motion. */
static void run_sstretch(const int *ids, int nids, int secs, int hz);

/* likewise: runs the in-place Jump (crouch/push/tuck/land) `reps` times while
 * streaming per-servo tracking CSV, so you can PID-tune the jump. */
static void run_sjump(const int *ids, int nids, int hz, int reps);

static void nvs_put_float(const char*k, float v);   /* fwd: used by setcal/calhere */

static void serial_handle_line(char *line){
    if(!strncmp(line,"sweep",5)){
        int sid=2, hold=800, cyc=3, hz=50;
        float lo=100, hi=200;
        sscanf(line,"sweep %d %f %f %d %d %d",&sid,&lo,&hi,&hold,&cyc,&hz);
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        if(sid<1 || sid>12){ printf("bad servo id\n"); return; }
        run_sweep(sid, lo, hi, hold, cyc, hz);
        return;
    }
    if(!strncmp(line,"swalk",5)){
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        int secs=6, hz=50, ids[12], nids=0;
        float vx=(float)sgspeed;
        char buf[128]; strncpy(buf,line,sizeof buf-1); buf[sizeof buf-1]=0;
        char *sp; strtok_r(buf," \t",&sp);          /* skip "swalk" */
        char *tok;
        if((tok=strtok_r(NULL," \t",&sp))) secs=atoi(tok);
        if((tok=strtok_r(NULL," \t",&sp))) hz=atoi(tok);
        if((tok=strtok_r(NULL," \t",&sp))) vx=atof(tok);
        while((tok=strtok_r(NULL," \t",&sp)) && nids<12){
            int v=atoi(tok); if(v>=1 && v<=12) ids[nids++]=v;
        }
        if(nids==0){ ids[0]=2; ids[1]=3; nids=2; }
        run_swalk(ids, nids, secs, hz, vx);
        return;
    }
    if(!strncmp(line,"sstretch",8)){
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        int secs=6, hz=50, ids[12], nids=0;
        char buf[128]; strncpy(buf,line,sizeof buf-1); buf[sizeof buf-1]=0;
        char *sp; strtok_r(buf," \t",&sp);          /* skip "sstretch" */
        char *tok;
        if((tok=strtok_r(NULL," \t",&sp))) secs=atoi(tok);
        if((tok=strtok_r(NULL," \t",&sp))) hz=atoi(tok);
        while((tok=strtok_r(NULL," \t",&sp)) && nids<12){
            int v=atoi(tok); if(v>=1 && v<=12) ids[nids++]=v;
        }
        if(nids==0){ ids[0]=2; ids[1]=3; nids=2; }
        run_sstretch(ids, nids, secs, hz);
        return;
    }
    if(!strncmp(line,"sjump",5)){
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        int hz=50, reps=1, ids[12], nids=0;
        char buf[128]; strncpy(buf,line,sizeof buf-1); buf[sizeof buf-1]=0;
        char *sp; strtok_r(buf," \t",&sp);          /* skip "sjump" */
        char *tok;
        if((tok=strtok_r(NULL," \t",&sp))) hz=atoi(tok);
        if((tok=strtok_r(NULL," \t",&sp))) reps=atoi(tok);
        while((tok=strtok_r(NULL," \t",&sp)) && nids<12){
            int v=atoi(tok); if(v>=1 && v<=12) ids[nids++]=v;
        }
        if(nids==0){ ids[0]=2; ids[1]=3; nids=2; }
        run_sjump(ids, nids, hz, reps);
        return;
    }
    // ---- teach / record / playback (serial equivalents of the web UI) ----
    if(!strcmp(line,"teach")){
        started_once=1;
        if(Relax){ Relax=0; reset_all_modes(); printf("teach OFF (back to stand)\n"); }
        else     { reset_all_modes(); Relax=1;  printf("teach ON - pose the legs by hand, then 'rec'\n"); }
        return;
    }
    if(!strcmp(line,"rec")){
        if(!Relax) printf("run 'teach' first\n");
        else if(rec_count>=MAX_FRAMES) printf("trace full (%d frames)\n", MAX_FRAMES);
        else { rec_request=1; printf("recording frame %d\n", rec_count+1); }
        return;
    }
    if(!strcmp(line,"recclear")){ rec_count=0; verify_idx=0; use_frame_timing=0; printf("trace cleared\n"); return; }
    if(!strcmp(line,"slowmo on")  || !strcmp(line,"slowmo")){
        SlowMo=1; printf("slow motion ON: Play ignores per-frame timing, uses "
                         "pspeed=%d ms / pdelay=%d ms\n", play_ms, play_delay_ms); return; }
    if(!strcmp(line,"slowmo off")){
        SlowMo=0; printf("slow motion OFF: Play uses each trace's own per-frame "
                         "timing (HDF5 traces are ~24 ms/frame)\n"); return; }
    // Report which frames contain servo values pinned at a limit by the
    // converter's clamp. Those joints stop following the trajectory.
    if(!strcmp(line,"framecheck")){
        if(rec_count==0){ printf("no frames loaded\n"); return; }
        int total=0;
        for(int f=0; f<rec_count; f++){
            int n=0;
            for(int id=1; id<=12; id++)
                if(rec_frames[f][id]<=0 || rec_frames[f][id]>=1023) n++;
            if(n){ printf("  frame %2d: %d joint(s) pinned at a limit\n", f, n); total+=n; }
        }
        printf("framecheck: %d pinned value(s) across %d frames%s\n", total, rec_count,
               total? "  -- DO NOT run at full speed" : "  -- clean");
        return;
    }
    if(!strcmp(line,"recdel")){ if(rec_count>0) rec_count--; printf("%d frames left\n", rec_count); return; }
    if(!strcmp(line,"recdump")){
        /* Dump the taught keyframes as CSV (SCS 0..1023) so a PC can replay them
         * in Isaac. Capture with: python -m serial.tools.miniterm ... or your logger. */
        printf("kf,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12\n");
        for(int f=0; f<rec_count; f++){
            printf("%d", f);
            for(int i=1;i<=12;i++) printf(",%u", rec_frames[f][i]);
            printf("\n");
        }
        printf("(%d frames)\n", rec_count);
        return;
    }
    // ---- Backflip teach/rec/recdump (exports REF+DELTA for hardcode_backflip_angle.h) ----
    if(!strcmp(line,"teach_backflip")){
        started_once=1;
        if(Relax){ Relax=0; reset_all_modes(); printf("teach OFF (back to stand)\n"); }
        else     { reset_all_modes(); Relax=1;  printf("teach ON - pose legs by hand, 'rec_bf' to capture\n"); }
        return;
    }
    if(!strcmp(line,"rec_bf")){
        if(!Relax){ printf("run 'teach_backflip' first\n"); return; }
        if(bf_count >= MAX_FRAMES){ printf("max %d frames reached\n", MAX_FRAMES); return; }
        for(int i=1; i<=12; i++){
            int cmd = 1023 - (int)driver_board_present_position(i);
            if(cmd<0) cmd=0; 
            if(cmd>1023) cmd=1023;
            if(bf_count == 0) bf_ref[i] = (uint16_t)cmd;
            bf_frames[bf_count][i] = (uint16_t)cmd;
        }
        bf_count++;
        printf("rec_bf: frame %d saved (%s)\n", bf_count, bf_count==1 ? "REFERENCE" : "delta from REF");
        return;
    }
    if(!strcmp(line,"recdel_bf")){
        if(bf_count>0) bf_count--;
        printf("recdel_bf: %d frames left\n", bf_count);
        return;
    }
    if(!strcmp(line,"recclear_bf")){
        bf_count=0;
        printf("recclear_bf: cleared\n");
        return;
    }
    if(!strcmp(line,"recdump_bf")){
        if(bf_count<1){ printf("no frames recorded. Use teach_backflip + rec_bf first.\n"); return; }
        printf("// === paste below into hardcode_backflip_angle.h ===\n");
        printf("#define BF3_FRAMES %d\n\n", bf_count);
        printf("// ---- REFERENCE POSE (frame 0) ----\n");
        printf("static const uint16_t BF3_REF[13] = {\n");
        printf("    /* idx  0     1     2     3     4     5     6     7     8     9    10    11    12 */\n");
        /* The leading 0 is index [0], unused (1-based servo indexing). It used
         * to be omitted, which left only 12 initialisers for a [13] array and
         * shifted every servo down one slot. */
        printf("             0,");
        for(int i=1;i<=12;i++) printf(" %4u%s", bf_ref[i], i<12?",":"");
        printf("\n};\n\n");
        if(bf_count > 1){
            printf("// ---- DELTA FRAMES (frames 1..%d) ----\n", bf_count-1);
            printf("static const int16_t BF3_DELTA[%d][13] = {\n", bf_count-1);
            for(int f=1; f<bf_count; f++){
                printf("    {0");
                for(int i=1;i<=12;i++){
                    int d = (int)bf_frames[f][i] - (int)bf_ref[i];
                    printf(", %5d", d);
                }
                printf("},  /* frame %d */\n", f);
            }
            printf("};\n");
        }
        printf("// === end of backflip data ===\n");
        return;
    }
    // Set the STAND CALIBRATION from a recdump neutral row: 12 SCS values.
    //   setcal 57,634,649,50,400,551,50,589,534,50,486,495
    // Stores offset[id] = (scs-511)*0.263 for all 12 servos and saves to NVS,
    // so every motion (walk, backflip, cal-test) sits on this exact stance.
    if(!strncmp(line,"setcal",6) && (line[6]==' ' || line[6]==',' || line[6]=='\t')){
        char tmp[160]; strncpy(tmp,line+7,sizeof tmp-1); tmp[sizeof tmp-1]=0;
        int v[13]; int n=0; char *sp3=NULL;
        for(char *tk=strtok_r(tmp," ,\t",&sp3); tk && n<12; tk=strtok_r(NULL," ,\t",&sp3)){
            int s=atoi(tk);
            if(s<0) s=0;
            if(s>1023) s=1023;
            v[++n]=s;
        }
        if(n!=12){ printf("setcal: need 12 SCS values (got %d)\n", n); return; }
        printf("setcal: stand calibration from neutral row ->\n");
        for(int i=1;i<=12;i++){
            offset[i] = (v[i] - 511) * 0.263f;
            char k[12]; snprintf(k,sizeof k,"offset%d",i);
            nvs_put_float(k, offset[i]);
            printf("  id %2d  scs %4d  offset %+7.2f\n", i, v[i], offset[i]);
        }
        printf("saved to NVS. Press Ini (or reboot) to stand on the new calibration.\n");
        return;
    }
    // Capture the CURRENT servo positions as the stand calibration. Pose the
    // robot in its normal stance first (e.g. via teach), then run 'calhere'.
    if(!strcmp(line,"calhere")){
        printf("calhere: capturing present positions as the stand ->\n");
        for(int i=1;i<=12;i++){
            int scs = (int)driver_board_present_position(i);
            offset[i] = (scs - 511) * 0.263f;
            char k[12]; snprintf(k,sizeof k,"offset%d",i);
            nvs_put_float(k, offset[i]);
            printf("  id %2d  scs %4d  offset %+7.2f\n", i, scs, offset[i]);
        }
        printf("saved to NVS.\n");
        return;
    }
    // Move to a single pose typed straight from a recdump line: 12 SCS values
    // (0..1023), comma OR space separated. e.g.
    //   pose 56,184,528,50,851,560,49,632,509,48,408,552
    // The robot eases into that pose at the current play speed and holds it.
    if(!strncmp(line,"pose",4) && (line[4]==' ' || line[4]==',' || line[4]=='\t')){
        char tmp[160]; strncpy(tmp,line+5,sizeof tmp-1); tmp[sizeof tmp-1]=0;
        int v[13]; int n=0; char *sp2=NULL;
        for(char *tk=strtok_r(tmp," ,\t",&sp2); tk && n<12; tk=strtok_r(NULL," ,\t",&sp2)){
            int s=atoi(tk); if(s<0)s=0; if(s>1023)s=1023; v[++n]=s;
        }
        if(n!=12){ printf("pose: need 12 SCS values (got %d)\n", n); return; }
        reset_all_modes();
        for(int i=1;i<=12;i++) pose_target[i]=(uint16_t)v[i];
        started_once=1; GotoPose=1;
        printf("pose: moving to %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
               v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11],v[12]);
        return;
    }

    // ---- pose_bf: move to a pose specified as DELTAS from BF3_REF ----------
    // Usage: pose_bf d1,d2,...,d12  (12 signed delta values)
    // Computes absolute SCS = BF3_REF[id] + delta, clamped to 0..1023,
    // then eases into that pose like the normal 'pose' command.
    if(!strncmp(line,"pose_bf",7) && (line[7]==' ' || line[7]==',' || line[7]=='\t' || line[7]=='\0')){
        if(line[7]=='\0'){ printf("usage: pose_bf d1,d2,...,d12  (12 signed delta values)\n"); return; }
        char tmp[160]; strncpy(tmp,line+8,sizeof tmp-1); tmp[sizeof tmp-1]=0;
        /* Reference to apply the deltas to. If frames have been taught this
         * session (rec_bf), bf_ref[] IS the new start stance — use it, so the
         * deltas straight out of `recdump_bf` land on the pose you just taught
         * instead of the stale flashed BF3_REF[]. */
        const int use_live_ref = (bf_count > 0);
        int v[13]; int n=0; char *sp_bf=NULL;
        for(char *tk=strtok_r(tmp," ,\t",&sp_bf); tk && n<12; tk=strtok_r(NULL," ,\t",&sp_bf)){
            int delta = atoi(tk);
            /* n is still 0-based HERE (the ++n below happens after this read),
             * so servo id = n+1. Reading BF3_REF[n] shifted every joint onto
             * its neighbour's reference: servo 1 got BF3_REF[0] (the unused 0),
             * servo 2 got servo 1's value, etc. */
            int id  = n + 1;
            int ref = use_live_ref ? (int)bf_ref[id] : (int)BF3_REF[id];
            int scs = ref + delta;
            if(scs<0) scs=0;
            if(scs>1023) scs=1023;
            v[++n] = scs;
        }
        if(n!=12){ printf("pose_bf: need 12 delta values (got %d)\n", n); return; }
        reset_all_modes();
        for(int i=1;i<=12;i++) pose_target[i]=(uint16_t)v[i];
        started_once=1; GotoPose=1;
        printf("pose_bf: ref=%s -> SCS %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
               use_live_ref ? "bf_ref (just taught)" : "BF3_REF (flashed)",
               v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11],v[12]);
        return;
    }

    if(!strcmp(line,"mirror")){ mirror_RL(); printf("mirrored R->L on %d frames\n", rec_count); return; }
    if(!strcmp(line,"play")){
        if(rec_count>0){ reset_all_modes(); started_once=1; Play=1; printf("playing %d frames\n", rec_count); }
        else printf("no frames recorded\n");
        return;
    }
    int vf;
    if(sscanf(line,"verify %d",&vf)==1){
        if(rec_count<=0){ printf("no frames\n"); return; }
        if(vf<0) vf=0;
        if(vf>=rec_count) vf=rec_count-1;
        verify_idx=vf; reset_all_modes(); started_once=1; goto_frame=vf; Goto=1;
        printf("verify frame %d\n", vf);
        return;
    }
    // ---- backflip frame inspection: load the 28 optimized poses into the trace
    // buffer as SCS, then use Verify </> / 'verify N' / 'play' to step them one
    // by one. Same math the backflip playback uses (BF_SIGN/BF_STAND + offset).
    if(!strcmp(line,"bfload")){
        reset_all_modes();
        int nf = BF_FRAMES < MAX_FRAMES ? BF_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++){
                float ang = BF_SIGN[id-1]*(BF_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
                int sig = 511 + (int)(ang / 0.263f);
                if(sig<0) sig=0;
                if(sig>1023) sig=1023;
                rec_frames[f][id] = (uint16_t)sig;
            }
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 0;
        printf("bfload: %d backflip frames -> trace. Use Verify </>, 'verify N', or 'play'.\n", nf);
        return;
    }
    int bff;
    if(sscanf(line,"bfgoto %d",&bff)==1){
        // (re)load the backflip poses, then hold at frame bff for inspection.
        int nf = BF_FRAMES < MAX_FRAMES ? BF_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++)
            for(int id=1; id<=12; id++){
                float ang = BF_SIGN[id-1]*(BF_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
                int sig = 511 + (int)(ang / 0.263f);
                if(sig<0) sig=0;
                if(sig>1023) sig=1023;
                rec_frames[f][id] = (uint16_t)sig;
            }
        rec_count = nf;
        if(bff<0) bff=0;
        if(bff>=nf) bff=nf-1;
        verify_idx=bff; reset_all_modes(); started_once=1; goto_frame=bff; Goto=1;
        printf("bfgoto: holding backflip frame %d / %d\n", bff, nf);
        return;
    }
    // ---- backflip #2 (hand-crafted, fewer frames) -> trace for Verify / play.
    if(!strcmp(line,"bfload2")){
        reset_all_modes();
        int nf = BF2_FRAMES < MAX_FRAMES ? BF2_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++){
                float ang = BF_SIGN[id-1]*(BF2_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
                int sig = 511 + (int)(ang / 0.263f);
                if(sig<0) sig=0;
                if(sig>1023) sig=1023;
                rec_frames[f][id] = (uint16_t)sig;
            }
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 0;
        printf("bfload2: %d hand-crafted frames -> trace. Use Verify </>, 'verify N', or 'play'.\n", nf);
        return;
    }
    // ---- CALIBRATION TEST: lift one leg at a time -> trace for Verify / play.
    if(!strcmp(line,"caltest")){
        reset_all_modes();
        int nf = CAL_FRAMES < MAX_FRAMES ? CAL_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++){
                float ang = BF_SIGN[id-1]*(CAL_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
                int sig = 511 + (int)(ang / 0.263f);
                if(sig<0) sig=0;
                if(sig>1023) sig=1023;
                rec_frames[f][id] = (uint16_t)sig;
            }
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 0;
        printf("caltest: %d frames -> trace (lifts FL,FR,BL,BR one at a time). "
               "Use Verify </> or 'play'.\n", nf);
        return;
    }
    // ---- "Backflip 3": hand-taught SCS keyframes (hardcode_backflip_angle.h).
    // Values are already raw command SCS, so copy them straight in (no remap).
    if(!strcmp(line,"bfload3")){
        reset_all_modes();
        int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++)
                rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
            frame_move_ms[f]  = BF3_MOVE_MS[f];   // per-frame speed
            frame_delay_ms[f] = BF3_DELAY_MS[f];  // per-frame dwell
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 1;
        printf("bfload3: %d backflip-3 frames -> trace (per-frame timing). "
               "Use Verify </>, 'verify N', or 'play'.\n", nf);
        return;
    }
    // One-shot: load Backflip 3 and play it immediately using its own
    // per-frame move times and delays (from hardcode_backflip_angle.h).
    if(!strcmp(line,"bf3")){
        reset_all_modes();
        int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++)
                rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
            frame_move_ms[f]  = BF3_MOVE_MS[f];
            frame_delay_ms[f] = BF3_DELAY_MS[f];
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 1;
        started_once=1; Play=1;
        printf("backflip 3: playing %d frames with per-frame timing\n", nf);
        return;
    }
    // ---- "Backflip 4": same data source as Backflip 3 (hardcode_backflip_angle.h),
    // but a separate command so you can maintain two variants.
    if(!strcmp(line,"bfload4")){
        reset_all_modes();
        int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++)
                rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
            frame_move_ms[f]  = BF3_MOVE_MS[f];
            frame_delay_ms[f] = BF3_DELAY_MS[f];
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 1;
        printf("bfload4: %d backflip-4 frames -> trace (per-frame timing). "
               "Use Verify </>, 'verify N', or 'play'.\n", nf);
        return;
    }
    if(!strcmp(line,"bf4")){
        reset_all_modes();
        int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
        for(int f=0; f<nf; f++){
            for(int id=1; id<=12; id++)
                rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
            frame_move_ms[f]  = BF3_MOVE_MS[f];
            frame_delay_ms[f] = BF3_DELAY_MS[f];
        }
        rec_count = nf; verify_idx = 0; use_frame_timing = 1;
        started_once=1; Play=1;
        printf("backflip 4: playing %d frames with per-frame timing\n", nf);
        return;
    }
    int psp;
    if(sscanf(line,"pspeed %d",&psp)==1){
        if(psp<100)  psp=100;
        if(psp>3000) psp=3000;
        play_ms=psp;
        nvs_set_i32(nvs,"play_ms",play_ms); nvs_commit(nvs);
        printf("play speed = %d ms/pose (lower = faster)\n", play_ms);
        return;
    }
    int pdl;
    if(sscanf(line,"pdelay %d",&pdl)==1){
        if(pdl<0)    pdl=0;
        if(pdl>5000) pdl=5000;
        play_delay_ms=pdl;
        nvs_set_i32(nvs,"play_dly",play_delay_ms); nvs_commit(nvs);
        printf("play delay = %d ms/pose (dwell/hold at each pose)\n", play_delay_ms);
        return;
    }
    if(!strcmp(line,"recsave")){
        nvs_set_i32(nvs,"rec_cnt",rec_count);
        nvs_set_blob(nvs,"rec_fr",rec_frames,sizeof rec_frames);
        nvs_commit(nvs);
        printf("saved %d frames to flash\n", rec_count);
        return;
    }
    if(!strcmp(line,"cli on")){
        if(!CliMode){ reset_all_modes(); CliMode=1; vTaskDelay(pdMS_TO_TICKS(50)); }
        printf("CLI mode ON (gait paused, servos hold position)\n");
        return;
    }
    if(!strcmp(line,"cli off")){
        CliMode=0;
        printf("CLI mode OFF (gait resumed)\n");
        return;
    }
    if(!strcmp(line,"offsets")){
        static const char *nm[13] = {"",
            "FR hip","FR thigh","FR calf",  "FL hip","FL thigh","FL calf",
            "RR hip","RR thigh","RR calf",  "RL hip","RL thigh","RL calf"};
        printf("servo offsets (deg), added on top of the IK angle:\n");
        for(int i=1;i<=12;i++)
            printf("  id %2d  %-8s  %+.1f\n", i, nm[i], offset[i]);
        return;
    }
    if(!strcmp(line,"trace off")){
        if(TraceId){ TraceId = 0; printf("trace stopped\n"); }
        else printf("trace is not running\n");
        return;
    }
    int tid;
    if(sscanf(line,"trace %d",&tid)==1){
        if(tid<1 || tid>12){ printf("bad servo id\n"); return; }
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        TraceId = tid;
        printf("TRACE ON servo %d at 10 Hz - commands still work while it runs.\n"
               "stop with 'trace off' or press 'q' on an empty line\n", tid);
        return;
    }
    if(!CliMode && strcmp(line,"help")!=0)
        printf("note: CLI mode is OFF - run 'cli on' first, or SPI replies may garble\n");
    cli_exec(line);
    fputs(cli_out, stdout);
}

static void console_task(void *arg){
    (void)arg;
    setvbuf(stdin, NULL, _IONBF, 0);
    char line[128];
    int pos = 0;
    uint32_t last_trace_ms = 0;
    printf("\nPupper serial CLI ready. 'cli on' to pause gait, 'help' for commands.\n> ");
    fflush(stdout);
    for(;;){
        int c = getchar();
        if(c == EOF){
            if(TraceId){
                uint32_t now = millis();
                if(now - last_trace_ms >= 100){   /* 10 Hz live stream */
                    last_trace_ms = now;
                    trace_print_line(TraceId);
                    fflush(stdout);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if(c=='q' && pos==0 && TraceId){          /* quick-stop the trace */
            TraceId = 0;
            printf("trace stopped\n> "); fflush(stdout);
            continue;
        }
        if(c=='\r' || c=='\n'){
            printf("\n");
            if(pos > 0){ line[pos]=0; pos=0; serial_handle_line(line); }
            printf("> "); fflush(stdout);
        }else if(c==8 || c==127){                 /* backspace */
            if(pos>0){ pos--; printf("\b \b"); fflush(stdout); }
        }else if(pos < (int)sizeof(line)-1 && c>=32 && c<127){
            line[pos++] = (char)c;
            putchar(c); fflush(stdout);           /* echo */
        }
    }
}

// Command name -> flag table (used by the MQTT command handler).
#if ENABLE_MQTT
typedef struct { const char *name; int *flag; } motion_cmd_t;
static const motion_cmd_t motion_cmds[] = {
    {"ini",       &Ini},      {"step",      &Step},     {"roll",   &Roll},
    {"pitch",     &Pitch},    {"stretch",   &Stretch},  {"advance",&Advance},
    {"back",      &Back},     {"left",      &Left},     {"right",  &Right},
    {"turnl",     &TurnL},    {"turnr",     &TurnR},    {"twerk",  &Twerk},
    {"jump",      &Jump},     {"jumpfwd",   &JumpFwd},  {"testspeed",&TestSpeed},
    {"mate",      &Mate},     {"stanford",  &Stanford},
    {"backflip",  &Backflip},
};
#define MOTION_CMD_COUNT (sizeof(motion_cmds)/sizeof(motion_cmds[0]))
#endif /* ENABLE_MQTT */

static void nvs_put_float(const char*k, float v){
    nvs_set_blob(nvs, k, &v, sizeof(float)); nvs_commit(nvs);
}
static float nvs_get_float(const char*k, float def){
    float v=def; size_t sz=sizeof(float);
    if(nvs_get_blob(nvs,k,&v,&sz)!=ESP_OK) v=def;
    return v;
}
static void nvs_put_int(const char*k, int v){
    nvs_set_i32(nvs,k,v); nvs_commit(nvs);
}

static void servo_write(int ch, float ang){
    int sig = 511 + (int)(ang / 0.263f);
    if(sig<0) sig=0;
    if(sig>1023) sig=1023;
    goal[ch] = (uint16_t)sig;
}

/* ---- teach / record / playback helpers -------------------------------- */

/* Fill a keyframe with the INITIAL (Ini) pose: every servo centred + its
 * calibration offset - the same pose the Ini button and power-on hold use.
 * Playback starts here so "all start from initial position". */
static void fill_ini_frame(uint16_t fr[13]){
    for(int i=1;i<=12;i++){
        int sig = 511 + (int)(offset[i]/0.263f);
        if(sig<0) sig=0;
        if(sig>1023) sig=1023;
        fr[i]=(uint16_t)sig;
    }
}

/* Smoothstep-interpolate goal[] from its current value to target[] over
 * move_ms milliseconds, flushing continuously at NORMAL torque. Aborts early
 * if *active is cleared (e.g. the user pressed another button). This is what
 * gives the adjustable, low "speed" - the servos themselves run position
 * control, so we pace the setpoint by hand. */
static void interp_to(const uint16_t target[13], int move_ms, volatile int *active){
    if(move_ms < 1) move_ms = 1;
    uint16_t start[13];
    for(int i=1;i<=12;i++) start[i]=goal[i];
    uint32_t t0=millis(), tim;
    while((tim=millis()-t0) < (uint32_t)move_ms){
        if(active && !*active) break;
        float f = (float)tim/(float)move_ms;       /* 0..1 */
        float s = f*f*(3.0f-2.0f*f);               /* smoothstep ease in/out */
        for(int i=1;i<=12;i++)
            goal[i]=(uint16_t)((int)start[i] +
                    (int)(((int)target[i]-(int)start[i])*s));
        servo_flush();
    }
    for(int i=1;i<=12;i++) goal[i]=target[i];
    servo_flush();
}

/* Hold goal[] steady (NORMAL torque) for ms milliseconds, flushing so the
 * servos keep their setpoint. Aborts early if *active is cleared. Used to
 * dwell at each pose during playback (the tunable "delay"). */
static void dwell_ms(int ms, volatile int *active){
    if(ms <= 0) return;
    uint32_t t0=millis();
    while((int)(millis()-t0) < ms){
        if(active && !*active) break;
        servo_flush();
        vTaskDelay(1);
    }
}

/* OPTIMIZER: mirror the RIGHT legs onto the LEFT in every recorded frame, so
 * you only have to hand-pose one side and both feet end up at the same
 * height. FR(1,2,3)->FL(4,5,6), RR(7,8,9)->RL(10,11,12). The hip copies
 * straight across; the thigh and calf FLIP about the servo centre
 * (1022 - scs), matching the fRIK/fLIK sign convention (left thigh = -right
 * thigh, left calf = -right calf). So the joint ANGLES differ but the foot
 * heights match - exactly the "not the same angle but the height is okay"
 * behaviour. */
static void mirror_RL(void){
    for(int f=0; f<rec_count; f++){
        uint16_t *F = rec_frames[f];
        int t5 =1022-(int)F[2], t6 =1022-(int)F[3];
        int t11=1022-(int)F[8], t12=1022-(int)F[9];
        #define CLMP(x) ((x)<0?0:((x)>1023?1023:(x)))
        F[4] =F[1];               F[5] =(uint16_t)CLMP(t5);  F[6] =(uint16_t)CLMP(t6);
        F[10]=F[7];               F[11]=(uint16_t)CLMP(t11); F[12]=(uint16_t)CLMP(t12);
        #undef CLMP
    }
}

/* The board-variant servo swap (SERVO_BOARD) now lives in driver_board.h and
 * is applied at the driver layer (db_phys), so it covers the walk, calibration,
 * the `pos` command, sweep/swalk and feedback consistently. The IK below just
 * uses plain logical servo ids 1..12. */

/* ---- neutral-angle calibration (like NEUTRAL_ANGLE_DEGREES in the BSP) --
 * On this robot the physical standing pose is ALL SERVOS CENTRED (the Ini
 * pose). The plain IK, however, returns big absolute angles (~44..52 deg)
 * for the model's standing pose, so starting any motion used to reposition
 * the legs first. Fix: command IK angles RELATIVE to the IK angles of the
 * neutral stand (x=0, z=NEUTRAL_Z). Then fRIK(0,0,NEUTRAL_Z) == centred
 * servos == Ini pose, and every gait starts right from the startup pose. */
#define NEUTRAL_Z 70.0f          /* stand height that maps to servo centre */
static float th1_neutral = 0, th2_neutral = 0;   /* radians */

static void ik_neutral_init(void){
    float ld = NEUTRAL_Z;        /* x=0 -> phi=0, ld=z */
    th1_neutral = -acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    th2_neutral = asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2)) - th1_neutral;
}
/* Stride direction. After the servo cables were re-seated the legs swing the
 * opposite way, so "Advance" walked backwards. Negating x at the IK boundary
 * flips forward/backward for EVERY caller (walk, teleop, turns, stanford gait)
 * in one place, and does NOT touch the stance geometry -- the robot stands
 * exactly as before. Set to +1.0f to restore the original direction. */
#define IK_X_DIR (+1.0f)

static void fRIK(float x,float th0,float z){
    x *= IK_X_DIR;
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(1,  th0                            + offset[1]);
    servo_write(2, -((th1-th1_neutral)*180.0f/PI)  + offset[2]);
    servo_write(3,  (th2-th2_neutral)*180.0f/PI    + offset[3]);
}
static void rRIK(float x,float th0,float z){
    x *= IK_X_DIR;
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(7,  th0                            + offset[7]);
    servo_write(8, -((th1-th1_neutral)*180.0f/PI)  + offset[8]);
    servo_write(9,  (th2-th2_neutral)*180.0f/PI    + offset[9]);
}
static void fLIK(float x,float th0,float z){
    x *= IK_X_DIR;
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(4,  th0                            + offset[4]);
    servo_write(5,  (th1-th1_neutral)*180.0f/PI    + offset[5]);
    servo_write(6, -((th2-th2_neutral)*180.0f/PI)  + offset[6]);
}
static void rLIK(float x,float th0,float z){
    x *= IK_X_DIR;
    float zd=z/cosf(th0/180.0f*PI);
    float ld=sqrtf(x*x+zd*zd);
    float phi=atan2f(x,zd);
    float th1=phi-acosf((L1*L1+ld*ld-L2*L2)/(2*L1*ld));
    float th2=asinf((ld*ld-L1*L1-L2*L2)/(2*L1*L2))-th1;
    servo_write(10, th0                            + offset[10]);
    servo_write(11, (th1-th1_neutral)*180.0f/PI    + offset[11]);
    servo_write(12,-((th2-th2_neutral)*180.0f/PI)  + offset[12]);
}

/* ---- automated walk-tune: run the Stanford trot gait while streaming
 * per-servo tracking CSV, so you can PID-tune against the REAL walk motion
 * (not just a step). Runs in the console task; needs 'cli on' so the gait
 * task doesn't fight the SPI bus. Same #WALK_BEGIN / #WALK_END framing idea
 * as run_sweep, but with one column group (set/now/err/cur/duty) per servo. */
static void run_swalk(const int *ids, int nids, int secs, int hz, float vx){
    if(hz < 1)   hz = 1;
    if(hz > 100) hz = 100;
    if(secs < 1) secs = 1;
    if(secs > 60) secs = 60;
    if(nids < 1) return;
    int log_period_ms = 1000 / hz;

    printf("#WALK_BEGIN secs=%d hz=%d vx=%.1f ids=", secs, hz, vx);
    for(int k=0;k<nids;k++) printf("%s%d", k?"-":"", ids[k]);
    printf("\n");
    printf("t_ms");
    for(int k=0;k<nids;k++)
        printf(",set%d,now%d,err%d,cur%d,duty%d",
               ids[k], ids[k], ids[k], ids[k], ids[k]);
    printf("\n");

    /* start walking from the calibrated stance (same as the live gait task):
     * exact BSP IK, body height = `height`. */
    servo_speed_all(0);
    pose_x = 0; pose_z = (float)height;
    stanford_gait_reset((float)height);

    uint32_t t0 = millis();
    uint32_t end_ms = t0 + (uint32_t)secs * 1000u;
    uint32_t next_log = t0;
    int64_t  next_us = esp_timer_get_time();

    while(millis() < end_ms && CliMode){
        sg_foot_t feet[4];
        stanford_gait_step(vx, 0.0f, 0.0f, (float)height,
                           SG_NATIVE_CLEARANCE_MM, feet);
        float sdeg[13];                      /* exact BSP IK, same as live walk */
        stanford_kinematics_servo_deg(feet, sdeg);
        for(int i=1;i<=12;i++) servo_write(i, sdeg[i] + offset[i]);
        servo_flush();

        uint32_t now_ms = millis();
        if((int32_t)(now_ms - next_log) >= 0){
            next_log += log_period_ms;
            printf("%lu", (unsigned long)(now_ms - t0));
            for(int k=0;k<nids;k++){
                int id = ids[k];
                float set_deg=0, now_deg=0, err_deg=0, cur=0, duty=0;
                bool ok = driver_board_get_live(id, DB_LIVE_SETPOINT_POS_DEG, &set_deg)
                       && driver_board_get_live(id, DB_LIVE_PRESENT_POS_DEG, &now_deg)
                       && driver_board_get_live(id, DB_LIVE_ERROR_POS_DEG,   &err_deg)
                       && driver_board_get_live(id, DB_LIVE_PRESENT_CUR_MA,  &cur)
                       && driver_board_get_live(id, DB_LIVE_PWM_DUTY,        &duty);
                if(ok) printf(",%.1f,%.1f,%.1f,%.0f,%.1f",
                              set_deg, now_deg, err_deg, cur, duty*100.0f);
                else   printf(",,,,,");
            }
            printf("\n");
        }

        /* pace to the next SG_DT (15 ms) gait tick; resync if we fell behind */
        next_us += (int64_t)(SG_DT * 1e6f);
        int64_t now = esp_timer_get_time();
        if(now > next_us + 100000) next_us = now;
        while(esp_timer_get_time() < next_us && CliMode) vTaskDelay(1);
    }

    /* park the legs back in the neutral stand so they aren't mid-stride */
    fRIK(0,0,NEUTRAL_Z); fLIK(0,0,NEUTRAL_Z);
    rRIK(0,0,NEUTRAL_Z); rLIK(0,0,NEUTRAL_Z);
    servo_flush();
    pose_x = 0; pose_z = NEUTRAL_Z;
    printf("#WALK_END\n");
}

/* ---- Stretch-motion PID trace: all four legs bob up/down together
 * (z = height + upHeight*sin), exactly like the web 'Stretch' button, while
 * streaming per-servo tracking CSV for the chosen ids. Reuses the same
 * #WALK_BEGIN / #WALK_END framing as run_swalk so plot_walk.py plots it too.
 * Needs 'cli on'. Uses the live height/upHeight/period globals.            */
static void run_sstretch(const int *ids, int nids, int secs, int hz){
    if(hz < 1)   hz = 1;
    if(hz > 100) hz = 100;
    if(secs < 1) secs = 1;
    if(secs > 60) secs = 60;
    if(nids < 1) return;
    int log_period_ms = 1000 / hz;
    uint32_t cyc = (uint32_t)period * 8u;        /* one full sine, ms */
    if(cyc < 1) cyc = 1;

    printf("#WALK_BEGIN secs=%d hz=%d vx=0.0 ids=", secs, hz);
    for(int k=0;k<nids;k++) printf("%s%d", k?"-":"", ids[k]);
    printf("\n");
    printf("t_ms");
    for(int k=0;k<nids;k++)
        printf(",set%d,now%d,err%d,cur%d,duty%d",
               ids[k], ids[k], ids[k], ids[k], ids[k]);
    printf("\n");

    servo_speed_all(0);

    uint32_t t0 = millis();
    uint32_t end_ms = t0 + (uint32_t)secs * 1000u;
    uint32_t next_log = t0;

    while(millis() < end_ms && CliMode){
        uint32_t tim = millis() - t0;
        float tt = (float)(tim % cyc) * 2.0f * PI / (float)cyc;
        float z = (float)height + (float)upHeight * sinf(tt);
        fRIK(0,0,z); rLIK(0,0,z); rRIK(0,0,z); fLIK(0,0,z);
        servo_flush();                            /* paces the loop to ~5 ms */

        uint32_t now_ms = millis();
        if((int32_t)(now_ms - next_log) >= 0){
            next_log += log_period_ms;
            printf("%lu", (unsigned long)(now_ms - t0));
            for(int k=0;k<nids;k++){
                int id = ids[k];
                float set_deg=0, now_deg=0, err_deg=0, cur=0, duty=0;
                bool ok = driver_board_get_live(id, DB_LIVE_SETPOINT_POS_DEG, &set_deg)
                       && driver_board_get_live(id, DB_LIVE_PRESENT_POS_DEG, &now_deg)
                       && driver_board_get_live(id, DB_LIVE_ERROR_POS_DEG,   &err_deg)
                       && driver_board_get_live(id, DB_LIVE_PRESENT_CUR_MA,  &cur)
                       && driver_board_get_live(id, DB_LIVE_PWM_DUTY,        &duty);
                if(ok) printf(",%.1f,%.1f,%.1f,%.0f,%.1f",
                              set_deg, now_deg, err_deg, cur, duty*100.0f);
                else   printf(",,,,,");
            }
            printf("\n");
        }
    }

    /* park back in the neutral stand */
    fRIK(0,0,NEUTRAL_Z); fLIK(0,0,NEUTRAL_Z);
    rRIK(0,0,NEUTRAL_Z); rLIK(0,0,NEUTRAL_Z);
    servo_flush();
    pose_x = 0; pose_z = NEUTRAL_Z;
    printf("#WALK_END\n");
}

/* Log one CSV row for the traced servos if a sample period has elapsed.
 * Shared by run_sjump; same column layout as run_swalk / run_sstretch. */
static void walk_log_row(const int *ids, int nids, uint32_t t0,
                         uint32_t *next_log, int log_period_ms){
    uint32_t now_ms = millis();
    if((int32_t)(now_ms - *next_log) < 0) return;
    *next_log += log_period_ms;
    printf("%lu", (unsigned long)(now_ms - t0));
    for(int k=0;k<nids;k++){
        int id = ids[k];
        float set_deg=0, now_deg=0, err_deg=0, cur=0, duty=0;
        bool ok = driver_board_get_live(id, DB_LIVE_SETPOINT_POS_DEG, &set_deg)
               && driver_board_get_live(id, DB_LIVE_PRESENT_POS_DEG, &now_deg)
               && driver_board_get_live(id, DB_LIVE_ERROR_POS_DEG,   &err_deg)
               && driver_board_get_live(id, DB_LIVE_PRESENT_CUR_MA,  &cur)
               && driver_board_get_live(id, DB_LIVE_PWM_DUTY,        &duty);
        if(ok) printf(",%.1f,%.1f,%.1f,%.0f,%.1f",
                      set_deg, now_deg, err_deg, cur, duty*100.0f);
        else   printf(",,,,,");
    }
    printf("\n");
}

/* ---- in-place Jump PID trace ------------------------------------------
 * Runs the SAME jump motion as the web/CLI 'jump' button (crouch -> push ->
 * tuck -> land), `reps` times, while streaming per-servo tracking CSV. NO
 * forward motion (this is the in-place jump only). Same #WALK_BEGIN /
 * #WALK_END framing as run_swalk so plot_walk.py plots it too. Needs 'cli on'.
 * Uses the live height/period globals - same crouch/push/tuck as the real
 * jump, so the trace reflects exactly what the robot does.                 */
static void run_sjump(const int *ids, int nids, int hz, int reps){
    if(hz < 1)   hz = 1;
    if(hz > 100) hz = 100;
    if(reps < 1) reps = 1;
    if(reps > 20) reps = 20;
    if(nids < 1) return;
    int log_period_ms = 1000 / hz;

    const float crouchZ = 40.0f, pushZ = 105.0f, tuckZ = 45.0f;

    printf("#WALK_BEGIN secs=%d hz=%d vx=0.0 ids=", reps, hz);
    for(int k=0;k<nids;k++) printf("%s%d", k?"-":"", ids[k]);
    printf("\n");
    printf("t_ms");
    for(int k=0;k<nids;k++)
        printf(",set%d,now%d,err%d,cur%d,duty%d",
               ids[k], ids[k], ids[k], ids[k], ids[k]);
    printf("\n");

    uint32_t t0 = millis();
    uint32_t next_log = t0;
    uint32_t time_mSt, tim; float tt;

    for(int r = 0; r < reps && CliMode; r++){
        /* 1) crouch down from the stand */
        time_mSt = millis(); tim = 0;
        while(tim < (uint32_t)period*2){ tim = millis()-time_mSt;
            tt = (float)(tim * PI / 2.0 / (period*2));
            float z = height - (height - crouchZ) * sinf(tt);
            fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush();
            walk_log_row(ids, nids, t0, &next_log, log_period_ms);
        }
        /* 2) brief settle at full crouch */
        time_mSt = millis(); tim = 0;
        while(tim < 20){ tim = millis()-time_mSt;
            fRIK(0,0,crouchZ); fLIK(0,0,crouchZ); rRIK(0,0,crouchZ); rLIK(0,0,crouchZ);
            servo_flush();
            walk_log_row(ids, nids, t0, &next_log, log_period_ms);
        }
        /* 3) explosive push to full extension */
        servo_speed_all(0);
        fRIK(0,0,pushZ); fLIK(0,0,pushZ); rRIK(0,0,pushZ); rLIK(0,0,pushZ);
        servo_flush(); servo_flush();
        /* 4) airborne - hold the push, keep logging */
        int airMs = (int)(160.0f + (70.0f - crouchZ) * 1.0f);
        time_mSt = millis(); tim = 0;
        while(tim < (uint32_t)airMs && CliMode){ tim = millis()-time_mSt;
            walk_log_row(ids, nids, t0, &next_log, log_period_ms);
            vTaskDelay(1);
        }
        /* 5) tuck the legs up */
        time_mSt = millis(); tim = 0;
        int tuckMs = 50;
        while(tim < (uint32_t)tuckMs){ tim = millis()-time_mSt;
            float frac = sinf((float)tim * PI / 2.0f / (float)tuckMs);
            float z = pushZ - (pushZ - tuckZ) * frac;
            fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush();
            walk_log_row(ids, nids, t0, &next_log, log_period_ms);
        }
        /* 6) ease back down to the stand */
        time_mSt = millis(); tim = 0;
        while(tim < (uint32_t)period*3){ tim = millis()-time_mSt;
            tt = (float)(tim * PI / 2.0 / (period*3));
            float z = tuckZ + (height - tuckZ) * sinf(tt);
            fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush();
            walk_log_row(ids, nids, t0, &next_log, log_period_ms);
        }
        /* 7) settle at the stand between reps */
        if(r + 1 < reps){
            time_mSt = millis(); tim = 0;
            while(tim < 500 && CliMode){ tim = millis()-time_mSt;
                fRIK(0,0,height); fLIK(0,0,height); rRIK(0,0,height); rLIK(0,0,height);
                servo_flush();
                walk_log_row(ids, nids, t0, &next_log, log_period_ms);
            }
        }
    }

    /* park in the calibrated stand */
    fRIK(0,0,height); fLIK(0,0,height); rRIK(0,0,height); rLIK(0,0,height);
    servo_flush();
    pose_x = 0; pose_z = height;
    printf("#WALK_END\n");
}

#define ROOT_BUF_SZ 40000   /* room for the CLI dump output + frame table */
/* How many taught frames get their angles printed. ~250 bytes each; the whole
 * page shares ROOT_BUF_SZ with cli_out, so this is what keeps a long teach
 * session from truncating the page. */
#define BF_DETAIL_ROWS 24
static esp_err_t send_root(httpd_req_t *req){
    char *b = malloc(ROOT_BUF_SZ);
    if(!b) return ESP_ERR_NO_MEM;
    int n=0;
    /* snprintf returns the length it WOULD have written, so once the page
     * fills, n runs past ROOT_BUF_SZ and (ROOT_BUF_SZ - n) goes negative.
     * That is passed as size_t -> ~4 GB -> unbounded write off the end of b.
     * Clamp n so the macro degrades to a no-op instead of corrupting the heap. */
    #define A(...) do{ \
        if(n < ROOT_BUF_SZ-1){ \
            int _w = snprintf(b+n, ROOT_BUF_SZ-n, __VA_ARGS__); \
            n = (_w < 0) ? n : (n + _w > ROOT_BUF_SZ-1 ? ROOT_BUF_SZ-1 : n + _w); \
        } \
    }while(0)
    #define ON(x) ((x)?"on":"off")

    A("<!DOCTYPE html><html lang=\"ja\"><head><meta charset=\"utf-8\">"
      "<title>Mini Pupper 2</title>"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>"
      ".container{margin:auto;text-align:center;font-size:1.2rem;}"
      "span,.pm{display:inline-block;border:1px solid #ccc;width:50px;height:30px;"
      "vertical-align:middle;margin-bottom:8px;}span{width:120px;}"
      "button{width:100px;height:40px;font-weight:bold;margin-bottom:8px;}"
      "button.on{background:lime;color:white;}"
      ".column-3{max-width:330px;margin:auto;text-align:center;display:flex;"
      "justify-content:space-between;flex-wrap:wrap;}"
      "button.twerk-btn{width:200px;background:#9b59b6;color:white;}"
      "button.twerk-btn.on{background:lime;color:white;}</style></head><body>"
      "<div class=\"container\"><h3>Mini Pupper 2</h3><div class=\"column-3\">");

    A("<button class=\"%s\" type=\"button\"><a href=\"/roll\">Roll</a></button><br>", ON(Roll));
    A("<button class=\"%s\" type=\"button\"><a href=\"/ad\">Advance</a></button><br>", ON(Advance));
    A("<button class=\"%s\" type=\"button\"><a href=\"/pitch\">Pitch</a></button><br>", ON(Pitch));
    A("<button class=\"%s\" type=\"button\"><a href=\"/left\">Left</a></button><br>", ON(Left));
    A("<button class=\"%s\" type=\"button\"><a href=\"/right\">Right</a></button><br>", ON(Right));
    A("<button class=\"%s\" type=\"button\"><a href=\"/turnL\">TurnL</a></button><br>", ON(TurnL));
    A("<button class=\"%s\" type=\"button\"><a href=\"/back\">Back</a></button><br>", ON(Back));
    A("<button class=\"%s\" type=\"button\"><a href=\"/turnR\">TurnR</a></button><br>", ON(TurnR));
    A("<button class=\"%s\" type=\"button\"><a href=\"/ini\">Ini</a></button><br>", ON(Ini));
    A("<button class=\"%s\" type=\"button\"><a href=\"/step\">Step</a></button><br>", ON(Step));
    A("<button class=\"%s\" type=\"button\"><a href=\"/stretch\">Stretch</a></button><br>", ON(Stretch));
    A("</div>");

    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\">"
      "<a href=\"/twerk\" style=\"color:white;\">&#127926; Twerk</a></button></div>", ON(Twerk));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#e67e22;\"><a href=\"/jump\" style=\"color:white;\">&#11014; Jump</a>"
      "</button></div>", ON(Jump));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#27ae60;\"><a href=\"/jumpfwd\" style=\"color:white;\">&#8599; Jump Fwd</a>"
      "</button></div>", ON(JumpFwd));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#8e44ad;\"><a href=\"/backflip\" style=\"color:white;\">&#128260; Backflip</a>"
      "</button></div>", ON(Backflip));
    /* Backflip20: its own table (backflip20.h) played with LINEAR interpolation,
     * so it reproduces play_slow.py rather than the smoothstep Play path. Timing
     * lives in the table, so SlowMo does not apply to it. */
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#16a085;\"><a href=\"/backflip20\" style=\"color:white;\">"
      "&#128260; Backflip20 (%s)</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#7f8c8d;\">"
      "<a href=\"/bf20load\" style=\"color:white;\">&#128194; Load for Verify</a>"
      "</button></div>", ON(Backflip20), BF20_SOURCE);

    /* ---- Teach a backflip by hand, from the browser --------------------- */
    A("<hr><h3 style=\"margin:6px;\">Teach backflip</h3>");
    A("<div style=\"margin:6px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:%s;\"><a href=\"/bfteach\" style=\"color:white;\">"
      "&#9995; Teach %s</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#27ae60;\">"
      "<a href=\"/bfrec\" style=\"color:white;\">&#11044; Record frame</a></button>"
      "</div>", ON(Relax), Relax?"#c0392b":"#2980b9", Relax?"ON (limp)":"OFF");
    A("<p style=\"font-size:0.9rem;\">Taught frames: <strong>%d</strong> / %d"
      " &middot; reference = frame %d</p>", bf_count, MAX_FRAMES, bf_ref_idx);
    A("<div style=\"margin:6px auto;\">"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#e67e22;\">"
      "<a href=\"/bfdel\" style=\"color:white;\">&#9003; Undo last</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#7f8c8d;\">"
      "<a href=\"/bfclear\" style=\"color:white;\">&#128465; Clear all</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#8e44ad;\">"
      "<a href=\"/bftaughtload\" style=\"color:white;\">&#128194; Load for Verify</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#16a085;\">"
      "<a href=\"/bfreplay\" style=\"color:white;\">&#9654; Replay</a></button>"
      "<button class=\"twerk-btn\" type=\"button\" style=\"background:#34495e;\">"
      "<a href=\"/bfdump\" style=\"color:white;\">&#128203; Dump C code</a></button>"
      "</div>");

    /* Show the dump here as well as in the CLI pane. That pane lives inside
     * `if(CliMode)`, so with CLI mode off - which is the normal state while
     * teaching - pressing Dump wrote the text and rendered it nowhere. */
    if(!CliMode && cli_out[0])
        A("<pre style=\"text-align:left;background:#111;color:#0f0;padding:8px;"
          "margin:6px;font-size:0.7rem;white-space:pre-wrap;word-wrap:break-word;"
          "\">%s</pre>", cli_out);

    /* Import: paste a dump back. id=bfimp so the page's submit interceptor
     * leaves it alone - that interceptor rewrites forms into GET query strings,
     * and a frame dump does not fit in a URI. */
    A("<div style=\"margin:6px auto;max-width:420px;\">"
      "<form id=\"bfimp\" onsubmit=\"return importBf(this)\">"
      "<textarea name=\"c\" rows=\"5\" style=\"width:98%%;font-size:0.68rem;\" "
      "placeholder=\"paste a Dump C code block (or hardcode_backflip_angle.h) "
      "here and press Import\"></textarea>"
      "<button type=\"submit\" style=\"width:100%%;height:32px;\">"
      "&#128229; Import frames</button></form></div>");

    if(bf_count > 0){
        A("<table style=\"margin:6px auto;font-size:0.78rem;border-collapse:collapse;\">"
          "<tr><th>frame</th><th>move ms</th><th>delay ms</th><th></th><th></th></tr>");
        for(int f=0; f<bf_count; f++){
            A("<tr%s><td>%d%s</td>"
              "<td><form action=\"/bfmove\" style=\"display:inline;\">"
              "<input type=\"hidden\" name=\"f\" value=\"%d\">"
              "<input name=\"ms\" value=\"%u\" size=\"5\" inputmode=\"numeric\">"
              "<button type=\"submit\">set</button></form></td>"
              "<td><form action=\"/bfmove\" style=\"display:inline;\">"
              "<input type=\"hidden\" name=\"f\" value=\"%d\">"
              "<input name=\"dly\" value=\"%u\" size=\"5\" inputmode=\"numeric\">"
              "<button type=\"submit\">set</button></form></td>"
              "<td><a href=\"/bfgoto?f=%d\">go</a></td>"
              "<td><a href=\"/bfsetref?f=%d\">ref</a></td></tr>",
              f==bf_ref_idx ? " style=\"background:#2c3e50;\"" : "",
              f, f==bf_ref_idx ? " *" : "",
              f, (unsigned)bf_move_ms[f], f, (unsigned)bf_delay_ms[f], f, f);

            /* The angles themselves, in exactly the form the pose / pose_bf
             * boxes take, so a frame can be copied into either without any
             * arithmetic: absolute SCS for `pose`, delta-from-reference for
             * `pose_bf`. Without this the table says a frame exists but not
             * what it is, which is no use for editing one movement.
             *
             * Capped: each of these costs ~250 bytes and the whole page shares
             * one ROOT_BUF_SZ buffer with the CLI output. Past the cap the A()
             * macro would just truncate, and a silently half-rendered page is
             * the same "I cannot see it" bug in a new place. The selected frame
             * is always shown, however far down the list it is. */
            if(f < BF_DETAIL_ROWS || f == verify_idx){
                A("<tr><td colspan=\"5\" style=\"text-align:left;padding:1px 6px;\">"
                  "<code style=\"color:#6cf;\">pose </code><code>");
                for(int i=1;i<=12;i++) A("%d%s", (int)bf_frames[f][i], i<12?",":"");
                A("</code><br><code style=\"color:#fc6;\">pose_bf </code><code>");
                for(int i=1;i<=12;i++)
                    A("%d%s", (int)bf_frames[f][i]-(int)bf_ref[i], i<12?",":"");
                A("</code></td></tr>");
            }else if(f == BF_DETAIL_ROWS){
                A("<tr><td colspan=\"5\" style=\"font-size:0.7rem;color:#888;\">"
                  "angles hidden past frame %d - press <b>go</b> to select a "
                  "frame and its angles appear</td></tr>", BF_DETAIL_ROWS-1);
            }
        }
        A("</table>");
    }

    /* ---- pose / pose_bf without a serial terminal ----------------------- */
    A("<div style=\"margin:6px auto;font-size:0.8rem;\">"
      "<form action=\"/wpose\" style=\"margin:4px;\">pose (12 absolute SCS): "
      "<input name=\"v\" size=\"40\" placeholder=\"511,511,511,...\" "
      "autocomplete=\"off\"><button type=\"submit\">go</button></form>"
      "<form action=\"/wposebf\" style=\"margin:4px;\">pose_bf (12 deltas from ref): "
      "<input name=\"v\" size=\"40\" placeholder=\"0,-30,40,...\" "
      "autocomplete=\"off\"><button type=\"submit\">go</button></form></div>");
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#2980b9;\"><a href=\"/testspeed\" style=\"color:white;\">&#9881; Test Speed</a>"
      "</button></div>", ON(TestSpeed));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#c0392b;\"><a href=\"/mate\" style=\"color:white;\">&#10084; Mate</a>"
      "</button></div>", ON(Mate));
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#16a085;\"><a href=\"/stanford\" style=\"color:white;\">&#128021; Stanford Walk</a>"
      "</button></div>", ON(Stanford));
    // Joystick pads (mini_pupper_web_controller style). Touching a pad
    // auto-starts the Stanford trot; releasing steps in place. Each pad
    // has a knob that follows the finger for visual feedback.
    A("<div style=\"margin:10px auto;display:flex;justify-content:center;gap:24px;\">"
      "<div><div id=\"padL\" style=\"position:relative;width:130px;height:130px;"
      "border-radius:50%%;background:#e8edf2;border:2px solid #8fa1b3;"
      "touch-action:none;-webkit-user-select:none;user-select:none;\">"
      "<div id=\"knobL\" style=\"position:absolute;left:39px;top:39px;width:52px;"
      "height:52px;border-radius:50%%;background:#4a6fa5;pointer-events:none;\"></div>"
      "</div><div style=\"font-size:0.72rem;color:#666;\">move (fwd / strafe)</div></div>"
      "<div><div id=\"padR\" style=\"position:relative;width:130px;height:130px;"
      "border-radius:50%%;background:#f2ede8;border:2px solid #b3a18f;"
      "touch-action:none;-webkit-user-select:none;user-select:none;\">"
      "<div id=\"knobR\" style=\"position:absolute;left:39px;top:39px;width:52px;"
      "height:52px;border-radius:50%%;background:#a5764a;pointer-events:none;\"></div>"
      "</div><div style=\"font-size:0.72rem;color:#666;\">turn</div></div></div>");
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#34495e;\"><a href=\"/climode\" style=\"color:white;\">&#128187; CLI Mode</a>"
      "</button></div>", ON(CliMode));
    // Dynamixel-Wizard-style parameter UI. Relative href (no leading '/') so
    // the AJAX body-swap interceptor skips it and the browser really navigates.
    A("<div style=\"margin:8px auto;\"><button class=\"twerk-btn\" type=\"button\" "
      "style=\"background:#2c3e50;\"><a href=\"wizard\" style=\"color:white;\">&#128062; MangDang Studio</a>"
      "</button></div>");
    if(CliMode){
        // AJAX CLI: command runs via fetch(), only the output box updates -
        // the page never reloads or jumps to the top.
        A("<div style=\"margin:8px auto;max-width:340px;\">"
          "<form id=\"clif\" onsubmit=\"return runCli(this)\">"
          "<input name=\"c\" id=\"clin\" style=\"width:225px;height:34px;\" "
          "placeholder=\"7 set kp_position 30\" autofocus autocomplete=\"off\">"
          "<button type=\"submit\" style=\"width:64px;height:40px;\">Run</button></form>"
          "<div style=\"font-size:0.72rem;color:#666;text-align:left;\">"
          "e.g. <code>7 pos 135 200</code> &middot; <code>7 tor 150</code> &middot; "
          "<code>7 stop</code> &middot; <code>trace 7</code> &middot; "
          "<code>7 get kp_position</code> &middot; <code>dump</code> &middot; "
          "<code>save</code> &middot; <code>help</code></div>"
          "<pre id=\"trout\" style=\"display:none;text-align:left;background:#001a33;"
          "color:#6cf;padding:8px;font-size:0.72rem;white-space:pre-wrap;"
          "word-wrap:break-word;\"></pre>"
          "<pre id=\"cliout\" style=\"text-align:left;background:#111;color:#0f0;padding:8px;"
          "font-size:0.72rem;white-space:pre-wrap;word-wrap:break-word;\">%s</pre>"
          "</div>", cli_out);
    }
    // Find which servo is currently overridden (if any)
    int active_id = 0;
    float active_deg = 135;
    for(int j=1;j<=12;j++){ if(manual_ovr[j]){ active_id=j; active_deg=manual_ovr_deg[j]; break; } }
    A("<div style=\"margin:8px auto;\">"
      "<form action=\"/leg\" method=\"get\" style=\"display:inline-flex;gap:6px;align-items:center;\">"
      "Servo <select name=\"id\" style=\"width:56px;height:34px;\">");
    for(int i=1;i<=12;i++)
        A("<option value=\"%d\"%s>%d</option>", i, (active_id==i)?" selected":"", i);
    A("</select>"
      "Angle <input type=\"number\" name=\"deg\" value=\"%.0f\" min=\"0\" max=\"270\" "
      "style=\"width:64px;height:34px;\">&deg;"
      "<button type=\"submit\" style=\"width:70px;background:%s;color:white;\">Set</button>"
      "</form></div>", active_deg, active_id?"lime":"#555");
    A("period (msec)<br><a class=\"pm\" href=\"/periodM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/periodP\">+</a><br>", period);
    A("height (mm)<br><a class=\"pm\" href=\"/heightM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/heightP\">+</a><br>", height);
    A("upHeight (mm)<br><a class=\"pm\" href=\"/upHeightM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/upHeightP\">+</a><br>", upHeight);
    A("stride (mm)<br><a class=\"pm\" href=\"/strideM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/strideP\">+</a><br>", stride);
    A("tilt (deg)<br><a class=\"pm\" href=\"/tiltM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/tiltP\">+</a><br>", tilt);
    A("stanford speed (mm/s)<br><a class=\"pm\" href=\"/sgsM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/sgsP\">+</a><br>", sgspeed);

    // ---- Teach & Record (hand-pose keyframes) --------------------------
    A("<hr><h3>&#128064; Teach &amp; Record</h3>"
      "<p style=\"font-size:0.85rem;color:#666;\">"
      "1) <b>Teach</b> relaxes the legs - pose them by hand.<br>"
      "2) <b>Record</b> each pose you want.<br>"
      "3) <b>Verify</b> steps through poses (low speed, normal torque).<br>"
      "4) <b>Play</b> runs the whole trace once. <b>Save</b> keeps it after power-off.</p>");
    A("<div style=\"margin:6px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#d35400;\"><a href=\"/relax\" style=\"color:white;\">"
      "&#9995; Teach %s</a></button></div>", ON(Relax), Relax?"(ON)":"");
    A("teach torque (mA) - lower = limper<br>"
      "<a class=\"pm\" href=\"/tcurM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/tcurP\">+</a><br>", teach_cur);
    A("<div class=\"column-3\" style=\"max-width:340px;\">"
      "<button type=\"button\" style=\"background:#2980b9;color:white;\">"
      "<a href=\"/rec\" style=\"color:white;\">&#128308; Record</a></button>"
      "<button type=\"button\"><a href=\"/recdel\">Del last</a></button>"
      "<button type=\"button\" style=\"background:#c0392b;color:white;\">"
      "<a href=\"/recclear\" style=\"color:white;\">Clear</a></button></div>");
    A("<p style=\"font-size:1rem;\">Frames recorded: <strong>%d</strong> / %d</p>",
      rec_count, MAX_FRAMES);
    // ---- CURRENT FLIP: whatever tools/newflip.py last generated ----------
    // This is the one to use. The others below are older fixed traces.
    A("<hr><h3>&#127917; Current flip</h3>"
      "<p style=\"font-size:0.85rem;color:#666;\">Source: <b>%s</b> &mdash; %d frames%s</p>",
      CURRENT_FLIP_SOURCE, CURRENT_FLIP_FRAMES,
      CURRENT_FLIP_REL_TO_INI ? ", starts from the robot's own stance" : "");
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#2c3e50;color:white;width:150px;\">"
      "<a href=\"/flipload\" style=\"color:white;\">&#128194; Load current</a></button>"
      "<button class=\"twerk-btn %s\" type=\"button\" style=\"background:#27ae60;width:150px;\">"
      "<a href=\"/flipplay\" style=\"color:white;\">&#9654; Play current</a></button></div>", ON(Play));
    if(current_flip_clamped)
        A("<p style=\"color:#c0392b;font-size:0.85rem;\"><b>%d value(s) clamped at a "
          "servo limit.</b> Regenerate with a lower --scale before running this.</p>",
          current_flip_clamped);

    // ---- older fixed traces ----------------------------------------------
    // Load the optimized backflip poses into the trace so Verify </> steps them.
    A("<hr><h3>&#128230; Older traces</h3>");
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#16a085;color:white;width:210px;\">"
      "<a href=\"/bfload\" style=\"color:white;\">&#128260; Load backflip frames</a></button></div>");
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#1abc9c;color:white;width:210px;\">"
      "<a href=\"/bfload2\" style=\"color:white;\">&#128260; Load backflip frames 2</a></button></div>");
    // Backflip 3: load (uses its own per-frame timing), plus a one-tap play.
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#16a085;color:white;width:150px;\">"
      "<a href=\"/bfload3\" style=\"color:white;\">&#128260; Load backflip 3</a></button>"
      "<button class=\"twerk-btn %s\" type=\"button\" style=\"background:#27ae60;width:150px;\">"
      "<a href=\"/bf3\" style=\"color:white;\">&#9654; Play backflip 3</a></button></div>", ON(Play));
    // Backflip 4: same data as Backflip 3, separate load + play buttons.
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#8e44ad;color:white;width:150px;\">"
      "<a href=\"/bfload4\" style=\"color:white;\">&#128260; Load backflip 4</a></button>"
      "<button class=\"twerk-btn %s\" type=\"button\" style=\"background:#9b59b6;width:150px;\">"
      "<a href=\"/bf4\" style=\"color:white;\">&#9654; Play backflip 4</a></button></div>", ON(Play));
    // HDF5 Trajectory #1: delta-format (auto-generated from bfv1.hdf5).
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#2980b9;color:white;width:150px;\">"
      "<a href=\"/hdf5traj1load\" style=\"color:white;\">&#128194; Load HDF5 Traj 1</a></button>"
      "<button class=\"twerk-btn %s\" type=\"button\" style=\"background:#3498db;width:150px;\">"
      "<a href=\"/hdf5traj1play\" style=\"color:white;\">&#9654; Play HDF5 Traj 1</a></button></div>", ON(Play));
    // Backflip v4: optimizer trajectory, amplitude-scaled to 0.90 so every
    // joint stays inside the servo range (no clamped values).
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#d35400;color:white;width:150px;\">"
      "<a href=\"/bfv4load\" style=\"color:white;\">&#128194; Load Backflip v4</a></button>"
      "<button class=\"twerk-btn %s\" type=\"button\" style=\"background:#e67e22;width:150px;\">"
      "<a href=\"/bfv4play\" style=\"color:white;\">&#9654; Play Backflip v4</a></button></div>", ON(Play));
    // Calibration test: lift one leg at a time (FL, FR, BL, BR).
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#e67e22;color:white;width:210px;\">"
      "<a href=\"/caltest\" style=\"color:white;\">&#129354; Load cal-test (1 leg each)</a></button></div>");
    // ---- SLOW MOTION + frame inspector --------------------------------
    // Play normally honours each trace's own per-frame timing. For the HDF5
    // trajectories that is ~24 ms/frame, which slams the servos. SlowMo
    // overrides it with the (much slower) global play_ms below.
    A("<hr><h3>&#128034; Slow motion / frame check</h3>");
    A("<div style=\"margin:6px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:%s;width:320px;\"><a href=\"/slowmo\" style=\"color:white;\">"
      "&#128034; Slow motion: %s</a></button></div>",
      ON(SlowMo), SlowMo?"#27ae60":"#c0392b", SlowMo?"ON (safe)":"OFF - FULL SPEED");
    A("<p style=\"font-size:0.85rem;color:#666;\">%s</p>",
      SlowMo ? "Play uses the slow speed below and ignores the trajectory's own "
               "per-frame timing. Safe for checking a new trace."
             : "<b style=\"color:#c0392b;\">Play uses the trajectory's real timing "
               "(~24 ms/frame on HDF5 traces). Only do this once the motion is "
               "verified.</b>");
    if(rec_count>0){
        A("Verify frame<br><a class=\"pm\" href=\"/verifyPrev\">&#9664;</a>"
          "<span>%d / %d</span><a class=\"pm\" href=\"/verifyNext\">&#9654;</a><br>",
          verify_idx+1, rec_count);
        // Show the 12 commanded SCS values for the frame under the cursor.
        // A value pinned at 0 or 1023 means the converter clamped it: that
        // joint has stopped following the trajectory and is parked against
        // its limit. Those are the frames that fight the gears.
        static const char *jn[13] = {"",
            "FR abd","FR hip","FR calf", "FL abd","FL hip","FL calf",
            "RR abd","RR hip","RR calf", "RL abd","RL hip","RL calf"};
        int n_pinned = 0;
        A("<table style=\"margin:8px auto;font-size:0.8rem;border-collapse:collapse;\">"
          "<tr><th style=\"padding:2px 8px;\">joint</th>"
          "<th style=\"padding:2px 8px;\">SCS</th>"
          "<th style=\"padding:2px 8px;\">deg</th></tr>");
        for(int id=1; id<=12; id++){
            int v = (int)rec_frames[verify_idx][id];
            int pinned = (v<=0 || v>=1023);
            if(pinned) n_pinned++;
            A("<tr><td style=\"padding:1px 8px;\">%s</td>"
              "<td style=\"padding:1px 8px;text-align:right;%s\">%d</td>"
              "<td style=\"padding:1px 8px;text-align:right;color:#888;\">%+.1f</td></tr>",
              jn[id], pinned?"color:#fff;background:#c0392b;font-weight:bold;":"",
              v, (v-511)*0.263);
        }
        A("</table>");
        if(n_pinned)
            A("<p style=\"color:#c0392b;font-size:0.85rem;\"><b>%d joint(s) pinned "
              "at a servo limit in this frame.</b> The converter clamped them, so "
              "they are no longer following the trajectory. Do not run this at "
              "full speed.</p>", n_pinned);
    } else {
        A("<p style=\"color:#999;\">Verify frame: none recorded yet</p>");
    }
    A("<div style=\"margin:6px auto;\"><button class=\"twerk-btn %s\" type=\"button\" "
      "style=\"background:#27ae60;\"><a href=\"/play\" style=\"color:white;\">"
      "&#9654; Play once</a></button></div>", ON(Play));
    A("play move (ms/pose)<br><a class=\"pm\" href=\"/pspM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/pspP\">+</a><br>", play_ms);
    A("pose delay (ms/pose)<br><a class=\"pm\" href=\"/pdlM\">-</a><span>%d</span>"
      "<a class=\"pm\" href=\"/pdlP\">+</a><br>", play_delay_ms);
    A("<div style=\"margin:6px auto;\">"
      "<button type=\"button\" style=\"background:#8e44ad;color:white;width:150px;\">"
      "<a href=\"/mirror\" style=\"color:white;\">&#128260; Mirror R&#8594;L</a></button>"
      "<button type=\"button\" style=\"background:#16a085;color:white;width:150px;\">"
      "<a href=\"/recsave\" style=\"color:white;\">&#128190; Save to flash</a></button></div>");

    A("<hr><h3>MiniPupper Calibration Tool</h3>"
      "<p style=\"font-size:0.9rem;color:#666;\">Press 'Ini' first, then adjust offsets."
      "<br>Each &minus; / + button = 1&deg;</p>");

    const char* legN[4]={"Leg 1 (Front Right)","Leg 2 (Front Left)","Leg 3 (Rear Right)","Leg 4 (Rear Left)"};
    const char* legC[4]={"#ffe4e1","#e1f5ff","#fff4e1","#e8ffe1"};
    const char* jN[3]={"Hip","Thigh","Calf"};
    A("<div style=\"display:flex;justify-content:center;flex-wrap:wrap;gap:10px;\">");
    for(int leg=0; leg<4; leg++){
        A("<div style=\"background:%s;padding:12px;border-radius:8px;min-width:172px;\">"
          "<strong>%s</strong>", legC[leg], legN[leg]);
        for(int j=0;j<3;j++){
            int id = leg*3 + j + 1;
            A("<div style=\"margin-top:8px;\">%s<br>"
              "<a class=\"pm\" href=\"/cal%dM\">&minus;</a>"
              "<span style=\"width:64px;\">%.1f&deg;</span>"
              "<a class=\"pm\" href=\"/cal%dP\">+</a></div>",
              jN[j], id, offset[id], id);
        }
        A("</div>");
    }
    A("</div>");
    A("<button type=\"button\" style=\"background:#ff6b6b;color:white;width:200px;\">"
      "<a href=\"/calReset\" style=\"color:white;\">Reset All Offsets to 0</a></button><br>"
      "</div>"
      // AJAX navigation: every button / +/- link / form is fetched in the
      // background and only the page body is swapped in place, so the page
      // NEVER reloads and never jumps - you stay exactly where you are.
      // (scroll save/restore kept as fallback for a manual F5)
      "<script>"
      "window.addEventListener('load',function(){"
      "var y=sessionStorage.getItem('sy');if(y)window.scrollTo(0,parseInt(y));});"
      "window.addEventListener('beforeunload',function(){"
      "sessionStorage.setItem('sy',window.scrollY);});"
      "function swapBody(t){var d=document.implementation.createHTMLDocument('');"
      "d.documentElement.innerHTML=t;document.body.innerHTML=d.body.innerHTML;"
      "if(window.initPads)initPads();}"
      // joystick pads: knob follows the finger, values -1..1 (circle
      // clamped), sent to /js at 10 Hz. Pointer events with touch/mouse
      // fallback so it works on every phone/desktop browser.
      "var jf=0,jsv=0,jt=0,jsend=0;"
      "function mkpad(id,kid,cb){var el=document.getElementById(id);"
      "var kn=document.getElementById(kid);if(!el||!kn||el._b)return;el._b=1;"
      "var drag=false;"
      "function xy(e){if(e.touches&&e.touches.length)e=e.touches[0];"
      "return [e.clientX,e.clientY];}"
      "function h(e){var p=xy(e),r=el.getBoundingClientRect();"
      "var R=(r.width-kn.offsetWidth)/2;"
      "var dx=p[0]-(r.left+r.width/2),dy=p[1]-(r.top+r.height/2);"
      "var m=Math.sqrt(dx*dx+dy*dy);"
      "if(m>R){dx=dx*R/m;dy=dy*R/m;}"
      "kn.style.left=(r.width/2-kn.offsetWidth/2+dx)+'px';"
      "kn.style.top=(r.height/2-kn.offsetHeight/2+dy)+'px';"
      "cb(dx/R,dy/R);jsend=1;}"
      "function rel(){if(!drag)return;drag=false;"
      "kn.style.left=(el.offsetWidth/2-kn.offsetWidth/2)+'px';"
      "kn.style.top=(el.offsetHeight/2-kn.offsetHeight/2)+'px';"
      "cb(0,0);jsend=1;}"
      "function dn(e){drag=true;h(e);e.preventDefault();}"
      "function mv(e){if(drag){h(e);e.preventDefault();}}"
      "if(window.PointerEvent){"
      "el.addEventListener('pointerdown',function(e){"
      "try{el.setPointerCapture(e.pointerId);}catch(x){}dn(e);});"
      "el.addEventListener('pointermove',mv);"
      "el.addEventListener('pointerup',rel);"
      "el.addEventListener('pointercancel',rel);"
      "}else{"
      "el.addEventListener('touchstart',dn,{passive:false});"
      "el.addEventListener('touchmove',mv,{passive:false});"
      "el.addEventListener('touchend',rel);"
      "el.addEventListener('mousedown',dn);"
      "document.addEventListener('mousemove',mv);"
      "document.addEventListener('mouseup',rel);}"
      "}"
      "function initPads(){"
      "mkpad('padL','knobL',function(x,y){jf=-y;jsv=-x;});"
      "mkpad('padR','knobR',function(x,y){jt=-x;});}"
      "window.addEventListener('load',initPads);"
      "setTimeout(initPads,300);"
      "if(!window._jsTimer){window._jsTimer=setInterval(function(){"
      "if(!document.getElementById('padL'))return;"
      "if(jsend||jf||jsv||jt){"
      "fetch('/js?f='+jf.toFixed(2)+'&s='+jsv.toFixed(2)+'&t='+jt.toFixed(2))"
      ".catch(function(){});"
      "if(!jf&&!jsv&&!jt)jsend=0;}},100);}"
      // web CLI (always defined, so it works after in-place body swaps too).
      // Trace runs in its OWN box (trout), so pos/tor/set/... commands keep
      // working below while the trace keeps updating.
      "var trI=null;"
      "function trStop(){if(trI){clearInterval(trI);trI=null;}"
      "var o=document.getElementById('trout');"
      "if(o){o.style.display='none';o.textContent='';}}"
      "function trStart(id){if(trI)clearInterval(trI);trI=setInterval(function(){"
      "var o=document.getElementById('trout');"
      "if(!o){clearInterval(trI);trI=null;return;}"
      "fetch('/tracepoll?id='+id).then(function(r){return r.text();})"
      ".then(function(t){o.style.display='block';o.textContent=t;"
      "if(t.indexOf('stopped')>=0)trStop();});},400);}"
      "function runCli(f){var v=f.c.value;if(!v)return false;"
      "var mOn=v.match(/^\\s*trace\\s+(\\d+)\\s*$/i);"
      "var mOff=v.match(/^\\s*trace\\s+off\\s*$/i);"
      "document.getElementById('cliout').textContent='...';"
      "fetch('/clix?c='+encodeURIComponent(v)).then(function(r){return r.text();})"
      ".then(function(t){document.getElementById('cliout').textContent=t;"
      "if(mOn)trStart(mOn[1]);else if(mOff)trStop();"
      "f.c.value='';f.c.focus();})"
      ".catch(function(e){document.getElementById('cliout').textContent='request failed: '+e;});"
      "return false;}"
      "document.addEventListener('click',function(e){"
      "var a=e.target&&e.target.closest?e.target.closest('a'):null;if(!a)return;"
      "var h=a.getAttribute('href');if(!h||h.charAt(0)!='/')return;"
      "e.preventDefault();"
      "fetch(h).then(function(r){return r.text();}).then(swapBody);});"
      // POST the pasted dump as a raw body, then swap in the new page.
      "function importBf(f){"
      "fetch('/bfimport',{method:'POST',body:f.c.value})"
      ".then(function(r){return r.text();}).then(swapBody);"
      "return false;}"
      "document.addEventListener('submit',function(e){"
      "var f=e.target;if(f.id=='clif'||f.id=='bfimp')return;"  // own AJAX handlers
      "e.preventDefault();"
      "var p=new URLSearchParams(new FormData(f)).toString();"
      "fetch(f.getAttribute('action')+'?'+p).then(function(r){return r.text();})"
      ".then(swapBody);});"
      "</script>"
      "</body></html>");

    #undef A
    #undef ON
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, b, n);
    free(b);
    return ESP_OK;
}

#define MOTION(name, var) \
static esp_err_t name(httpd_req_t*r){ \
    uint32_t t0 = millis(); \
    toggle_motion(&var); \
    esp_err_t ret = send_root(r); \
    ESP_LOGI(TAG, "HTTP %s handled in %lu ms", r->uri, (unsigned long)(millis()-t0)); \
    return ret; }
MOTION(h_ini,Ini)   MOTION(h_step,Step)   MOTION(h_roll,Roll)
MOTION(h_pitch,Pitch) MOTION(h_stretch,Stretch) MOTION(h_ad,Advance)
MOTION(h_back,Back) MOTION(h_left,Left)   MOTION(h_right,Right)
MOTION(h_turnL,TurnL) MOTION(h_turnR,TurnR) MOTION(h_twerk,Twerk)
MOTION(h_jump,Jump)
MOTION(h_jumpfwd,JumpFwd)
MOTION(h_backflip,Backflip)
MOTION(h_testspeed,TestSpeed)
MOTION(h_mate,Mate)
MOTION(h_stanford,Stanford)

static esp_err_t h_root(httpd_req_t*r){ return send_root(r); }

static esp_err_t h_periodM(httpd_req_t*r){ if(period>30){period-=5;nvs_put_int("period",period);} return send_root(r);}
static esp_err_t h_periodP(httpd_req_t*r){ if(period<=1000){period+=5;nvs_put_int("period",period);} return send_root(r);}
static esp_err_t h_heightM(httpd_req_t*r){ if(height>30){height-=5;nvs_put_int("height",height);} return send_root(r);}
static esp_err_t h_heightP(httpd_req_t*r){ if(height<=90){height+=5;nvs_put_int("height",height);} return send_root(r);}
static esp_err_t h_upM(httpd_req_t*r){ if(upHeight>0){upHeight-=2;nvs_put_int("upHeight",upHeight);} return send_root(r);}
static esp_err_t h_upP(httpd_req_t*r){ if(upHeight<=40){upHeight+=2;nvs_put_int("upHeight",upHeight);} return send_root(r);}
static esp_err_t h_strM(httpd_req_t*r){ if(stride>0){stride-=2;nvs_put_int("stride",stride);} return send_root(r);}
static esp_err_t h_strP(httpd_req_t*r){ if(stride<=40){stride+=2;nvs_put_int("stride",stride);} return send_root(r);}
static esp_err_t h_tiltM(httpd_req_t*r){ if(tilt>0){tilt-=2;nvs_put_int("tilt",tilt);} return send_root(r);}
static esp_err_t h_tiltP(httpd_req_t*r){ if(tilt<=40){tilt+=2;nvs_put_int("tilt",tilt);} return send_root(r);}
static esp_err_t h_sgsM(httpd_req_t*r){ if(sgspeed>20){sgspeed-=10;nvs_put_int("sgspeed",sgspeed);} return send_root(r);}
// up to 400 mm/s: 200 = reference max (Config.py max_x_velocity); beyond
// that is experimental - step length doubles and swings may not keep up
static esp_err_t h_sgsP(httpd_req_t*r){ if(sgspeed<400){sgspeed+=10;nvs_put_int("sgspeed",sgspeed);} return send_root(r);}

static esp_err_t h_cal(httpd_req_t*r){
    int id=0; sscanf(r->uri, "/cal%d", &id);
    char sign = r->uri[strlen(r->uri)-1];
    if(id>=1 && id<=12){
        offset[id] += (sign=='P') ? 1.0f : -1.0f;
        char k[12]; snprintf(k,sizeof k,"offset%d",id);
        nvs_put_float(k, offset[id]);
    }
    return send_root(r);
}
static esp_err_t h_calReset(httpd_req_t*r){
    for(int i=1;i<=12;i++){ offset[i]=0; char k[12];
        snprintf(k,sizeof k,"offset%d",i); nvs_put_float(k,0); }
    return send_root(r);
}

// /leg?id=N&deg=D  -> hold servo N at angle D degrees (0-270).
// Leaves all other servos at their neutral stand position.
static esp_err_t h_leg(httpd_req_t*r){
    char q[64], idv[8], degv[16];
    int id = 0; float deg = 135;
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK){
        if(httpd_query_key_value(q,"id",idv,sizeof idv)==ESP_OK) id = atoi(idv);
        if(httpd_query_key_value(q,"deg",degv,sizeof degv)==ESP_OK) deg = strtof(degv,NULL);
    }
    if(id<1 || id>12) id=1;
    if(deg<0) deg=0;
    if(deg>270) deg=270;
    reset_all_modes();
    started_once=1;
    // Set override for this servo; clear others
    for(int i=1;i<=12;i++) manual_ovr[i] = 0;
    manual_ovr[id] = 1;
    manual_ovr_deg[id] = deg;
    return send_root(r);
}

// ---- teach / record / playback handlers -------------------------------
// Toggle teach mode: relax the legs to a low-torque follow so you can pose
// them by hand. Pressing again exits back to the neutral stand.
static esp_err_t h_relax(httpd_req_t*r){
    started_once=1;
    if(Relax){ Relax=0; reset_all_modes(); }
    else     { reset_all_modes(); Relax=1; }
    return send_root(r);
}
// Record: request a snapshot of the current hand pose (captured by the gait
// task on its next pass, while in teach mode).
static esp_err_t h_rec(httpd_req_t*r){
    if(Relax && rec_count<MAX_FRAMES) rec_request=1;
    return send_root(r);
}
static esp_err_t h_recclear(httpd_req_t*r){ rec_count=0; verify_idx=0; use_frame_timing=0; return send_root(r); }
static esp_err_t h_recdel(httpd_req_t*r){                 // delete the last frame
    if(rec_count>0) rec_count--;
    if(verify_idx>=rec_count) verify_idx = rec_count>0 ? rec_count-1 : 0;
    return send_root(r);
}
static esp_err_t h_mirror(httpd_req_t*r){ mirror_RL(); return send_root(r); }
// Play the whole trace once (from the initial pose).
static esp_err_t h_play(httpd_req_t*r){
    if(rec_count>0){ reset_all_modes(); started_once=1; Play=1; }
    return send_root(r);
}
// Verify: move to a specific frame at low speed / normal torque and hold it.
static void start_goto(int f){
    if(rec_count<=0) return;
    if(f<0) f=0;
    if(f>=rec_count) f=rec_count-1;
    verify_idx=f; reset_all_modes(); started_once=1; goto_frame=f; Goto=1;
}
static esp_err_t h_verify(httpd_req_t*r){
    char q[32], v[8]; int f=verify_idx;
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"f",v,sizeof v)==ESP_OK) f=atoi(v);
    start_goto(f);
    return send_root(r);
}
static esp_err_t h_verifyPrev(httpd_req_t*r){ start_goto(verify_idx-1); return send_root(r); }
static esp_err_t h_verifyNext(httpd_req_t*r){ start_goto(verify_idx+1); return send_root(r); }
// Toggle the slow-motion override (Play ignores per-frame timing while ON).
static esp_err_t h_slowmo(httpd_req_t*r){ SlowMo = !SlowMo; return send_root(r); }
// Load the 28 optimized backflip poses into the trace buffer (as SCS) so the
// Verify </> buttons step through the FLIP frames one by one.
static esp_err_t h_bfload(httpd_req_t*r){
    reset_all_modes();
    int nf = BF_FRAMES < MAX_FRAMES ? BF_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++)
        for(int id=1; id<=12; id++){
            float ang = BF_SIGN[id-1]*(BF_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
            int sig = 511 + (int)(ang / 0.263f);
            if(sig<0) sig=0;
            if(sig>1023) sig=1023;
            rec_frames[f][id] = (uint16_t)sig;
        }
    rec_count = nf; verify_idx = 0; use_frame_timing = 0;
    return send_root(r);
}
// Load the hand-crafted (backflip_edit.py) poses into the trace.
static esp_err_t h_bfload2(httpd_req_t*r){
    reset_all_modes();
    int nf = BF2_FRAMES < MAX_FRAMES ? BF2_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++)
        for(int id=1; id<=12; id++){
            float ang = BF_SIGN[id-1]*(BF2_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
            int sig = 511 + (int)(ang / 0.263f);
            if(sig<0) sig=0;
            if(sig>1023) sig=1023;
            rec_frames[f][id] = (uint16_t)sig;
        }
    rec_count = nf; verify_idx = 0; use_frame_timing = 0;
    return send_root(r);
}
// Web: load the calibration test (lift one leg at a time) into the trace.
static esp_err_t h_caltest(httpd_req_t*r){
    reset_all_modes();
    int nf = CAL_FRAMES < MAX_FRAMES ? CAL_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++)
        for(int id=1; id<=12; id++){
            float ang = BF_SIGN[id-1]*(CAL_URDF_DEG[f][id-1] - BF_STAND[id-1]) + offset[id];
            int sig = 511 + (int)(ang / 0.263f);
            if(sig<0) sig=0;
            if(sig>1023) sig=1023;
            rec_frames[f][id] = (uint16_t)sig;
        }
    rec_count = nf; verify_idx = 0; use_frame_timing = 0;
    return send_root(r);
}
// Backflip 3: copy the hardcoded SCS frames + per-frame timing into the trace.
static void load_bf3(void){
    int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++)
            rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
        frame_move_ms[f]  = BF3_MOVE_MS[f];
        frame_delay_ms[f] = BF3_DELAY_MS[f];
    }
    rec_count = nf; verify_idx = 0; use_frame_timing = 1;
}
// Web: load Backflip 3 into the trace (then Verify </> or Play once).
static esp_err_t h_bfload3(httpd_req_t*r){ reset_all_modes(); load_bf3(); return send_root(r); }
// Web: load Backflip 3 and play it immediately with its per-frame timing.
static esp_err_t h_bf3(httpd_req_t*r){ reset_all_modes(); load_bf3(); started_once=1; Play=1; return send_root(r); }
// Backflip 4: same data source as Backflip 3, separate load + play helpers.
static void load_bf4(void){
    int nf = BF3_FRAMES < MAX_FRAMES ? BF3_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++)
            rec_frames[f][id] = (uint16_t)((int)BF3_REF[id] + (f==0 ? 0 : (int)BF3_DELTA[f-1][id]));
        frame_move_ms[f]  = BF3_MOVE_MS[f];
        frame_delay_ms[f] = BF3_DELAY_MS[f];
    }
    rec_count = nf; verify_idx = 0; use_frame_timing = 1;
}
static esp_err_t h_bfload4(httpd_req_t*r){ reset_all_modes(); load_bf4(); return send_root(r); }
static esp_err_t h_bf4(httpd_req_t*r){ reset_all_modes(); load_bf4(); started_once=1; Play=1; return send_root(r); }
// ---- HDF5 Trajectory #1 (delta-format, auto-generated from bfv1.hdf5) ----
static void load_hdf5_traj1(void){
    int nf = HDF5_TRAJ1_FRAMES < MAX_FRAMES ? HDF5_TRAJ1_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++)
            rec_frames[f][id] = (uint16_t)((int)HDF5_TRAJ1_REF[id] + (f==0 ? 0 : (int)HDF5_TRAJ1_DELTA[f-1][id]));
        frame_move_ms[f]  = HDF5_TRAJ1_MOVE_MS[f];
        frame_delay_ms[f] = HDF5_TRAJ1_DELAY_MS[f];
    }
    rec_count = nf; verify_idx = 0; use_frame_timing = 1;
}
// ---- Backflip v4 (optimizer trajectory, mp2_backflip_v4.hdf5) ----
// Generated with --scale 0.90, which is the largest amplitude at which every
// joint stays inside 0..1023. At full scale the two front knees ran past their
// limits on 9 frames and would have been clamped (i.e. parked against the stop
// while the trajectory kept moving).
static void load_mp2_bf_v4(void){
    int nf = MP2_BF_V4_FRAMES < MAX_FRAMES ? MP2_BF_V4_FRAMES : MAX_FRAMES;
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++)
            rec_frames[f][id] = (uint16_t)((int)MP2_BF_V4_REF[id] + (f==0 ? 0 : (int)MP2_BF_V4_DELTA[f-1][id]));
        frame_move_ms[f]  = MP2_BF_V4_MOVE_MS[f];
        frame_delay_ms[f] = MP2_BF_V4_DELAY_MS[f];
    }
    rec_count = nf; verify_idx = 0; use_frame_timing = 1;
}
static esp_err_t h_bfv4load(httpd_req_t*r){ reset_all_modes(); load_mp2_bf_v4(); return send_root(r); }
static esp_err_t h_bfv4play(httpd_req_t*r){ reset_all_modes(); load_mp2_bf_v4(); started_once=1; Play=1; return send_root(r); }

// ---- CURRENT FLIP: the slot tools/newflip.py writes ----------------------
// Regenerating current_flip.h swaps the motion. Nothing below changes.
//
// When CURRENT_FLIP_REL_TO_INI is set the deltas are added onto the LIVE Ini
// stance (511 + offset[]/0.263, i.e. your `setcal` calibration) rather than a
// baked-in REF. So the motion starts from wherever the robot is already
// standing — no teaching, and nothing to redo after a recalibration.
static void load_current_flip(void){
    uint16_t ini[13]; fill_ini_frame(ini);
    int nf = CURRENT_FLIP_FRAMES < MAX_FRAMES ? CURRENT_FLIP_FRAMES : MAX_FRAMES;
    current_flip_clamped = 0;
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++){
            // Deliberately a runtime ternary on a compile-time constant rather
            // than an #if: both arms stay compiled, so neither ini[] nor
            // CURRENT_FLIP_REF[] can go unreferenced and trip
            // -Wunused-const-variable under -Werror=all. The compiler folds it.
            int base = CURRENT_FLIP_REL_TO_INI ? (int)ini[id]
                                               : (int)CURRENT_FLIP_REF[id];
            int v = base + (f==0 ? 0 : (int)CURRENT_FLIP_DELTA[f-1][id]);
            // Clamp HERE, not with a silent cast. A value past the stop means
            // that joint has left the trajectory; count it so the web page and
            // `framecheck` can say so out loud.
            if(v < 0)    { v = 0;    current_flip_clamped++; }
            if(v > 1023) { v = 1023; current_flip_clamped++; }
            rec_frames[f][id] = (uint16_t)v;
        }
        frame_move_ms[f]  = CURRENT_FLIP_MOVE_MS[f];
        frame_delay_ms[f] = CURRENT_FLIP_DELAY_MS[f];
    }
    rec_count = nf; verify_idx = 0; use_frame_timing = 1;
    printf("current flip: %s, %d frames%s\n", CURRENT_FLIP_SOURCE, nf,
           CURRENT_FLIP_REL_TO_INI ? " (from robot's own stance)" : "");
    if(current_flip_clamped)
        printf("  WARNING: %d value(s) clamped at a servo limit -- "
               "regenerate with a lower --scale\n", current_flip_clamped);
}
/* ---- Backflip20: play backflip20.h with LINEAR interpolation ---------------
 *
 * Deliberately not routed through the trace / Play machinery. interp_to() eases
 * with a smoothstep, which arrives at every frame with ZERO velocity: on a
 * 19-knot trajectory that is nineteen separate little moves with a dead stop
 * between each. The angles come out right and the motion does not.
 *
 * Linear interpolation carries velocity through each frame boundary, which is
 * what the host-side play_slow.py does - and that is the version that moved
 * correctly on this robot. Same table, same result.
 *
 * BF20_COUNTS holds IDEAL counts (511 == the 70 mm stand); offset[] is added
 * here, so recalibrating never invalidates the table. */
#define BF20_STEP_MS 20      /* command interval; servo_flush() floors at 5 ms */

static void backflip20_run(volatile int *active)
{
    printf("backflip20: %s, %d frames\n", BF20_SOURCE, BF20_FRAMES);

    uint16_t prev[13];
    for(int i=1;i<=12;i++) prev[i] = goal[i];

    for(int f=0; f<BF20_FRAMES && (!active || *active); f++){
        /* Target for this frame, with the live calibration folded in.
         *
         * The four ABDUCTION servos (1/4/7/10) are deliberately left alone. The
         * planner locks abduction at zero and never produces data for it, so
         * the table carries a placeholder 511 for those columns - and 511 is
         * not where they sit on this robot (a taught frame reads them near 50).
         * Commanding the placeholder would swing all four hips through ~120 deg
         * that the trajectory never asked for. play_slow.py, the version that
         * was verified on the robot, writes only the eight leg servos; this
         * matches it. */
        uint16_t tgt[13];
        for(int i=1;i<=12;i++){
            if(i==1 || i==4 || i==7 || i==10){ tgt[i] = goal[i]; continue; }
            int v = (int)BF20_COUNTS[f][i] + (int)(offset[i]/0.263f);
            if(v < 0)    v = 0;
            if(v > 1023) v = 1023;
            tgt[i] = (uint16_t)v;
        }

        int mv = BF20_MOVE_MS[f];
        if(mv < 1) mv = 1;
        uint32_t t0 = millis(), tim;
        while((tim = millis()-t0) < (uint32_t)mv){
            if(active && !*active) break;
            /* LINEAR, not smoothstep - this is the whole point. */
            for(int i=1;i<=12;i++)
                goal[i] = (uint16_t)((int)prev[i] +
                          ((int)tgt[i]-(int)prev[i]) * (int)tim / mv);
            servo_flush();
            if(BF20_STEP_MS > 5) vTaskDelay(pdMS_TO_TICKS(BF20_STEP_MS-5));
        }
        for(int i=1;i<=12;i++){ goal[i] = tgt[i]; prev[i] = tgt[i]; }
        servo_flush();

        if(BF20_DELAY_MS[f] > 0) dwell_ms(BF20_DELAY_MS[f], active);
    }

    /* Settle back on the stand and hold it. */
    uint16_t ini[13]; fill_ini_frame(ini);
    interp_to(ini, 900, active);
    for(int i=1;i<=12;i++) hold_frame[i]=ini[i];
    HoldPose = 1;
}

static esp_err_t h_backflip20(httpd_req_t*r){
    reset_all_modes(); started_once=1; Backflip20=1; return send_root(r);
}

/* Copy backflip20.h into the trace buffer so the existing Verify < > cursor can
 * step it one frame at a time. Same offset[] handling as backflip20_run(), so a
 * frame inspected here is the frame that will play.
 *
 * Stepping is how you decide what to cut: walk the frames, note the indices that
 * are not doing anything, delete those ROWS from the _bf20.csv and regenerate.
 * Note that Play from the trace uses the smoothstep interp_to() and will stop at
 * every frame - for the real motion use the Backflip20 button, which does not. */
static void load_bf20(void){
    int nf = BF20_FRAMES < MAX_FRAMES ? BF20_FRAMES : MAX_FRAMES;
    uint16_t ini[13]; fill_ini_frame(ini);
    for(int f=0; f<nf; f++){
        for(int id=1; id<=12; id++){
            /* Abduction is a placeholder in the table - hold the stance value,
             * same reasoning as in backflip20_run(). */
            if(id==1 || id==4 || id==7 || id==10){ rec_frames[f][id] = ini[id]; continue; }
            int v = (int)BF20_COUNTS[f][id] + (int)(offset[id]/0.263f);
            if(v < 0)    v = 0;
            if(v > 1023) v = 1023;
            rec_frames[f][id] = (uint16_t)v;
        }
        frame_move_ms[f]  = BF20_MOVE_MS[f];
        frame_delay_ms[f] = BF20_DELAY_MS[f];
    }
    rec_count = nf;
    verify_idx = 0;
    use_frame_timing = 1;
    printf("backflip20: %d frames -> trace. Use Verify < > to step them.\n", nf);
}
static esp_err_t h_bf20load(httpd_req_t*r){ reset_all_modes(); load_bf20(); return send_root(r); }

/* ============ teach-backflip, from the web instead of the serial CLI ========
 * Same buffers and the same capture arithmetic as the `teach_backflip`/`rec_bf`
 * commands - these are additional front doors onto them, not a second
 * implementation, so a pose taught here and one taught over serial are the
 * same pose.
 *
 * The one thing the CLI never had is per-frame timing you can edit after the
 * fact: bf_move_ms[]/bf_delay_ms[] are set to sensible defaults on capture and
 * changed with /bfmove, so you can teach the shape first and tune the speed of
 * each individual movement afterwards without re-teaching anything.
 */
static int q_int(httpd_req_t*r, const char*key, int def){
    char q[192], v[16];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,key,v,sizeof v)==ESP_OK) return atoi(v);
    return def;
}

/* Toggle limp mode. Pressing it again after teaching re-enters teach without
 * clearing what is already recorded - "just re-teach" means adding to or
 * replacing frames, not starting over, so nothing is destroyed here. */
static esp_err_t h_bfteach(httpd_req_t*r){
    started_once = 1;
    if(Relax){ Relax = 0; reset_all_modes(); }
    else     { reset_all_modes(); Relax = 1; }
    return send_root(r);
}

static esp_err_t h_bfrec(httpd_req_t*r){
    if(!Relax || bf_count >= MAX_FRAMES) return send_root(r);
    for(int i=1; i<=12; i++){
        /* Present position is reported on the opposite scale to the command
         * (driver_board.c flips on write but not on read), hence 1023 - x.
         * Identical to the rec_bf command. */
        int cmd = 1023 - (int)driver_board_present_position(i);
        if(cmd < 0)    cmd = 0;
        if(cmd > 1023) cmd = 1023;
        if(bf_count == bf_ref_idx) bf_ref[i] = (uint16_t)cmd;
        bf_frames[bf_count][i] = (uint16_t)cmd;
    }
    bf_move_ms[bf_count]  = (uint16_t)(bf_count == 0 ? 800 : 200);
    bf_delay_ms[bf_count] = (uint16_t)(bf_count == 0 ? 300 : 0);
    bf_count++;
    return send_root(r);
}

static esp_err_t h_bfdel(httpd_req_t*r){
    if(bf_count > 0) bf_count--;
    if(bf_ref_idx >= bf_count) bf_ref_idx = bf_count > 0 ? bf_count-1 : 0;
    return send_root(r);
}
static esp_err_t h_bfclear(httpd_req_t*r){ bf_count = 0; bf_ref_idx = 0; return send_root(r); }

/* Jump to a taught frame.
 *
 * NOT start_goto(): that one indexes rec_frames[] and clamps to rec_count, i.e.
 * whatever trace happens to be loaded. If the taught frames have not been
 * pushed into the trace - or a different trace is loaded, like backflip20's 9 -
 * then "go" on frame 2 or 3 silently clamps and appears to do nothing. This
 * reads bf_frames[] directly, so it works the moment a frame is recorded.
 *
 * It also deliberately restores teach mode afterwards if it was on: GotoPose
 * needs the servos powered to move there, but dropping out of teach on every
 * inspection would mean re-enabling it before each new capture. */
static esp_err_t h_bfgoto(httpd_req_t*r){
    int f = q_int(r, "f", -1);
    if(f < 0 || f >= bf_count) return send_root(r);
    reset_all_modes();
    for(int i=1;i<=12;i++) pose_target[i] = bf_frames[f][i];
    verify_idx = f;
    started_once = 1;
    GotoPose = 1;
    return send_root(r);
}

/* Choose which taught frame is the reference the deltas are measured from. */
static esp_err_t h_bfsetref(httpd_req_t*r){
    int f = q_int(r, "f", 0);
    if(f >= 0 && f < bf_count){
        bf_ref_idx = f;
        for(int i=1;i<=12;i++) bf_ref[i] = bf_frames[f][i];
    }
    return send_root(r);
}

/* /bfmove?f=N&ms=X&dly=Y - retime one movement without re-teaching it. */
static esp_err_t h_bfmove(httpd_req_t*r){
    int f = q_int(r, "f", -1);
    int ms = q_int(r, "ms", -1);
    int dly = q_int(r, "dly", -1);
    if(f >= 0 && f < bf_count){
        if(ms  >= 1) bf_move_ms[f]  = (uint16_t)(ms  > 20000 ? 20000 : ms);
        if(dly >= 0) bf_delay_ms[f] = (uint16_t)(dly > 20000 ? 20000 : dly);
    }
    return send_root(r);
}

/* Push the taught frames into the trace so Verify < > and Play work on them. */
static void load_bf_taught(void){
    for(int f=0; f<bf_count; f++){
        for(int id=1; id<=12; id++) rec_frames[f][id] = bf_frames[f][id];
        frame_move_ms[f]  = bf_move_ms[f]  ? bf_move_ms[f]  : play_ms;
        frame_delay_ms[f] = bf_delay_ms[f];
    }
    rec_count = bf_count; verify_idx = 0; use_frame_timing = 1;
}
/* Named h_bftaughtload, not h_bfload: /bfload is already taken by the loader
 * for the 28 compiled-in optimised poses. */
static esp_err_t h_bftaughtload(httpd_req_t*r){
    reset_all_modes(); load_bf_taught(); return send_root(r);
}
static esp_err_t h_bfreplay(httpd_req_t*r){
    if(bf_count < 1) return send_root(r);
    reset_all_modes(); load_bf_taught(); started_once=1; Play=1; return send_root(r);
}

/* Same C as `recdump_bf` prints on serial, into the page's output pane so it
 * can be copied without a terminal attached. */
static esp_err_t h_bfdump(httpd_req_t*r){
    cli_out[0] = 0;
    if(bf_count < 1){ cli_printf("no frames taught yet\n"); return send_root(r); }
    cli_printf("// === paste into hardcode_backflip_angle.h ===\n");
    cli_printf("#define BF3_FRAMES %d\n\n", bf_count);
    cli_printf("static const uint16_t BF3_REF[13] = {\n             0,");
    for(int i=1;i<=12;i++) cli_printf(" %4u%s", (unsigned)bf_ref[i], i<12?",":"");
    cli_printf("\n};\n\n");
    if(bf_count > 1){
        cli_printf("static const int16_t BF3_DELTA[%d][13] = {\n", bf_count-1);
        for(int f=1; f<bf_count; f++){
            cli_printf("    {0");
            for(int i=1;i<=12;i++)
                cli_printf(", %5d", (int)bf_frames[f][i] - (int)bf_ref[i]);
            cli_printf("},  /* frame %d */\n", f);
        }
        cli_printf("};\n");
    }
    cli_printf("static const uint16_t BF3_MOVE_MS[%d] = {", bf_count);
    for(int f=0; f<bf_count; f++) cli_printf(" %u%s", (unsigned)bf_move_ms[f], f<bf_count-1?",":"");
    cli_printf(" };\nstatic const uint16_t BF3_DELAY_MS[%d] = {", bf_count);
    for(int f=0; f<bf_count; f++) cli_printf(" %u%s", (unsigned)bf_delay_ms[f], f<bf_count-1?",":"");
    cli_printf(" };\n// === end ===\n");
    return send_root(r);
}

/* ---- Import: paste a Dump back in and it becomes the taught frames --------
 *
 * The counterpart to /bfdump, so the pair is a real export/import round trip:
 * dump, keep the text anywhere, paste it back later and the frames are exactly
 * where they were. That matters because the taught buffer is RAM only - a
 * reflash, a power cycle or a dropped link loses it.
 *
 * Parsing ignores the C entirely and just reads integers in order, after
 * blanking comments (a `/ * frame 1 * /` marker would otherwise be scanned as
 * the number 1). It accepts anything shaped like the dump, so a hand-edited
 * hardcode_backflip_angle.h pastes in as readily as the button's own output. */
static void strip_c_comments(char *s){
    char *w = s;
    for(char *p = s; *p; ){
        if(p[0]=='/' && p[1]=='*'){
            p += 2;
            while(*p && !(p[0]=='*' && p[1]=='/')) p++;
            if(*p) p += 2;
        }else if(p[0]=='/' && p[1]=='/'){
            while(*p && *p != '\n') p++;
        }else{
            *w++ = *p++;
        }
    }
    *w = 0;
}

/* Read up to n integers starting at *pp. Returns how many were found. */
static int read_ints(const char **pp, int *dst, int n){
    const char *p = *pp;
    int got = 0;
    while(got < n && *p){
        while(*p && *p != '-' && (*p < '0' || *p > '9')) p++;
        if(!*p) break;
        char *end;
        long v = strtol(p, &end, 10);
        if(end == p) break;
        dst[got++] = (int)v;
        p = end;
    }
    *pp = p;
    return got;
}

/* Find `tok`, then step past the '{' that opens its initialiser. Without the
 * brace step the "13" in `BF3_REF[13] = {` would be read as the first value. */
static const char *find_array(const char *s, const char *tok){
    const char *p = strstr(s, tok);
    if(!p) return NULL;
    p = strchr(p, '{');
    return p ? p + 1 : NULL;
}

static esp_err_t h_bfimport(httpd_req_t*r){
    int total = r->content_len;
    cli_out[0] = 0;
    if(total <= 0 || total > 24000){
        cli_printf("import: body is %d bytes (need 1..24000)\n", total);
        return send_root(r);
    }
    char *body = malloc(total + 1);
    if(!body){ cli_printf("import: out of memory\n"); return send_root(r); }

    int got = 0;
    while(got < total){
        int k = httpd_req_recv(r, body + got, total - got);
        if(k <= 0){ free(body); cli_printf("import: receive failed\n");
                    return send_root(r); }
        got += k;
    }
    body[total] = 0;
    strip_c_comments(body);

    /* Frame count. Accept the #define, or fall back to counting DELTA rows. */
    int n = 0;
    const char *p = strstr(body, "BF3_FRAMES");
    if(p){ p += strlen("BF3_FRAMES"); read_ints(&p, &n, 1); }
    if(n < 1 || n > MAX_FRAMES){
        free(body);
        cli_printf("import: BF3_FRAMES missing or out of range (got %d)\n", n);
        return send_root(r);
    }

    int ref[13], mv[MAX_FRAMES], dl[MAX_FRAMES];
    const char *q = find_array(body, "BF3_REF");
    if(!q || read_ints(&q, ref, 13) != 13){
        free(body); cli_printf("import: BF3_REF needs 13 values\n");
        return send_root(r);
    }

    /* Deltas straight into the frame buffer, so a failure part-way cannot
     * leave the live taught set half-overwritten with someone else's frames. */
    static uint16_t tmp[MAX_FRAMES][13];
    for(int i=1;i<=12;i++) tmp[0][i] = (uint16_t)(ref[i] < 0 ? 0 :
                                                  ref[i] > 1023 ? 1023 : ref[i]);
    if(n > 1){
        const char *d = find_array(body, "BF3_DELTA");
        if(!d){ free(body); cli_printf("import: BF3_DELTA not found\n");
                return send_root(r); }
        for(int f=1; f<n; f++){
            int row[13];
            if(read_ints(&d, row, 13) != 13){
                free(body);
                cli_printf("import: BF3_DELTA has fewer than %d rows\n", n-1);
                return send_root(r);
            }
            for(int i=1;i<=12;i++){
                int v = ref[i] + row[i];
                if(v < 0)    v = 0;
                if(v > 1023) v = 1023;
                tmp[f][i] = (uint16_t)v;
            }
        }
    }

    /* Timing is optional - an older dump without it still imports. */
    for(int f=0; f<n; f++){ mv[f] = f==0 ? 800 : 200; dl[f] = f==0 ? 300 : 0; }
    const char *m = find_array(body, "BF3_MOVE_MS");
    if(m) read_ints(&m, mv, n);
    const char *y = find_array(body, "BF3_DELAY_MS");
    if(y) read_ints(&y, dl, n);

    for(int f=0; f<n; f++){
        for(int i=1;i<=12;i++) bf_frames[f][i] = tmp[f][i];
        bf_move_ms[f]  = (uint16_t)(mv[f] < 1 ? 1 : mv[f] > 20000 ? 20000 : mv[f]);
        bf_delay_ms[f] = (uint16_t)(dl[f] < 0 ? 0 : dl[f] > 20000 ? 20000 : dl[f]);
    }
    for(int i=1;i<=12;i++) bf_ref[i] = tmp[0][i];
    bf_count = n;
    bf_ref_idx = 0;

    free(body);
    cli_printf("import: %d frames loaded. Press Replay, or go to a frame.\n", n);
    return send_root(r);
}

/* /wpose?v=a,b,...,l  - the `pose` command. Absolute SCS, all 12. */
static esp_err_t h_wpose(httpd_req_t*r){
    char q[256], v[192];
    if(httpd_req_get_url_query_str(r,q,sizeof q)!=ESP_OK ||
       httpd_query_key_value(q,"v",v,sizeof v)!=ESP_OK) return send_root(r);
    int val[13]; int n=0; char *sp=NULL;
    for(char *tk=strtok_r(v," ,\t",&sp); tk && n<12; tk=strtok_r(NULL," ,\t",&sp)){
        int s = atoi(tk);
        if(s < 0)    s = 0;
        if(s > 1023) s = 1023;
        val[++n] = s;
    }
    if(n != 12){ cli_out[0]=0; cli_printf("pose: need 12 values, got %d\n", n);
                 return send_root(r); }
    reset_all_modes();
    for(int i=1;i<=12;i++) pose_target[i] = (uint16_t)val[i];
    started_once = 1; GotoPose = 1;
    return send_root(r);
}

/* /wposebf?v=d1,...,d12 - the `pose_bf` command: deltas from the reference. */
static esp_err_t h_wposebf(httpd_req_t*r){
    char q[256], v[192];
    if(httpd_req_get_url_query_str(r,q,sizeof q)!=ESP_OK ||
       httpd_query_key_value(q,"v",v,sizeof v)!=ESP_OK) return send_root(r);
    int d[13]; int n=0; char *sp=NULL;
    for(char *tk=strtok_r(v," ,\t",&sp); tk && n<12; tk=strtok_r(NULL," ,\t",&sp)){
        d[++n] = atoi(tk);
    }
    if(n != 12){ cli_out[0]=0; cli_printf("pose_bf: need 12 deltas, got %d\n", n);
                 return send_root(r); }
    /* Apply to whatever is currently serving as the reference: the taught one
     * if anything has been taught, otherwise the compiled-in BF3_REF. */
    reset_all_modes();
    for(int i=1;i<=12;i++){
        int base = bf_count > 0 ? (int)bf_ref[i] : (int)BF3_REF[i];
        int s = base + d[i];
        if(s < 0)    s = 0;
        if(s > 1023) s = 1023;
        pose_target[i] = (uint16_t)s;
    }
    started_once = 1; GotoPose = 1;
    return send_root(r);
}

static esp_err_t h_fliploadr(httpd_req_t*r){ reset_all_modes(); load_current_flip(); return send_root(r); }
static esp_err_t h_flipplay(httpd_req_t*r){ reset_all_modes(); load_current_flip(); started_once=1; Play=1; return send_root(r); }

static esp_err_t h_hdf5traj1load(httpd_req_t*r){ reset_all_modes(); load_hdf5_traj1(); return send_root(r); }
static esp_err_t h_hdf5traj1play(httpd_req_t*r){ reset_all_modes(); load_hdf5_traj1(); started_once=1; Play=1; return send_root(r); }
static esp_err_t h_pspM(httpd_req_t*r){ if(play_ms>100){ play_ms-=100; nvs_put_int("play_ms",play_ms);} return send_root(r); }
static esp_err_t h_pspP(httpd_req_t*r){ if(play_ms<3000){ play_ms+=100; nvs_put_int("play_ms",play_ms);} return send_root(r); }
// Dwell/hold at each pose before moving to the next (ms). 0 = no pause.
static esp_err_t h_pdlM(httpd_req_t*r){ if(play_delay_ms>0){ play_delay_ms-=100; if(play_delay_ms<0) play_delay_ms=0; nvs_put_int("play_dly",play_delay_ms);} return send_root(r); }
static esp_err_t h_pdlP(httpd_req_t*r){ if(play_delay_ms<5000){ play_delay_ms+=100; nvs_put_int("play_dly",play_delay_ms);} return send_root(r); }
// Teach-mode torque cap (mA): lower = limper / easier to pose by hand.
static esp_err_t h_tcurM(httpd_req_t*r){ if(teach_cur>10){ teach_cur-=10; nvs_put_int("teach_cur",teach_cur);} return send_root(r); }
static esp_err_t h_tcurP(httpd_req_t*r){ if(teach_cur<400){ teach_cur+=10; nvs_put_int("teach_cur",teach_cur);} return send_root(r); }
// Persist the trace to flash so the recorded angles survive a reboot.
static esp_err_t h_recsave(httpd_req_t*r){
    nvs_set_i32(nvs,"rec_cnt",rec_count);
    nvs_set_blob(nvs,"rec_fr",rec_frames,sizeof rec_frames);
    nvs_commit(nvs);
    return send_root(r);
}

// Toggle CLI mode: pauses the gait task (exclusive SPI access), servos
// hold their last commanded position via the AT32 control loops.
static esp_err_t h_climode(httpd_req_t*r){
    CliMode = !CliMode;
    if(CliMode){
        reset_all_modes();
        snprintf(cli_out, sizeof cli_out,
                 "CLI mode ON. Gait paused, servos hold position.\n"
                 "Type 'help' for commands.\n");
        vTaskDelay(pdMS_TO_TICKS(50));   // let the gait task park
    }else cli_out[0]=0;
    return send_root(r);
}

static esp_err_t h_clicmd(httpd_req_t*r){
    if(!CliMode){
        snprintf(cli_out, sizeof cli_out, "Enable CLI mode first.\n");
        return send_root(r);
    }
    char q[300], c[256];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"c",c,sizeof c)==ESP_OK){
        urldecode(c);
        cli_exec(c);
    }
    return send_root(r);
}

// AJAX variant: runs the command and returns ONLY the output as text/plain,
// so the browser updates the console box without reloading the page.
static esp_err_t h_clix(httpd_req_t*r){
    char q[300], c[256];
    if(!CliMode){
        httpd_resp_set_type(r, "text/plain");
        return httpd_resp_sendstr(r, "Enable CLI mode first.\n");
    }
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"c",c,sizeof c)==ESP_OK){
        urldecode(c);
        cli_exec(c);
    }
    httpd_resp_set_type(r, "text/plain");
    return httpd_resp_sendstr(r, cli_out);
}

// Web joystick input: /js?f=..&s=..&t=..  (each -1..1)
//   f = forward/back, s = strafe (+left), t = turn (+left)
// Touching a pad auto-starts the Stanford gait (like the web controller).
static esp_err_t h_js(httpd_req_t*r){
    char q[96], v[16];
    float f=0, s=0, t=0;
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK){
        if(httpd_query_key_value(q,"f",v,sizeof v)==ESP_OK) f=strtof(v,NULL);
        if(httpd_query_key_value(q,"s",v,sizeof v)==ESP_OK) s=strtof(v,NULL);
        if(httpd_query_key_value(q,"t",v,sizeof v)==ESP_OK) t=strtof(v,NULL);
    }
    if(f> 1)f= 1;
    if(f<-1)f=-1;
    if(s> 1)s= 1;
    if(s<-1)s=-1;
    if(t> 1)t= 1;
    if(t<-1)t=-1;

    if(!CliMode && !Stanford &&
       (fabsf(f)>0.08f || fabsf(s)>0.08f || fabsf(t)>0.08f)){
        reset_all_modes();
        started_once = 1;
        Stanford = 1;              // auto-start trotting on joystick input
    }
    // scale everything by the web "stanford speed" setting (full stick =
    // sgspeed mm/s; yaw scaled proportionally so turning matches)
    float k = (float)sgspeed / JOY_VX_MAX;
    js_vx = f * (float)sgspeed;
    js_vy = s * JOY_VY_MAX * k;
    js_wz = t * JOY_WZ_MAX * k;
    js_last_ms = millis();

    httpd_resp_set_type(r, "text/plain");
    return httpd_resp_sendstr(r, "ok");
}

// LIVE POSE STREAM: current position of all 12 servos as CSV (SCS 0..1023),
// e.g. "57,618,703,51,419,529,50,540,559,49,501,496". Poll this from the PC to
// mirror the real robot into MuJoCo in real time (see teach_live.py).
// Works in any mode; in `teach` the legs are limp so you can pose by hand.
static esp_err_t h_pos(httpd_req_t*r){
    char buf[128]; int n = 0;
    for(int id=1; id<=12; id++)
        n += snprintf(buf+n, sizeof(buf)-n, "%s%u",
                      id==1 ? "" : ",", driver_board_present_position(id));
    snprintf(buf+n, sizeof(buf)-n, "\n");
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(r, buf);
}

// Live trace poll: read ALL control-loop values of one servo over SPI
// (same set the AT32 uart_trace prints) and return them as text.
static esp_err_t h_tracepoll(httpd_req_t*r){
    httpd_resp_set_type(r, "text/plain");
    if(!CliMode) return httpd_resp_sendstr(r, "CLI mode off - trace stopped.\n");

    char q[64], v[8];
    int id = 0;
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"id",v,sizeof v)==ESP_OK) id = atoi(v);
    if(id<1 || id>12) return httpd_resp_sendstr(r, "trace: bad servo id\n");

    float lv[DB_LIVE_COUNT];
    bool live_ok = true;
    /* temperature (last id) is optional - see h_api_live() */
    for(int i=0; i<DB_LIVE_TEMPERATURE_C && live_ok; i++)
        live_ok = driver_board_get_live(id, i, &lv[i]);

    char b[640];
    if(live_ok){
        static const char *mn[] = {"IDLE","POSITION","TORQUE","IK"};
        int m = (int)lv[DB_LIVE_MODE];
        float t;
        if(!driver_board_get_live(id, DB_LIVE_TEMPERATURE_C, &t) || t <= DB_TEMP_INVALID)
            t = driver_board_present_temperature(id);
        int n = snprintf(b, sizeof b,
            "TRACE servo %d (live)   mode=%s   loop=%lu\n"
            "position:  set=%7.1f deg  now=%7.1f deg  err=%6.1f deg\n"
            "current:   cap=%5.0f mA  set=%5.0f mA  now=%5.0f mA  err=%5.0f mA\n"
            "pwm duty:  %5.1f %%\n"
            "raw ADC:   pos=%5.0f   cur=%5.0f\n",
            id, (m>=0&&m<4)?mn[m]:"?", (unsigned long)lv[DB_LIVE_LOOP_COUNTER],
            lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
            lv[DB_LIVE_ERROR_POS_DEG],
            lv[DB_LIVE_MAX_CURRENT_MA], lv[DB_LIVE_SETPOINT_CUR_MA],
            lv[DB_LIVE_PRESENT_CUR_MA], lv[DB_LIVE_ERROR_CUR_MA],
            lv[DB_LIVE_PWM_DUTY]*100.0f,
            lv[DB_LIVE_POS_ADC], lv[DB_LIVE_CUR_ADC]);
        if(t > DB_TEMP_INVALID)
            snprintf(b+n, sizeof b-n, "NTC temp:  %5.1f degC\n", t);
        else
            snprintf(b+n, sizeof b-n, "NTC temp:  --  (no reading)\n");
    }else if(driver_board_poll(id)){
        // old AT32 firmware without GET_LIVE: basic feedback only
        uint16_t p = driver_board_present_position(id);
        float t = driver_board_present_temperature(id);
        int n = snprintf(b, sizeof b,
            "TRACE servo %d (basic - flash new AT32 fw for full trace)\n"
            "pos = %4u SCS  %6.1f deg raw\ncur = %4d mA\n",
            id, p, (float)p*270.0f/1024.0f, driver_board_present_current(id));
        if(t > DB_TEMP_INVALID)
            snprintf(b+n, sizeof b-n, "ntc = %5.1f degC\n", t);
    }else{
        snprintf(b, sizeof b, "trace: SPI poll failed\n");
    }
    return httpd_resp_sendstr(r, b);
}

/* ============= Dynamixel-Wizard-style parameter UI (/wizard) =============
 * Static page embedded in flash + small JSON API. All parameter access
 * requires CLI mode (gait paused) so the HTTP task owns the SPI bus.     */
extern const char wizard_html_start[] asm("_binary_wizard_html_start");

static esp_err_t h_wizard(httpd_req_t*r){
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, wizard_html_start, strlen(wizard_html_start));
}

static int api_qint(httpd_req_t*r, const char*key, int def){
    char q[128], v[24];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,key,v,sizeof v)==ESP_OK) return atoi(v);
    return def;
}
static float api_qfloat(httpd_req_t*r, const char*key, float def){
    char q[128], v[24];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,key,v,sizeof v)==ESP_OK) return strtof(v,NULL);
    return def;
}
static esp_err_t api_json(httpd_req_t*r, const char*s){
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_sendstr(r, s);
}

/* /api/angles : read all 12 servo present positions as JSON.
 * Works WITHOUT CLI mode so you can monitor angles while the gait runs. */
static esp_err_t h_api_angles(httpd_req_t*r){
    char b[640]; int n = 0;
    n += snprintf(b+n, sizeof b-n, "{\"ok\":true,\"angles\":[");
    for(int id=1; id<=12; id++){
        uint16_t pos = driver_board_present_position(id);
        float deg = (float)pos * 270.0f / 1024.0f;
        float t   = driver_board_present_temperature(id);   /* rides on the same frame */
        n += snprintf(b+n, sizeof b-n, "%s{\"id\":%d,\"scs\":%u,\"deg\":%.1f,\"c\":",
                      id==1?"":",", id, pos, deg);
        if(t > DB_TEMP_INVALID) n += snprintf(b+n, sizeof b-n, "%.1f}", t);
        else                    n += snprintf(b+n, sizeof b-n, "null}");
    }
    n += snprintf(b+n, sizeof b-n, "]}");
    return api_json(r,b);
}

/* /api/setangle?id=N&deg=D : set servo N to angle D (0-270).
 * Works in TWO modes:
 *   CLI mode ON  -> uses driver_board_direct() (gait paused, direct SPI)
 *   CLI mode OFF -> uses the gait-task manual override mechanism
 *                   (gait runs, just overrides this servo on each tick) */
static esp_err_t h_api_setangle(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    float deg = api_qfloat(r,"deg",135);
    if(id<1 || id>12) return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    if(deg<0 || deg>270) return api_json(r,"{\"ok\":false,\"err\":\"deg out of range (0-270)\"}");

    if(CliMode){
        // Direct SPI access (gait paused)
        bool ok = driver_board_direct(id, DB_MODE_POSITION, deg, 200);
        char b[128];
        snprintf(b,sizeof b,"{\"ok\":%s,\"id\":%d,\"deg\":%.1f,\"mode\":\"direct\"}",
                 ok?"true":"false", id, deg);
        return api_json(r,b);
    }else{
        // Gait-task manual override (gait keeps running)
        reset_all_modes();
        started_once = 1;
        for(int i=1;i<=12;i++) manual_ovr[i] = 0;
        manual_ovr[id] = 1;
        manual_ovr_deg[id] = deg;
        char b[128];
        snprintf(b,sizeof b,"{\"ok\":true,\"id\":%d,\"deg\":%.1f,\"mode\":\"override\"}",
                 id, deg);
        return api_json(r,b);
    }
}

static esp_err_t h_api_status(httpd_req_t*r){
    char b[64];
    snprintf(b,sizeof b,"{\"ok\":true,\"climode\":%d}", CliMode?1:0);
    return api_json(r,b);
}

/* /api/climode?on=0|1 : enter/leave CLI mode (like the main page button) */
static esp_err_t h_api_climode(httpd_req_t*r){
    int on = api_qint(r,"on",-1);
    if(on==1 && !CliMode){
        reset_all_modes();
        CliMode = 1;
        vTaskDelay(pdMS_TO_TICKS(50));   /* let the gait task park */
    }else if(on==0){
        CliMode = 0;
    }
    return h_api_status(r);
}

/* /api/dump?id=N : all config parameters of one servo */
static esp_err_t h_api_dump(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    if(!CliMode)      return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(id<1 || id>12) return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    char b[640]; int n=0;
    n += snprintf(b+n,sizeof b-n,"{\"ok\":true,\"id\":%d,\"params\":[",id);
    for(int p=0; p<DB_PARAM_COUNT; p++){
        float v;
        if(driver_board_get_param(id,p,&v))
            n += snprintf(b+n,sizeof b-n,"%s{\"p\":%d,\"v\":%g}", p?",":"", p, v);
        else
            n += snprintf(b+n,sizeof b-n,"%s{\"p\":%d,\"v\":null}", p?",":"", p);
    }
    n += snprintf(b+n,sizeof b-n,"]}");
    return api_json(r,b);
}

/* /api/set?id=N&p=P&v=V : set one parameter (RAM), read it back */
static esp_err_t h_api_set(httpd_req_t*r){
    int id = api_qint(r,"id",0), p = api_qint(r,"p",-1);
    float v = api_qfloat(r,"v",0), rb=0;
    if(!CliMode)                 return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(id<1||id>12)              return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    if(p<0||p>=DB_PARAM_COUNT)   return api_json(r,"{\"ok\":false,\"err\":\"bad param\"}");
    if(driver_board_set_param(id,p,v) && driver_board_get_param(id,p,&rb)){
        char b[96];
        snprintf(b,sizeof b,"{\"ok\":true,\"p\":%d,\"v\":%g}",p,rb);
        return api_json(r,b);
    }
    return api_json(r,"{\"ok\":false,\"err\":\"no reply\"}");
}

/* /api/setall?p=P&v=V[&save=1] : write ONE parameter to ALL 12 servos (RAM),
 * optionally commit every board to flash afterwards.
 * Reply: {"ok":true,"p":P,"v":V,"n":<written>,"fail":[ids...],"saved":bool} */
static esp_err_t h_api_setall(httpd_req_t*r){
    int p    = api_qint(r,"p",-1);
    int save = api_qint(r,"save",0);
    float v  = api_qfloat(r,"v",0), rb=0;
    if(!CliMode)               return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(p<0||p>=DB_PARAM_COUNT) return api_json(r,"{\"ok\":false,\"err\":\"bad param\"}");

    char fail[64]; int fn=0, n=0;
    fail[0]=0;
    for(int id=1; id<=12; id++){
        bool ok = driver_board_set_param(id,p,v) && driver_board_get_param(id,p,&rb);
        if(ok) n++;
        else   fn += snprintf(fail+fn,sizeof fail-fn,"%s%d", fn?",":"", id);
        vTaskDelay(pdMS_TO_TICKS(2));   /* give the AT32 time between writes */
    }
    bool saved = false;
    if(save && n) saved = driver_board_save_config(-1);   /* -1 = all boards */

    char b[192];
    snprintf(b,sizeof b,
        "{\"ok\":%s,\"p\":%d,\"v\":%g,\"n\":%d,\"fail\":[%s],\"saved\":%s}",
        n?"true":"false", p, v, n, fail, saved?"true":"false");
    return api_json(r,b);
}

/* /api/live?id=N : live control-loop values (same set as 'trace') */
static esp_err_t h_api_live(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    if(!CliMode)      return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(id<1 || id>12) return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    float lv[DB_LIVE_COUNT]; bool ok=true;
    /* Everything up to DB_LIVE_TEMPERATURE_C is required. Temperature is
     * fetched separately and treated as optional so that a board still
     * running pre-NTC AT32 firmware degrades to "no temperature" instead of
     * dropping the whole trace back to the basic path. */
    for(int i=0; i<DB_LIVE_TEMPERATURE_C && ok; i++)
        ok = driver_board_get_live(id, i, &lv[i]);
    char b[576];
    if(ok){
        /* NTC temperature: prefer the live read, fall back to the value that
         * rides along on every ordinary feedback frame. */
        float t;
        bool have_t = driver_board_get_live(id, DB_LIVE_TEMPERATURE_C, &t);
        if(!have_t || t <= DB_TEMP_INVALID){
            t = driver_board_present_temperature(id);
            have_t = (t > DB_TEMP_INVALID);
        }
        int n = snprintf(b,sizeof b,
            "{\"ok\":true,\"full\":true,\"pos_adc\":%g,\"cur_adc\":%g,"
            "\"set_deg\":%g,\"now_deg\":%g,\"err_deg\":%g,"
            "\"cap_ma\":%g,\"set_ma\":%g,\"now_ma\":%g,\"err_ma\":%g,"
            "\"duty\":%g,\"mode\":%d,\"loop\":%lu",
            lv[DB_LIVE_POS_ADC], lv[DB_LIVE_CUR_ADC],
            lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
            lv[DB_LIVE_ERROR_POS_DEG],
            lv[DB_LIVE_MAX_CURRENT_MA], lv[DB_LIVE_SETPOINT_CUR_MA],
            lv[DB_LIVE_PRESENT_CUR_MA], lv[DB_LIVE_ERROR_CUR_MA],
            lv[DB_LIVE_PWM_DUTY], (int)lv[DB_LIVE_MODE],
            (unsigned long)lv[DB_LIVE_LOOP_COUNTER]);
        if(have_t) n += snprintf(b+n,sizeof b-n,",\"temp_c\":%.1f", t);
        snprintf(b+n,sizeof b-n,"}");
    }else if(driver_board_poll(id)){
        float t = driver_board_present_temperature(id);
        int n = snprintf(b,sizeof b,
            "{\"ok\":true,\"full\":false,\"now_deg\":%g,\"now_ma\":%d",
            (float)driver_board_present_position(id)*270.0f/1024.0f,
            driver_board_present_current(id));
        if(t > DB_TEMP_INVALID) n += snprintf(b+n,sizeof b-n,",\"temp_c\":%.1f", t);
        snprintf(b+n,sizeof b-n,"}");
    }else{
        snprintf(b,sizeof b,"{\"ok\":false,\"err\":\"spi\"}");
    }
    return api_json(r,b);
}

/* /api/temps : NTC temperature of all 12 servos, degC.
 *
 * Deliberately works WITHOUT CLI mode: the AT32 puts the temperature in
 * every feedback frame, so while the gait is running the cache is already
 * fresh and this costs no SPI traffic at all. When the gait is parked
 * (CLI mode) the cache would go stale, so re-poll the four boards first -
 * board_resend() replays the last commanded frame, which leaves an idle
 * servo idle and a holding servo holding.
 *
 * Reply: {"ok":true,"climode":0|1,"temps":[{"id":1,"c":31.4},...]}
 *        "c" is null for a servo that has never answered.               */
static esp_err_t h_api_temps(httpd_req_t*r){
    if(CliMode) for(int bd=0; bd<4; bd++) driver_board_poll_board(bd);

    char b[512]; int n = 0;
    n += snprintf(b+n, sizeof b-n, "{\"ok\":true,\"climode\":%d,\"temps\":[", CliMode?1:0);
    for(int id=1; id<=12; id++){
        float t = driver_board_present_temperature(id);
        if(t > DB_TEMP_INVALID)
            n += snprintf(b+n, sizeof b-n, "%s{\"id\":%d,\"c\":%.1f}", id==1?"":",", id, t);
        else
            n += snprintf(b+n, sizeof b-n, "%s{\"id\":%d,\"c\":null}", id==1?"":",", id);
    }
    snprintf(b+n, sizeof b-n, "]}");
    return api_json(r,b);
}

/* /api/save?id=N (0 = all boards) : commit board config to flash */
static esp_err_t h_api_save(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    if(!CliMode) return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    int board = (id>=1 && id<=12) ? (id-1)/3 : -1;
    return api_json(r, driver_board_save_config(board)
                       ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"save failed\"}");
}

/* /api/restore?id=N (0 = all boards) : factory defaults (RAM only) */
static esp_err_t h_api_restore(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    if(!CliMode) return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    int board = (id>=1 && id<=12) ? (id-1)/3 : -1;
    return api_json(r, driver_board_factory_restore(board)
                       ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"restore failed\"}");
}

/* /api/scan : probe all 12 servos (one param read each), list responders */
static esp_err_t h_api_scan(httpd_req_t*r){
    if(!CliMode) return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    char b[128]; int n=0, first=1;
    n += snprintf(b+n,sizeof b-n,"{\"ok\":true,\"found\":[");
    for(int id=1; id<=12; id++){
        float v;
        if(driver_board_get_param(id, DB_PARAM_KP_POSITION, &v)){
            n += snprintf(b+n,sizeof b-n,"%s%d", first?"":",", id);
            first = 0;
        }
    }
    n += snprintf(b+n,sizeof b-n,"]}");
    return api_json(r,b);
}

/* /api/direct?id=N&m=0|1|2&deg=..&cur=..  (0 idle, 1 position, 2 torque) */
static esp_err_t h_api_direct(httpd_req_t*r){
    int id = api_qint(r,"id",0), m = api_qint(r,"m",0);
    float deg = api_qfloat(r,"deg",135), cur = api_qfloat(r,"cur",130);
    if(!CliMode)      return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(id<1 || id>12) return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    uint16_t mode = (m==1) ? DB_MODE_POSITION : (m==2) ? DB_MODE_TORQUE : DB_MODE_IDLE;
    return api_json(r, driver_board_direct(id, mode, deg, (int16_t)cur)
                       ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"spi\"}");
}

static void reg(httpd_handle_t s,const char*uri,esp_err_t(*h)(httpd_req_t*)){
    httpd_uri_t u={.uri=uri,.method=HTTP_GET,.handler=h};
    httpd_register_uri_handler(s,&u);
}
/* POST variant. Needed for the frame import: the page's submit interceptor
 * turns every form into a GET query string, and a full frame dump does not fit
 * in a URI. */
static void reg_post(httpd_handle_t s,const char*uri,esp_err_t(*h)(httpd_req_t*)){
    httpd_uri_t u={.uri=uri,.method=HTTP_POST,.handler=h};
    httpd_register_uri_handler(s,&u);
}

static void start_webserver(void){
    httpd_handle_t s=NULL;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    /* 95 static routes + 24 generated /calNM|/calNP = 119. The teach-backflip
     * and pose routes pushed this past the old 110, and reg() ignores the
     * registration failure, so the overflow would have shown up as a handful of
     * buttons silently 404ing rather than as an error. */
    cfg.max_uri_handlers=140;   // base + teach/record + 24 calibration handlers
    cfg.stack_size=8192;
    cfg.core_id = 0;
    cfg.lru_purge_enable=true;
    ESP_ERROR_CHECK(httpd_start(&s,&cfg));

    reg(s,"/",h_root);
    reg(s,"/ini",h_ini);     reg(s,"/step",h_step);   reg(s,"/roll",h_roll);
    reg(s,"/pitch",h_pitch); reg(s,"/stretch",h_stretch);
    reg(s,"/ad",h_ad);       reg(s,"/back",h_back);   reg(s,"/left",h_left);
    reg(s,"/right",h_right); reg(s,"/turnL",h_turnL); reg(s,"/turnR",h_turnR);
    reg(s,"/twerk",h_twerk); reg(s,"/jump",h_jump); reg(s,"/jumpfwd",h_jumpfwd); reg(s,"/testspeed",h_testspeed);
    reg(s,"/backflip",h_backflip);
    reg(s,"/backflip20",h_backflip20);   // linear-interp playback of backflip20.h
    reg(s,"/bf20load",h_bf20load);       // load backflip20.h into the Verify trace
    /* teach-backflip from the browser (same buffers as the serial commands) */
    reg(s,"/bfteach",h_bfteach);   reg(s,"/bfrec",h_bfrec);
    reg(s,"/bfdel",h_bfdel);       reg(s,"/bfclear",h_bfclear);
    reg(s,"/bfsetref",h_bfsetref); reg(s,"/bfmove",h_bfmove);
    reg(s,"/bfgoto",h_bfgoto);
    reg(s,"/bftaughtload",h_bftaughtload); reg(s,"/bfreplay",h_bfreplay);
    reg(s,"/bfdump",h_bfdump);
    reg_post(s,"/bfimport",h_bfimport);   // paste a dump back in (POST: too big for a URI)
    reg(s,"/wpose",h_wpose);       reg(s,"/wposebf",h_wposebf);
    reg(s,"/mate",h_mate);
    reg(s,"/stanford",h_stanford);
    reg(s,"/relax",h_relax);       reg(s,"/rec",h_rec);
    reg(s,"/recclear",h_recclear); reg(s,"/recdel",h_recdel);
    reg(s,"/mirror",h_mirror);     reg(s,"/play",h_play);
    reg(s,"/verify",h_verify);     reg(s,"/verifyPrev",h_verifyPrev);
    reg(s,"/verifyNext",h_verifyNext);
    reg(s,"/slowmo",h_slowmo);
    reg(s,"/bfload",h_bfload);
    reg(s,"/bfload2",h_bfload2);
    reg(s,"/bfload3",h_bfload3);   reg(s,"/bf3",h_bf3);
    reg(s,"/bfload4",h_bfload4);   reg(s,"/bf4",h_bf4);
    reg(s,"/hdf5traj1load",h_hdf5traj1load); reg(s,"/hdf5traj1play",h_hdf5traj1play);
    reg(s,"/bfv4load",h_bfv4load);           reg(s,"/bfv4play",h_bfv4play);
    reg(s,"/flipload",h_fliploadr);          reg(s,"/flipplay",h_flipplay);
    reg(s,"/caltest",h_caltest);
    reg(s,"/pos",h_pos);           // live servo positions (CSV) for teach_live.py
    reg(s,"/pspM",h_pspM);         reg(s,"/pspP",h_pspP);
    reg(s,"/pdlM",h_pdlM);         reg(s,"/pdlP",h_pdlP);
    reg(s,"/tcurM",h_tcurM);       reg(s,"/tcurP",h_tcurP);
    reg(s,"/recsave",h_recsave);
    reg(s,"/climode",h_climode);
    reg(s,"/clicmd",h_clicmd);
    reg(s,"/clix",h_clix);
    reg(s,"/tracepoll",h_tracepoll);
    reg(s,"/js",h_js);
    reg(s,"/wizard",h_wizard);
    reg(s,"/api/status",h_api_status);
    reg(s,"/api/angles",h_api_angles);
    reg(s,"/api/setangle",h_api_setangle);
    reg(s,"/api/climode",h_api_climode);
    reg(s,"/api/dump",h_api_dump);
    reg(s,"/api/set",h_api_set);
    reg(s,"/api/setall",h_api_setall);
    reg(s,"/api/live",h_api_live);
    reg(s,"/api/save",h_api_save);
    reg(s,"/api/restore",h_api_restore);
    reg(s,"/api/direct",h_api_direct);
    reg(s,"/api/scan",h_api_scan);
    reg(s,"/api/temps",h_api_temps);   // NTC temps, all 12, works with gait running
    reg(s,"/periodM",h_periodM); reg(s,"/periodP",h_periodP);
    reg(s,"/heightM",h_heightM); reg(s,"/heightP",h_heightP);
    reg(s,"/upHeightM",h_upM);   reg(s,"/upHeightP",h_upP);
    reg(s,"/strideM",h_strM);    reg(s,"/strideP",h_strP);
    reg(s,"/tiltM",h_tiltM);     reg(s,"/tiltP",h_tiltP);
    reg(s,"/sgsM",h_sgsM);       reg(s,"/sgsP",h_sgsP);
    reg(s,"/calReset",h_calReset);
    reg(s,"/leg",h_leg);
    char uri[12];
    for(int i=1;i<=12;i++){
        snprintf(uri,sizeof uri,"/cal%dM",i); reg(s,strdup(uri),h_cal);
        snprintf(uri,sizeof uri,"/cal%dP",i); reg(s,strdup(uri),h_cal);
    }
}

#if ENABLE_MQTT
static esp_mqtt_client_handle_t mqtt_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch(event_id){
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected, subscribing to %s", MQTT_CMD_TOPIC);
            esp_mqtt_client_subscribe(mqtt_client, MQTT_CMD_TOPIC, 0);
            break;
        case MQTT_EVENT_DATA: {
            char cmd[32] = {0};
            int len = event->data_len < (int)sizeof(cmd)-1 ? event->data_len : (int)sizeof(cmd)-1;
            memcpy(cmd, event->data, len);
            for(int i=0;i<len;i++) cmd[i] = (char)tolower((unsigned char)cmd[i]);

            for(size_t i=0;i<MOTION_CMD_COUNT;i++){
                if(strcmp(cmd, motion_cmds[i].name)==0){
                    toggle_motion(motion_cmds[i].flag);
                    esp_mqtt_client_publish(mqtt_client, MQTT_STATE_TOPIC, cmd, 0, 0, 0);
                    break;
                }
            }
            break;
        }
        default: break;
    }
}

static void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}
#endif /* ENABLE_MQTT */

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    if(event_base==WIFI_EVENT && event_id==WIFI_EVENT_STA_START){
        esp_wifi_connect();
    }else if(event_base==WIFI_EVENT && event_id==WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    }else if(event_base==IP_EVENT && event_id==IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
#if ENABLE_MQTT
        mqtt_app_start();
#endif
    }
}

static void wifi_init_sta(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t sc = {0};
    strncpy((char*)sc.sta.ssid, WIFI_SSID, sizeof(sc.sta.ssid)-1);
    strncpy((char*)sc.sta.password, WIFI_PASS, sizeof(sc.sta.password)-1);
    sc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sc));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", WIFI_SSID);
}

static void gait_task(void *arg){
    float tim, tt;
    uint32_t time_mSt;

    for(int i=1;i<=12;i++) goal[i] = 511;
    servo_speed_all(0);

    for(;;){
        if(CliMode){
            // Web CLI owns the SPI bus; do not touch the driver boards.
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;

        }else if(Relax){
            // TEACH MODE: low-torque follow so the legs can be posed by hand
            // and stay roughly where you leave them. Each pass we command every
            // servo to its OWN present position with a small current cap, so it
            // gently resists gravity but yields to a firm push, then tracks the
            // new position. If a Record was requested, snapshot the pose here
            // (in the gait task, so we own the SPI bus and get a clean read).
            cur_override_mA = (uint16_t)teach_cur;   // live-tunable, lower = limper
            // IMPORTANT: driver_board_sync_write() INVERTS position (2700-x)
            // when commanding, but driver_board_present_position() reports
            // feedback WITHOUT that inversion. So present SCS runs opposite to
            // the commanded SCS (goal[]) - for the same physical angle,
            // present == 1023 - commanded. Convert feedback back into the
            // command convention here so the leg is held where it actually is,
            // and so recorded frames play back to the SAME physical pose.
            for(int i=1;i<=12;i++){
                int cmd = 1023 - (int)driver_board_present_position(i);
                if(cmd<0) cmd=0;
                if(cmd>1023) cmd=1023;
                goal[i] = (uint16_t)cmd;
            }
            servo_flush();                       // gentle hold + refresh feedback
            if(rec_request){
                if(rec_count < MAX_FRAMES){
                    for(int i=1;i<=12;i++){
                        int cmd = 1023 - (int)driver_board_present_position(i);
                        if(cmd<0) cmd=0;
                        if(cmd>1023) cmd=1023;
                        rec_frames[rec_count][i] = (uint16_t)cmd;
                    }
                    rec_count++;
                    use_frame_timing = 0;   // taught frames use global timing
                }
                rec_request = 0;
            }
            vTaskDelay(1);

        }else if(Play){
            // PLAY the whole trace ONCE: go to the initial (stance) pose first,
            // step pose-to-pose at low speed with NORMAL torque, then return to
            // the stance pose at the end and hold there.
            cur_override_mA = 0;
            uint16_t ini[13]; fill_ini_frame(ini);
            // SlowMo forces the global play_ms/play_delay_ms even for traces
            // that shipped their own per-frame timing (Backflip 3/4, HDF5).
            const int ft = use_frame_timing && !SlowMo;
            if(ft){
                // Backflip 3/4 (per-frame timing): start from the reference
                // frame (frame 0 = BF3_REF) instead of ini, so the robot
                // establishes the reference pose FIRST, then plays the delta
                // frames (1..N) on top of it.
                int mv0 = frame_move_ms[0] > 0 ? frame_move_ms[0] : play_ms;
                int dl0 = frame_delay_ms[0] >= 0 ? frame_delay_ms[0] : play_delay_ms;
                if(mv0 < 1) mv0 = 1;
                interp_to(rec_frames[0], mv0, &Play);
                dwell_ms(dl0, &Play);
            } else if(use_frame_timing){
                // SlowMo + a delta trace: still establish frame 0 (the REF
                // stance) first, but travel there at the slow global speed.
                interp_to(rec_frames[0], play_ms, &Play);
                dwell_ms(play_delay_ms, &Play);
            } else {
                interp_to(ini, play_ms, &Play);      // "all start from initial position"
            }
            for(int f=(use_frame_timing?1:0); f<rec_count && Play; f++){
                // per-frame timing (Backflip 3/4) if loaded, else the globals
                int mv = ft ? frame_move_ms[f]  : play_ms;
                int dl = ft ? frame_delay_ms[f] : play_delay_ms;
                if(mv < 1) mv = 1;
                interp_to(rec_frames[f], mv, &Play);
                dwell_ms(dl, &Play);              // dwell at this pose
            }
            if(Play) interp_to(ini, play_ms, &Play);   // ...and end back at stance
            if(Play){
                for(int i=1;i<=12;i++) hold_frame[i]=ini[i];
                HoldPose = 1;
            }
            Play = 0;

        }else if(Backflip20){
            cur_override_mA = 0;
            backflip20_run(&Backflip20);
            Backflip20 = 0;

        }else if(GotoPose){
            // Move to a single SCS pose typed in via the 'pose' command, then
            // hold it (normal torque). Same easing as verify/play.
            cur_override_mA = 0;
            interp_to(pose_target, play_ms, &GotoPose);
            for(int i=1;i<=12;i++) hold_frame[i]=pose_target[i];
            HoldPose = 1;
            GotoPose = 0;

        }else if(Goto){
            // VERIFY one keyframe: move to it at low speed / normal torque, then
            // hold it so you can inspect the angles before committing to Play.
            cur_override_mA = 0;
            if(goto_frame>=0 && goto_frame<rec_count){
                interp_to(rec_frames[goto_frame], play_ms, &Goto);
                for(int i=1;i<=12;i++) hold_frame[i]=rec_frames[goto_frame][i];
                HoldPose = 1;
            }
            Goto = 0;

        }else if(HoldPose){
            // Steady hold of the last played / verified pose (normal torque).
            cur_override_mA = 0;
            for(int i=1;i<=12;i++) goal[i]=hold_frame[i];
            servo_flush();
            vTaskDelay(1);

        }else if(Ini){
            servo_speed_all(0);
            for(int i=1; i<=12; i++) servo_write(i, offset[i]);
            servo_flush();
            pose_x = 0; pose_z = NEUTRAL_Z;   // Ini == neutral stand now
            vTaskDelay(1);

        }else if(Step){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,0,height-upHeight*sinf(tt)); rLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,0,height-upHeight*cosf(tt)); rLIK(0,0,height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,0,height-upHeight*sinf(tt)); fLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,0,height-upHeight*cosf(tt)); fLIK(0,0,height-upHeight*cosf(tt)); servo_flush(); }

        }else if(Roll){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height);
                rRIK(0,tilt*sinf(tt),height);  fLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Pitch){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,0,height-upHeight*sinf(tt)); rLIK(0,0,height+upHeight*sinf(tt));
                rRIK(0,0,height+upHeight*sinf(tt)); fLIK(0,0,height-upHeight*sinf(tt)); servo_flush(); }

        }else if(Stretch){
            time_mSt=millis(); tim=0;
            while(tim<period*8){ tim=millis()-time_mSt; tt=(float)(tim*2*PI/(period*8));
                fRIK(0,0,height+upHeight*sinf(tt)); rLIK(0,0,height+upHeight*sinf(tt));
                rRIK(0,0,height+upHeight*sinf(tt)); fLIK(0,0,height+upHeight*sinf(tt)); servo_flush(); }

        }else if(Advance){
            /* Walk direction: this gait's stride sweep runs the opposite way on
             * this robot, so Advance drove it backwards. Negate the stride HERE
             * (simple walk only) -- the Stanford gait and everything else that
             * uses fRIK/fLIK/rRIK/rLIK are left untouched. Flip the sign to
             * +stride to restore the original direction. */
            const int astride = -stride;
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-astride*cosf(tt),0,height-upHeight*sinf(tt)); rLIK(-astride*cosf(tt),0,height-upHeight*sinf(tt));
                rRIK( astride*cosf(tt),0,height);                   fLIK( astride*cosf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( astride*sinf(tt),0,height-upHeight*cosf(tt)); rLIK( astride*sinf(tt),0,height-upHeight*cosf(tt));
                rRIK(-astride*sinf(tt),0,height);                   fLIK(-astride*sinf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( astride*cosf(tt),0,height);                   rLIK( astride*cosf(tt),0,height);
                rRIK(-astride*cosf(tt),0,height-upHeight*sinf(tt)); fLIK(-astride*cosf(tt),0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-astride*sinf(tt),0,height);                   rLIK(-astride*sinf(tt),0,height);
                rRIK( astride*sinf(tt),0,height-upHeight*cosf(tt)); fLIK( astride*sinf(tt),0,height-upHeight*cosf(tt)); servo_flush(); }
            ESP_LOGI(TAG, "cur(mA): 1=%d 2=%d 3=%d 4=%d 5=%d 6=%d 7=%d 8=%d 9=%d 10=%d 11=%d 12=%d",
                driver_board_present_current(1),  driver_board_present_current(2),
                driver_board_present_current(3),  driver_board_present_current(4),
                driver_board_present_current(5),  driver_board_present_current(6),
                driver_board_present_current(7),  driver_board_present_current(8),
                driver_board_present_current(9),  driver_board_present_current(10),
                driver_board_present_current(11), driver_board_present_current(12));

        }else if(Back){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*cosf(tt),0,height-upHeight*sinf(tt)); rLIK( stride*cosf(tt)+15,0,height-upHeight*sinf(tt));
                rRIK(-stride*cosf(tt)+15,0,height);                fLIK(-stride*cosf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*sinf(tt),0,height-upHeight*cosf(tt)); rLIK(-stride*sinf(tt)+15,0,height-upHeight*cosf(tt));
                rRIK( stride*sinf(tt)+15,0,height);                fLIK( stride*sinf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*cosf(tt),0,height);                   rLIK(-stride*cosf(tt)+15,0,height);
                rRIK( stride*cosf(tt)+15,0,height-upHeight*sinf(tt)); fLIK( stride*cosf(tt),0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*sinf(tt),0,height);                   rLIK( stride*sinf(tt)+15,0,height);
                rRIK(-stride*sinf(tt)+15,0,height-upHeight*cosf(tt)); fLIK(-stride*sinf(tt),0,height-upHeight*cosf(tt)); servo_flush(); }

        }else if(Left){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0, tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,-tilt*sinf(tt),height); fLIK(0,tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); rLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); fLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*sinf(tt),height); rLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Right){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,tilt*sinf(tt),height); fLIK(0,-tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*cosf(tt),height-upHeight*cosf(tt)); rLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); fLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height); servo_flush(); }

        }else if(TurnL){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0, tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,tilt*sinf(tt),height); fLIK(0,tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));         rLIK(0,-tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt));   fLIK(0,tilt-2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*sinf(tt),height); rLIK(0,tilt*sinf(tt),height); servo_flush(); }

        }else if(TurnR){
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); rLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));
                rRIK(0, tilt*cosf(tt),height-upHeight*cosf(tt));        fLIK(0, tilt*cosf(tt),height-upHeight*cosf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                rRIK(0,-tilt*sinf(tt),height); fLIK(0,-tilt*sinf(tt),height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));          rLIK(0,tilt*cosf(tt),height-upHeight*cosf(tt));
                rRIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt));  fLIK(0,-tilt+2*tilt*sinf(tt),height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(0,-tilt*sinf(tt),height); rLIK(0,-tilt*sinf(tt),height); servo_flush(); }

        }else if(Twerk){
            // Fast up/down vibration: one quick full cycle per pass, which
            // the outer loop repeats continuously while Twerk is held.
            // Front and rear bounce in opposite phase for a bigger shake.
            float twerkAmp  = upHeight * 1.0f;
            float frontAmp  = upHeight * 0.6f;  // front legs bounce less than rear
            float zLo = 15.0f, zHi = 100.0f; // keep within safe leg reach

            servo_speed_all(300); // slower servo travel for a gentler shake
            time_mSt=millis(); tim=0;
            while(tim<period*5){ tim=millis()-time_mSt; tt=(float)(tim*2.0*PI/(period*5));
                float zf = fmaxf(zLo, fminf(zHi, height - frontAmp*sinf(tt)));
                float zr = fmaxf(zLo, fminf(zHi, height + twerkAmp*sinf(tt)));
                fRIK(0,0,zf); fLIK(0,0,zf);
                rRIK(0,0,zr); rLIK(0,0,zr); servo_flush(); }

        }else if(Mate){
            // Front legs stand tall and stay still; rear end thrusts up/down.
            float frontZ   = 90.0f;          // raised front stance, held fixed
            float rearMidZ = 50.0f;          // rear sits lower -> mounting posture
            float rearAmp  = upHeight * 1.5f;
            float zLo = 15.0f, zHi = 100.0f; // keep within safe leg reach

            servo_speed_all(400);
            fRIK(0,0,frontZ); fLIK(0,0,frontZ);
            servo_flush();

            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt; tt=(float)(tim*2.0*PI/(period*3));
                float zr = fmaxf(zLo, fminf(zHi, rearMidZ + rearAmp*sinf(tt)));
                fRIK(0,0,frontZ); fLIK(0,0,frontZ);
                rRIK(0,0,zr); rLIK(0,0,zr); servo_flush(); }

        }else if(Backflip){
            // ==================================================================
            // OPTIMIZED BACKFLIP -- SAFE BENCH DEMO (position-sequenced).
            // Steps through the flip keyframes, WAITING for the servos to reach
            // each pose (using position feedback) before advancing. This plays
            // the SHAPES of the backflip; it will NOT leave the ground -- a real
            // flip needs ~500 rpm joint speed the servos can't reach. Use it to
            // verify the motion + servo directions safely.
            //
            //  !!! FIRST RUN: hold/prop the robot and check each leg moves the
            //  RIGHT way. If a joint runs backwards, flip its sign in BF_SIGN[]
            //  (mp2_backflip_data.h) and re-flash. A wrong sign at speed breaks it.
            // ==================================================================
            const float BF_TOL_DEG    = 6.0f;    // "reached" tolerance (deg)
            const int   BF_TIMEOUT_MS = 1200;    // max wait per keyframe (ms)

            servo_speed_all(0);   // current cap applied in servo_flush()

            for(int fr=0; fr<BF_FRAMES && Backflip; fr++){
                float tgt[13];
                for(int id=1; id<=12; id++){
                    tgt[id] = BF_SIGN[id-1]*(BF_URDF_DEG[fr][id-1] - BF_STAND[id-1]) + offset[id];
                    servo_write(id, tgt[id]);
                }
                servo_flush();

                uint32_t t0 = millis();
                while(Backflip){
                    servo_flush();   // re-send + refresh position feedback
                    bool all_ok = true;
                    for(int id=1; id<=12; id++){
                        float now = ((int)driver_board_present_position(id) - 511) * 0.263f;
                        if(fabsf(now - tgt[id]) > BF_TOL_DEG){ all_ok = false; break; }
                    }
                    if(all_ok) break;
                    if(millis()-t0 > BF_TIMEOUT_MS) break;
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
            }

            // park back in the neutral stand
            fRIK(0,0,NEUTRAL_Z); fLIK(0,0,NEUTRAL_Z);
            rRIK(0,0,NEUTRAL_Z); rLIK(0,0,NEUTRAL_Z);
            servo_flush();
            Backflip = 0;

        }else if(Jump){
            float crouchZ = 40;
            float pushZ   = 105;
            float tuckZ   = 45;

            time_mSt=millis(); tim=0;
            while(tim<period*2){ tim=millis()-time_mSt;
                tt = (float)(tim * PI / 2.0 / (period*2));
                float z = height - (height - crouchZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<20){ tim=millis()-time_mSt;
                fRIK(0,0,crouchZ); fLIK(0,0,crouchZ); rRIK(0,0,crouchZ); rLIK(0,0,crouchZ); servo_flush(); }

            servo_speed_all(0);
            fRIK(0,0,pushZ); fLIK(0,0,pushZ); rRIK(0,0,pushZ); rLIK(0,0,pushZ);
            servo_flush();
            servo_flush();
            int airMs = (int)(160.0f + (70.0f - crouchZ) * 1.0f);
            vTaskDelay(pdMS_TO_TICKS(airMs));

            time_mSt=millis(); tim=0;
            int tuckMs = 50;
            while(tim<tuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)tuckMs);
                float z = pushZ - (pushZ - tuckZ) * frac;
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            time_mSt=millis(); tim=0;
            while(tim<period*3){ tim=millis()-time_mSt;
                tt = (float)(tim * PI / 2.0 / (period*3));
                float z = tuckZ + (height - tuckZ) * sinf(tt);
                fRIK(0,0,z); fLIK(0,0,z); rRIK(0,0,z); rLIK(0,0,z); servo_flush(); }

            Jump = 0;

        }else if(JumpFwd){
            float crouchZrear  = 60;
            float crouchZfront = 40;
            float pushZ        = 105;
            float tuckZ        = 45;

            // ----------------------------------------------------------------
            // Phase 1: CONTROLLED CROUCH
            //   Medium speed (70) so it looks like the dog is deliberately
            //   loading up energy rather than just falling down fast.
            // ----------------------------------------------------------------
            // Phase 1: CONTROLLED CROUCH
            //   Medium speed (100) so it looks like the dog is deliberately
            //   loading up energy rather than just falling down fast.
            // ----------------------------------------------------------------
            servo_speed_all(100);   // NOW this actually controls how slow it lowers
            fRIK(0,0,crouchZfront); fLIK(0,0,crouchZfront);
            rRIK(0,0,crouchZrear);  rLIK(0,0,crouchZrear);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(900));  // give it time to travel slowly
            // Brief settle so every servo actually reaches its crouch pose
            time_mSt=millis(); tim=0;
            while(tim<25){ tim=millis()-time_mSt;
                fRIK(0,0,crouchZfront); fLIK(0,0,crouchZfront);
                rRIK(0,0,crouchZrear);  rLIK(0,0,crouchZrear);
                servo_flush(); }

            // ----------------------------------------------------------------
            // Phase 2: EXPLOSIVE EXTENSION — max speed (0)
            //   The speed contrast with the slow crouch is what makes this
            //   feel snappy. Two packets for bus reliability.
            // ----------------------------------------------------------------
            servo_speed_all(0);
            fRIK(0,0,pushZ); fLIK(0,0,pushZ);
            rRIK(0,0,pushZ); rLIK(0,0,pushZ);
            servo_flush();
            servo_flush();

            // Read torque (load %) and current (mA) on the rear knee servos
            // right at the moment of the push, to see how hard they're working.
            {
                // feedback now comes back on each SPI transaction (position + current)
                ESP_LOGI(TAG, "JumpFwd push RR knee(9):  pos=%u cur=%dmA",
                         driver_board_present_position(9),  driver_board_present_current(9));
                ESP_LOGI(TAG, "JumpFwd push RL knee(12): pos=%u cur=%dmA",
                         driver_board_present_position(12), driver_board_present_current(12));
            }

            // Airborne window
            int airMs = (int)(160.0f + (70.0f - crouchZrear) * 1.0f);
            vTaskDelay(pdMS_TO_TICKS(airMs));

            // ----------------------------------------------------------------
            // Phase 3: POUNCE TUCK — still max speed (0)
            //   Must stay fast: we're airborne and need legs repositioned
            //   before the dog hits the ground.
            //   Rear tucks first while front stays reaching → pounce look.
            // ----------------------------------------------------------------
            // servo_speed already 0 from Phase 2, no need to set again
            time_mSt=millis(); tim=0;
            int rearTuckMs = 45;
            while(tim<rearTuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)rearTuckMs);
                float zr = pushZ - (pushZ - tuckZ) * frac;
                rRIK(0,0,zr); rLIK(0,0,zr);
                fRIK(0,0,pushZ); fLIK(0,0,pushZ);
                servo_flush(); }

            time_mSt=millis(); tim=0;
            int frontTuckMs = 45;
            while(tim<frontTuckMs){ tim=millis()-time_mSt;
                float frac = sinf((float)tim * PI / 2.0f / (float)frontTuckMs);
                float zf = pushZ - (pushZ - tuckZ) * frac;
                fRIK(0,0,zf); fLIK(0,0,zf);
                rRIK(0,0,tuckZ); rLIK(0,0,tuckZ);
                servo_flush(); }

            // ----------------------------------------------------------------
            // Phase 4: SOFT LANDING RECOVERY — slower speed (250)
            //   Legs extend gently to absorb the impact instead of snapping
            //   down hard. Looks springy, like the dog sticks the landing.
            // ----------------------------------------------------------------
            // Phase 4: SOFT LANDING — one command, servo speed controls how fast it arrives
            servo_speed_all(0);    // NOW this actually does something
            fRIK(0,0,height); fLIK(0,0,height);
            rRIK(0,0,height); rLIK(0,0,height);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(period * 4));  // wait for it to finish travelling
            // Always reset to max speed so other motions are unaffected
            servo_speed_all(0);
            JumpFwd = 0;
        }else if(TestSpeed){
            ESP_LOGI(TAG, "--- Speed Test START ---");

            // Step 1: go to a neutral mid position at max speed
            servo_speed_all(2047);
            fRIK(0,0,70); fLIK(0,0,70); rRIK(0,0,70); rLIK(0,0,70);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(1500));

            // Step 2: move to a lower position SLOWLY — you should see it creep down
            ESP_LOGI(TAG, "Moving SLOW (speed=30)");
            servo_speed_all(30);
            fRIK(0,0,100); fLIK(0,0,100); rRIK(0,0,100); rLIK(0,0,100);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(3000));   // watch it move slowly

            // Step 3: snap back FAST — you should see it jump back instantly
            ESP_LOGI(TAG, "Moving FAST (speed=2047)");
            servo_speed_all(2047);
            fRIK(0,0,70); fLIK(0,0,70); rRIK(0,0,70); rLIK(0,0,70);
            servo_flush();
            vTaskDelay(pdMS_TO_TICKS(2000));   // watch it snap back

            ESP_LOGI(TAG, "--- Speed Test DONE ---");
            servo_speed_all(0);
            TestSpeed = 0;   // auto-clears after one run

        }else if(Stanford){
            // Stanford Pupper trot gait (forward only), ported from
            // mangdangroboticsclub/StanfordQuadruped. Uses the NATIVE Mini
            // Pupper parameters from the BSP Config.py (height 80 mm,
            // clearance 30 mm, 15 ms tick) — independent of the web
            // sliders, so height/period/stride/upHeight are untouched.
            static int64_t sg_next_us = 0;

            // Walk at the NEUTRAL height (the calibrated Ini stance), not
            // the reference's 80 mm: the neutral-angle calibration is exact
            // at NEUTRAL_Z, so the gait starts walking DIRECTLY from the
            // stance the robot is already in - no dip / transition first.
            // Exact BSP IK rests at the BSP reference height (80mm), where
            // stanford_kinematics centres the servos == your Ini pose.
            // Walk at the calibrated `height` (the SAME web-slider value
            // Advance/idle use), NOT the 70mm centre. The exact IK module keeps
            // its neutral anchored at NEUTRAL_Z=70 (centred servos), so passing
            // `height` extends the legs to that stand - matching Advance.
            #define SG_WALK_HEIGHT ((float)height)

            if(!sg_started){
                // Short ramp only if the legs are away from the walk stance
                // (e.g. coming from a crouched mode); from Ini/stand this is
                // a no-op because pose == (0, NEUTRAL_Z) already.
                servo_speed_all(0);
                if(fabsf(pose_x) > 1.0f || fabsf(pose_z - SG_WALK_HEIGHT) > 1.0f){
                    const int rampMs = 400;
                    uint32_t t0 = millis();
                    for(;;){
                        uint32_t el = millis() - t0;
                        if(el > (uint32_t)rampMs) el = rampMs;
                        float f = sinf((float)el * PI / 2.0f / (float)rampMs);
                        float x = pose_x + (0.0f           - pose_x) * f;
                        float z = pose_z + (SG_WALK_HEIGHT - pose_z) * f;
                        fRIK(x,0,z); fLIK(x,0,z); rRIK(x,0,z); rLIK(x,0,z);
                        servo_flush();
                        if(el >= (uint32_t)rampMs || !Stanford) break;
                    }
                }
                pose_x = 0; pose_z = SG_WALK_HEIGHT;
                stanford_gait_reset(SG_WALK_HEIGHT);
                sg_next_us = esp_timer_get_time();
                sg_started = 1;
            }

            // Velocity command: joystick if fresh, else the fixed forward
            // walk (Stanford button without touching the pads).
            float vx = (float)sgspeed, vy = 0, wz = 0;
            if(millis() - js_last_ms < JOY_TIMEOUT_MS){
                vx = js_vx; vy = js_vy; wz = js_wz;
            }

            sg_foot_t feet[4];
            stanford_gait_step(vx, vy, wz, SG_WALK_HEIGHT,
                               SG_NATIVE_CLEARANCE_MM, feet);

            // Exact mini_pupper_2pro_bsp IK: one call fills all 12 servo
            // angles (degrees, before offset), then apply your calibration.
            // Abduction (strafe/turn) is handled inside with the 26mm offset.
            float sdeg[13];
            stanford_kinematics_servo_deg(feet, sdeg);
            for(int i=1;i<=12;i++) servo_write(i, sdeg[i] + offset[i]);
            servo_flush();

            // Pace to the next 15 ms tick; resync if we fell far behind.
            sg_next_us += (int64_t)(SG_DT * 1e6f);
            int64_t now = esp_timer_get_time();
            if(now > sg_next_us + 100000) sg_next_us = now;
            while(esp_timer_get_time() < sg_next_us && Stanford) vTaskDelay(1);

        }else{
            // Check if any servo has a manual override set.
            int any_ovr = 0;
            for(int i=1;i<=12;i++){ if(manual_ovr[i]){ any_ovr=1; break; } }
            if(any_ovr){
                // Hold a neutral stand, but drive overridden servos to their
                // manually entered degree positions instead of the IK values.
                servo_speed_all(0);
                fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
                for(int i=1;i<=12;i++){
                    if(manual_ovr[i]){
                        int scs = (int)(manual_ovr_deg[i] * 1024.0f / 270.0f);
                        if(scs<0) scs=0;
                        if(scs>1023) scs=1023;
                        goal[i] = (uint16_t)scs;
                    }
                }
                servo_flush();
                vTaskDelay(1);
            }else if(!started_once){
                // POWER-ON POSE: stand at the calibrated `height`.
                servo_speed_all(0);
                fRIK(0,0,height); fLIK(0,0,height); rRIK(0,0,height); rLIK(0,0,height);
                servo_flush();
                pose_x = 0; pose_z = height;
                vTaskDelay(1);
            }else{
                fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
                servo_flush();
                pose_x = 0; pose_z = height;
                vTaskDelay(1);
            }
        }
    }
}

void app_main(void){
    esp_err_t r = nvs_flash_init();
    if(r==ESP_ERR_NVS_NO_FREE_PAGES || r==ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase(); nvs_flash_init();
    }
    nvs_open("parameter", NVS_READWRITE, &nvs);

    ik_neutral_init();                   // Ini pose == IK neutral stand (see comment)
    driver_board_init();                 // SPI bus + 4 AT32 driver boards + servo power ON
    vTaskDelay(pdMS_TO_TICKS(1000));     // let servo power rails settle

    int32_t v;
    if(nvs_get_i32(nvs,"period",&v)==ESP_OK) period=v;
    if(nvs_get_i32(nvs,"height",&v)==ESP_OK) height=v;
    if(nvs_get_i32(nvs,"sgspeed",&v)==ESP_OK) sgspeed=v;
    if(nvs_get_i32(nvs,"play_ms",&v)==ESP_OK) play_ms=v;
    if(nvs_get_i32(nvs,"play_dly",&v)==ESP_OK && v>=0 && v<=5000) play_delay_ms=v;
    if(nvs_get_i32(nvs,"teach_cur",&v)==ESP_OK && v>=10 && v<=400) teach_cur=v;
    for(int i=1;i<=12;i++){ char k[12]; snprintf(k,sizeof k,"offset%d",i);
        offset[i]=nvs_get_float(k, offset[i]); }

    // Reload the saved teach/record trace ("hardcoded" hand poses).
    if(nvs_get_i32(nvs,"rec_cnt",&v)==ESP_OK && v>=0 && v<=MAX_FRAMES) rec_count=v;
    { size_t sz=sizeof rec_frames;
      nvs_get_blob(nvs,"rec_fr",rec_frames,&sz); }

    wifi_init_sta();
    start_webserver();

    xTaskCreatePinnedToCore(gait_task, "gait", 8192, NULL, 22, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 5, NULL, 0);
}