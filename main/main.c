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

// Manual override for servo 8 (Rear Right shoulder). When manual8 is set the
// gait task holds a neutral stand but drives servo 8 to manual8_pos.
static int manual8 = 0;
static uint16_t manual8_pos = 511;   // SCS position 0..1023 (511 = centre)

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
        cur[i] = speed_to_current_mA(goal_speed[i+1]);  // speed -> current limit (mA)
    }
    driver_board_sync_write(pos, cur);
}

static inline uint32_t millis(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }

static void reset_all_modes(void){
    Ini=Step=Roll=Pitch=Stretch=0;
    Advance=Back=Left=Right=TurnL=TurnR=Twerk=Jump=JumpFwd=TestSpeed=Mate=Stanford=0; // <-- add TestSpeed here
    manual8=0;
    sg_started=0;
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
                   "  trace <id> [hz]   live position/current + ESP timestamp (default 10Hz,\n"
                   "                    up to 200Hz; e.g. 'trace 2 50'; 'trace off' to stop)\n"
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
static uint32_t TracePeriodMs = 100;    /* sample period; 100ms = 10Hz default */

static void trace_print_line(int id){
    static const char *mn[] = {"IDLE","POS ","TOR ","IK  "};
    /* ESP timestamp in ms (0.1 ms resolution) so you can compute speed/RPM
     * by hand: RPM = (now2 - now1) deg / (t2 - t1) ms * 1000 / 6.            */
    double t_ms = (double)esp_timer_get_time() / 1000.0;
    float lv[DB_LIVE_COUNT];
    bool ok = true;
    for(int i=0; i<DB_LIVE_COUNT && ok; i++)
        ok = driver_board_get_live(id, i, &lv[i]);
    if(ok){
        int m = (int)lv[DB_LIVE_MODE];
        printf("t=%10.1f ms | pos s/n/e %6.1f/%6.1f/%5.1f deg | cur c/s/n/e %4.0f/%4.0f/%4.0f/%4.0f mA"
               " | duty %5.1f%% | adc %4.0f/%4.0f | %s | loop %lu\n",
               t_ms,
               lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
               lv[DB_LIVE_ERROR_POS_DEG],
               lv[DB_LIVE_MAX_CURRENT_MA], lv[DB_LIVE_SETPOINT_CUR_MA],
               lv[DB_LIVE_PRESENT_CUR_MA], lv[DB_LIVE_ERROR_CUR_MA],
               lv[DB_LIVE_PWM_DUTY]*100.0f,
               lv[DB_LIVE_POS_ADC], lv[DB_LIVE_CUR_ADC],
               (m>=0&&m<4)?mn[m]:"?", (unsigned long)lv[DB_LIVE_LOOP_COUNTER]);
    }else if(driver_board_poll(id)){
        printf("t=%10.1f ms | pos %4u SCS  cur %5d mA  (basic - old AT32 fw)\n",
               t_ms, driver_board_present_position(id), driver_board_present_current(id));
    }else printf("t=%10.1f ms | SPI poll failed\n", t_ms);
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
                for(int i=0; i<DB_LIVE_COUNT && ok; i++)
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

/* likewise: runs the all-leg Stretch bob (up/down) while streaming per-servo
 * tracking CSV, so you can PID-tune the vertical stretch motion. */
static void run_sstretch(const int *ids, int nids, int secs, int hz);

/* likewise: runs the in-place Jump (crouch/push/tuck/land) `reps` times while
 * streaming per-servo tracking CSV, so you can PID-tune the jump. */
static void run_sjump(const int *ids, int nids, int hz, int reps);

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
    int tid, thz = 0;
    int tn = sscanf(line,"trace %d %d",&tid,&thz);
    if(tn >= 1){
        if(tid<1 || tid>12){ printf("bad servo id\n"); return; }
        if(!CliMode){ printf("run 'cli on' first (gait would fight the SPI bus)\n"); return; }
        if(tn >= 2 && thz > 0){
            if(thz > 200) thz = 200;              /* SPI/GET_LIVE caps it anyway */
            TracePeriodMs = 1000u / (uint32_t)thz;
            if(TracePeriodMs < 1) TracePeriodMs = 1;
        }else{
            TracePeriodMs = 100;                  /* default 10 Hz */
        }
        TraceId = tid;
        printf("TRACE ON servo %d at ~%lu Hz - commands still work while it runs.\n"
               "stop with 'trace off' or press 'q' on an empty line\n",
               tid, (unsigned long)(1000u / TracePeriodMs));
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
                if(now - last_trace_ms >= TracePeriodMs){   /* rate set by 'trace <id> [hz]' */
                    last_trace_ms = now;
                    trace_print_line(TraceId);
                    fflush(stdout);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(TraceId ? 2 : 20));   /* poll fast while tracing */
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
static void fRIK(float x,float th0,float z){
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
        servo_flush(); //servo_flush();
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

#define ROOT_BUF_SZ 24000   /* room for the CLI dump output in the page */
static esp_err_t send_root(httpd_req_t *req){
    char *b = malloc(ROOT_BUF_SZ);
    if(!b) return ESP_ERR_NO_MEM;
    int n=0;
    #define A(...) n += snprintf(b+n, ROOT_BUF_SZ-n, __VA_ARGS__)
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
    A("<div style=\"margin:8px auto;\"><form action=\"/leg8\" method=\"get\" "
      "style=\"display:inline;\">Leg 8 pos (0-1023): "
      "<input type=\"number\" name=\"v\" value=\"%d\" min=\"0\" max=\"1023\" "
      "style=\"width:80px;height:34px;\"><button type=\"submit\" "
      "style=\"width:110px;background:%s;color:white;\">Set Leg 8</button>"
      "</form></div>", manual8_pos, manual8?"lime":"#555");
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
      "document.addEventListener('submit',function(e){"
      "var f=e.target;if(f.id=='clif')return;"   // CLI form has its own AJAX
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

// /leg8?v=NNN  -> hold servo 8 at raw SCS position NNN (0..1023, 511=centre)
static esp_err_t h_leg8(httpd_req_t*r){
    char q[32], val[8];
    if(httpd_req_get_url_query_str(r,q,sizeof q)==ESP_OK &&
       httpd_query_key_value(q,"v",val,sizeof val)==ESP_OK){
        int p=atoi(val);
        if(p<0) p=0;
        if(p>1023) p=1023;
        reset_all_modes();
        started_once=1;
        manual8_pos=(uint16_t)p;
        manual8=1;
    }
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
    for(int i=0; i<DB_LIVE_COUNT && live_ok; i++)
        live_ok = driver_board_get_live(id, i, &lv[i]);

    char b[560];
    if(live_ok){
        static const char *mn[] = {"IDLE","POSITION","TORQUE","IK"};
        int m = (int)lv[DB_LIVE_MODE];
        snprintf(b, sizeof b,
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
    }else if(driver_board_poll(id)){
        // old AT32 firmware without GET_LIVE: basic feedback only
        uint16_t p = driver_board_present_position(id);
        snprintf(b, sizeof b,
            "TRACE servo %d (basic - flash new AT32 fw for full trace)\n"
            "pos = %4u SCS  %6.1f deg raw\ncur = %4d mA\n",
            id, p, (float)p*270.0f/1024.0f, driver_board_present_current(id));
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

/* /api/live?id=N : live control-loop values (same set as 'trace') */
static esp_err_t h_api_live(httpd_req_t*r){
    int id = api_qint(r,"id",0);
    if(!CliMode)      return api_json(r,"{\"ok\":false,\"err\":\"climode\"}");
    if(id<1 || id>12) return api_json(r,"{\"ok\":false,\"err\":\"bad id\"}");
    float lv[DB_LIVE_COUNT]; bool ok=true;
    for(int i=0; i<DB_LIVE_COUNT && ok; i++)
        ok = driver_board_get_live(id, i, &lv[i]);
    char b[512];
    if(ok){
        snprintf(b,sizeof b,
            "{\"ok\":true,\"full\":true,\"pos_adc\":%g,\"cur_adc\":%g,"
            "\"set_deg\":%g,\"now_deg\":%g,\"err_deg\":%g,"
            "\"cap_ma\":%g,\"set_ma\":%g,\"now_ma\":%g,\"err_ma\":%g,"
            "\"duty\":%g,\"mode\":%d,\"loop\":%lu}",
            lv[DB_LIVE_POS_ADC], lv[DB_LIVE_CUR_ADC],
            lv[DB_LIVE_SETPOINT_POS_DEG], lv[DB_LIVE_PRESENT_POS_DEG],
            lv[DB_LIVE_ERROR_POS_DEG],
            lv[DB_LIVE_MAX_CURRENT_MA], lv[DB_LIVE_SETPOINT_CUR_MA],
            lv[DB_LIVE_PRESENT_CUR_MA], lv[DB_LIVE_ERROR_CUR_MA],
            lv[DB_LIVE_PWM_DUTY], (int)lv[DB_LIVE_MODE],
            (unsigned long)lv[DB_LIVE_LOOP_COUNTER]);
    }else if(driver_board_poll(id)){
        uint16_t p = driver_board_present_position(id);
        snprintf(b,sizeof b,
            "{\"ok\":true,\"full\":false,\"now_deg\":%g,\"now_ma\":%d}",
            (float)p*270.0f/1024.0f, driver_board_present_current(id));
    }else{
        snprintf(b,sizeof b,"{\"ok\":false,\"err\":\"spi\"}");
    }
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

static void start_webserver(void){
    httpd_handle_t s=NULL;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers=80;
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
    reg(s,"/mate",h_mate);
    reg(s,"/stanford",h_stanford);
    reg(s,"/climode",h_climode);
    reg(s,"/clicmd",h_clicmd);
    reg(s,"/clix",h_clix);
    reg(s,"/tracepoll",h_tracepoll);
    reg(s,"/js",h_js);
    reg(s,"/wizard",h_wizard);
    reg(s,"/api/status",h_api_status);
    reg(s,"/api/climode",h_api_climode);
    reg(s,"/api/dump",h_api_dump);
    reg(s,"/api/set",h_api_set);
    reg(s,"/api/live",h_api_live);
    reg(s,"/api/save",h_api_save);
    reg(s,"/api/restore",h_api_restore);
    reg(s,"/api/direct",h_api_direct);
    reg(s,"/api/scan",h_api_scan);
    reg(s,"/periodM",h_periodM); reg(s,"/periodP",h_periodP);
    reg(s,"/heightM",h_heightM); reg(s,"/heightP",h_heightP);
    reg(s,"/upHeightM",h_upM);   reg(s,"/upHeightP",h_upP);
    reg(s,"/strideM",h_strM);    reg(s,"/strideP",h_strP);
    reg(s,"/tiltM",h_tiltM);     reg(s,"/tiltP",h_tiltP);
    reg(s,"/sgsM",h_sgsM);       reg(s,"/sgsP",h_sgsP);
    reg(s,"/calReset",h_calReset);
    reg(s,"/leg8",h_leg8);
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
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); rLIK(-stride*cosf(tt),0,height-upHeight*sinf(tt));
                rRIK( stride*cosf(tt),0,height);                   fLIK( stride*cosf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); rLIK( stride*sinf(tt),0,height-upHeight*cosf(tt));
                rRIK(-stride*sinf(tt),0,height);                   fLIK(-stride*sinf(tt),0,height); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK( stride*cosf(tt),0,height);                   rLIK( stride*cosf(tt),0,height);
                rRIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); fLIK(-stride*cosf(tt),0,height-upHeight*sinf(tt)); servo_flush(); }
            time_mSt=millis(); tim=0;
            while(tim<period){ tim=millis()-time_mSt; tt=(float)(tim*PI/2/period);
                fRIK(-stride*sinf(tt),0,height);                   rLIK(-stride*sinf(tt),0,height);
                rRIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); fLIK( stride*sinf(tt),0,height-upHeight*cosf(tt)); servo_flush(); }
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

        }else if(manual8){
            // Hold a neutral stand, but drive servo 8 to the manually entered
            // position instead of the value the IK just computed.
            servo_speed_all(0);
            fRIK(0,0,height); rRIK(0,0,height); fLIK(0,0,height); rLIK(0,0,height);
            goal[8] = manual8_pos;   // override just servo 8
            servo_flush();
            vTaskDelay(1);

        }else if(!started_once){
            // POWER-ON POSE: stand at the calibrated `height` (x=0, so no
            // backward jerk - just the legs extending to the proper stand),
            // the SAME height Advance and the idle hold use, so boot is level
            // and matches every mode instead of sitting low at the 70mm centre.
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
    for(int i=1;i<=12;i++){ char k[12]; snprintf(k,sizeof k,"offset%d",i);
        offset[i]=nvs_get_float(k, offset[i]); }

    wifi_init_sta();
    start_webserver();

    xTaskCreatePinnedToCore(gait_task, "gait", 8192, NULL, 22, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 5, NULL, 0);
}