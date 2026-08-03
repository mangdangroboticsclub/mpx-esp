/* driver_board.c  -  SPI backend for the Mini Pupper controller driver board.
 *
 * Ported from minipupper2pro/esp32 (SERVOS_BY_SPI path of mini_pupper_servos.cpp),
 * stripped down to exactly what the ESP-only gait code needs:
 *   - one-shot init (bus + 4 devices + power pin)
 *   - sync write of 12 positions + 12 current limits
 *   - cached feedback (present position + present current)
 */
#include "driver_board.h"

#include <string.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"      /* WORD_ALIGNED_ATTR */
#include "esp_rom_sys.h"   /* esp_rom_delay_us */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DRVBOARD"

/* ---- pin map (same as reference board) ---- */
#define SPI_MASTER_ID    SPI2_HOST
#define SPI_MASTER_MOSI  11
#define SPI_MASTER_MISO  13
#define SPI_MASTER_CLK   12
#define SPI_MASTER_CS0   10   /* left  front */
#define SPI_MASTER_CS1   9    /* right front */
#define SPI_MASTER_CS2   14   /* left  rear  */
#define SPI_MASTER_CS3   21   /* right rear  */
#define POWER_EN_GPIO    8

/* board index -> the CS GPIO that board's device actually drives.
 * Must stay in step with the switch in spi_xfer() / spi_xfer_any(). */
static int db_cs_gpio(int board)
{
    switch (board) {
        case 0:  return SPI_MASTER_CS1;   /* servos 1-3   FR */
        case 1:  return SPI_MASTER_CS0;   /* servos 4-6   FL */
        case 2:  return SPI_MASTER_CS3;   /* servos 7-9   RR */
        default: return SPI_MASTER_CS2;   /* servos 10-12 RL */
    }
}

/* ---- on-the-wire protocol (identical to AT32 spi_command_frame) ---- */
#define START_FIELD  0xA5A5
#define MODE_FIELD   0x0001
#define MODE_POSITION 0x0001   /* AT32: position control, "torque" field = max current mA */

#pragma pack(push,1)
typedef struct { uint16_t mode, position; int16_t torque; uint16_t kp, kd; } servo_cmd_sub_t;
typedef struct { uint16_t start, mode; servo_cmd_sub_t s1, s2, s3; uint16_t check_sum; } host_SMS_t;

/* res1 carries the NTC temperature in signed 0.1 degC (253 = 25.3 degC).
 * res2 is still unused. Same 10-byte layout as before. */
typedef struct { uint16_t status, position; int16_t torque; uint16_t res1, res2; } servo_fb_sub_t;
typedef struct { uint16_t start, status; servo_fb_sub_t s1, s2, s3; uint16_t check_sum; } SMS_host_t;
#pragma pack(pop)

#if !DB_POWER_ONLY
static spi_device_handle_t dev_left_front, dev_right_front, dev_left_rear, dev_right_rear;
#endif

/* cached feedback, index 0 == servo ID 1 */
static uint16_t fb_position[12];
static int16_t  fb_current[12];
static int16_t  fb_temp_dc[12];   /* NTC temperature, 0.1 degC */

/* shadow of the last commanded state per servo (deci-degrees), so a direct
 * single-servo write can resend the board frame without disturbing the
 * other two servos on the same board */
static uint16_t sh_mode[12]  = { [0 ... 11] = MODE_POSITION };
static uint16_t sh_posdd[12] = { [0 ... 11] = 1350 };   /* 135.0 deg centre */
static int16_t  sh_cur[12]   = { 0 };

/* board index (0..3) -> SPI device. Matches reference spi_read_write_bytes(). */
static esp_err_t spi_xfer(uint8_t board, uint8_t size, uint8_t *tx, uint8_t *rx)
{
#if DB_POWER_ONLY
    /* power-only bench mode: the SPI bus was never initialised */
    (void)size; (void)tx; (void)rx; (void)board;
    return ESP_ERR_INVALID_STATE;
#else
    spi_transaction_t t;
    /* board not fitted (see DB_BOARD_MASK) - skip quietly */
    if (!db_board_present(board)) return ESP_ERR_NOT_FOUND;
    memset(&t, 0, sizeof(t));
    t.length    = (size_t)size * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    switch (board) {
        case 0: return spi_device_transmit(dev_right_front, &t); /* servos 1-3  FR */
        case 1: return spi_device_transmit(dev_left_front,  &t); /* servos 4-6  FL */
        case 2: return spi_device_transmit(dev_right_rear,  &t); /* servos 7-9  RR */
        case 3: return spi_device_transmit(dev_left_rear,   &t); /* servos 10-12 RL */
        default: return ESP_FAIL;
    }
#endif /* DB_POWER_ONLY */
}

#if !DB_POWER_ONLY
/* Same as spi_xfer() but ignores DB_BOARD_MASK, so the bring-up scan can
 * poke every CS line even when only one board is declared present. */
static esp_err_t spi_xfer_any(uint8_t board, uint8_t size, uint8_t *tx, uint8_t *rx)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length    = (size_t)size * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    switch (board) {
        case 0: return spi_device_transmit(dev_right_front, &t);
        case 1: return spi_device_transmit(dev_left_front,  &t);
        case 2: return spi_device_transmit(dev_right_rear,  &t);
        case 3: return spi_device_transmit(dev_left_rear,   &t);
        default: return ESP_FAIL;
    }
}
#endif /* !DB_POWER_ONLY */

void driver_board_power(bool on)
{
    gpio_set_level(POWER_EN_GPIO, on ? 1 : 0);
}

void driver_board_init(void)
{
    /* power-enable pin */
    gpio_config_t io = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << POWER_EN_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    driver_board_power(false);

#if DB_POWER_ONLY
    /* Bench bring-up: hold the servo rail on and stop there. The SPI bus is
     * deliberately NOT initialised, so MOSI/MISO/CLK/CS stay high-Z and the
     * AT32 UART CLI owns the servos.
     * (app_main already waits 1 s after this call for the rails to settle.) */
    driver_board_power(true);
    ESP_LOGW(TAG, "POWER-ONLY MODE: servo rail ON (GPIO %d), SPI disabled.",
             POWER_EN_GPIO);
    ESP_LOGW(TAG, "Drive the servos from the AT32 UART CLI (PA9/PA10, H1 pins 6/7). "
                  "Set DB_POWER_ONLY to 0 in driver_board.h to hand control back to the ESP.");
    return;
#else

    /* SPI bus */
    spi_bus_config_t bus = {
        .mosi_io_num   = SPI_MASTER_MOSI,
        .miso_io_num   = SPI_MASTER_MISO,
        .sclk_io_num   = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_MASTER_ID, &bus, SPI_DMA_CH_AUTO));

    /* 4 driver-board devices (12 MHz, mode 0) */
    spi_device_interface_config_t dev = {
        .mode           = 0,
        .duty_cycle_pos = 128,
        .clock_speed_hz = 12 * 1000 * 1000,   /* 15 MHz (15 MHz max) */
        .queue_size     = 2,
    };
    dev.spics_io_num = SPI_MASTER_CS0; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_left_front));
    dev.spics_io_num = SPI_MASTER_CS1; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_right_front));
    dev.spics_io_num = SPI_MASTER_CS2; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_left_rear));
    dev.spics_io_num = SPI_MASTER_CS3; ESP_ERROR_CHECK(spi_bus_add_device(SPI_MASTER_ID, &dev, &dev_right_rear));

    driver_board_power(true);
    ESP_LOGI(TAG, "driver board SPI init OK (mask 0x%X, %d board(s), %d servos active)",
             DB_BOARD_MASK, db_board_count(), db_board_count() * 3);
#if DB_BOARD_MASK != 0x0F
    for (int b = 0; b < 4; b++)
        if (db_board_present(b))
            ESP_LOGW(TAG, "BENCH MODE: board %d driven (CS GPIO %d, servos %d-%d). "
                          "Set DB_BOARD_MASK to 0x0F for the full robot.",
                     b, db_cs_gpio(b), b * 3 + 1, b * 3 + 3);
#endif
#endif /* DB_POWER_ONLY */
}

void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12])
{
    /* SPI DMA needs word-aligned buffers; these structs are #pragma pack(1)
     * so the compiler would otherwise be free to put them on an odd offset. */
    WORD_ALIGNED_ATTR host_SMS_t frame;
    WORD_ALIGNED_ATTR SMS_host_t rx;

    for (int i = 0; i < 4; i++) {
        if (!db_board_present(i)) continue;    /* board not fitted */
        const int b = i * 3;   /* first servo index of this board */
        frame.start = START_FIELD;
        frame.mode  = MODE_FIELD;

        servo_cmd_sub_t *sub[3] = { &frame.s1, &frame.s2, &frame.s3 };
        for (int j = 0; j < 3; j++) {
            int idx = b + j;                     /* PHYSICAL channel index */
            int L   = db_phys(idx + 1) - 1;      /* LOGICAL servo feeding it */
            sub[j]->mode = MODE_POSITION;
            /* SCS 0..1023 -> AT32 deci-degrees 0..2700, with the same global
             * direction flip the reference uses. Position already carries the
             * gait's per-servo calibration offset (applied in servo_write).
             * pos[]/cur_mA[] are indexed by logical servo, so pull the logical
             * servo mapped to this physical channel (board-variant swap). */
            sub[j]->position = (uint16_t)(2700 - (uint32_t)pos[L] * 2700u / 1024u);
            sub[j]->torque   = (int16_t)cur_mA[L];   /* MODE_POSITION => max current (mA) */
            sub[j]->kp = 0;
            sub[j]->kd = 0;
            sh_mode[idx]  = MODE_POSITION;             /* keep shadow in sync */
            sh_posdd[idx] = sub[j]->position;
            sh_cur[idx]   = sub[j]->torque;
        }
        frame.check_sum = 0;

        if (spi_xfer((uint8_t)i, sizeof(host_SMS_t), (uint8_t *)&frame, (uint8_t *)&rx) == ESP_OK) {
            servo_fb_sub_t *fb[3] = { &rx.s1, &rx.s2, &rx.s3 };
            for (int j = 0; j < 3; j++) {
                fb_position[b + j] = (uint16_t)((uint32_t)fb[j]->position * 1024u / 2700u);
                fb_current[b + j]  = fb[j]->torque;   /* present motor current, mA */
                fb_temp_dc[b + j]  = (int16_t)fb[j]->res1;  /* NTC temp, 0.1 degC */
            }
        }
    }
}

/* ---- AT32 sms_config parameter access (config frames, start 0xC0DE) ---- */
#define START_CONFIG    0xC0DE
#define CFG_OP_NOP      0
#define CFG_OP_SET      1
#define CFG_OP_GET      2
#define CFG_OP_SAVE     3
#define CFG_OP_RESTORE  4
#define CFG_OP_GET_LIVE 5

#pragma pack(push,1)
typedef struct {
    uint16_t start, op, servo_index, param_id;
    float    value;
    uint8_t  padding[24];   /* pad to sizeof(host_SMS_t) = 36 bytes */
} cfg_frame_t;
#pragma pack(pop)
_Static_assert(sizeof(cfg_frame_t) == sizeof(host_SMS_t), "cfg frame size");

static const char *param_names[DB_PARAM_COUNT] = {
    "reverse_position_sensor", "min_position_adc", "max_position_adc",
    "range_position_deg", "reverse_motor", "kp_position", "kd_position",
    "kp_current", "kff_current", "max_pwm_duty_cycle",
};

const char *driver_board_param_name(int param_id)
{
    if (param_id < 0 || param_id >= DB_PARAM_COUNT) return NULL;
    return param_names[param_id];
}

int driver_board_param_id(const char *name)
{
    for (int i = 0; i < DB_PARAM_COUNT; i++)
        if (strcmp(name, param_names[i]) == 0) return i;
    return -1;
}

static bool cfg_xfer(uint8_t board, uint16_t op, uint16_t servo_index,
                     uint16_t param_id, float value, SMS_host_t *rx_out)
{
    WORD_ALIGNED_ATTR cfg_frame_t f;
    WORD_ALIGNED_ATTR SMS_host_t rx;
    memset(&f, 0, sizeof f);
    f.start = START_CONFIG;
    f.op = op; f.servo_index = servo_index;
    f.param_id = param_id; f.value = value;
    if (spi_xfer(board, sizeof f, (uint8_t *)&f, (uint8_t *)&rx) != ESP_OK)
        return false;
    if (rx_out) *rx_out = rx;
    return true;
}

/* The AT32 loads its reply into the frame clocked out on the NEXT
 * transaction, so every request is a request/NOP transaction pair. */
static bool cfg_request(uint8_t board, uint16_t op, uint16_t servo_index,
                        uint16_t param_id, float value, float *out)
{
    SMS_host_t rx;
    if (!cfg_xfer(board, op, servo_index, param_id, value, NULL)) return false;
    esp_rom_delay_us(500);                       /* let the AT32 IRQ run   */
    if (!cfg_xfer(board, CFG_OP_NOP, 0, 0, 0, &rx)) return false;
    if (rx.status != START_CONFIG) return false; /* not a config response  */
    if (rx.s1.position != param_id) return false;/* echo mismatch          */
    /* config responses put the float across res1+res2 (4 bytes, little
     * endian) - unchanged by the temperature field, which only applies to
     * SERVO feedback frames, not config responses. */
    if (out) memcpy(out, &rx.s1.res1, sizeof(float));
    return true;
}

bool driver_board_set_param(int servo, int param_id, float value)
{
    if (servo < 1 || servo > 12 || param_id < 0 || param_id >= DB_PARAM_COUNT)
        return false;
    int p = db_phys(servo) - 1;                 /* logical -> physical */
    return cfg_request((uint8_t)(p / 3), CFG_OP_SET,
                       (uint16_t)(p % 3), (uint16_t)param_id,
                       value, NULL);
}

bool driver_board_get_param(int servo, int param_id, float *out)
{
    if (servo < 1 || servo > 12 || param_id < 0 || param_id >= DB_PARAM_COUNT)
        return false;
    int p = db_phys(servo) - 1;                 /* logical -> physical */
    return cfg_request((uint8_t)(p / 3), CFG_OP_GET,
                       (uint16_t)(p % 3), (uint16_t)param_id,
                       0, out);
}

bool driver_board_get_live(int servo, int live_id, float *out)
{
    if (servo < 1 || servo > 12 || live_id < 0 || live_id >= DB_LIVE_COUNT)
        return false;
    int p = db_phys(servo) - 1;                 /* logical -> physical */
    return cfg_request((uint8_t)(p / 3), CFG_OP_GET_LIVE,
                       (uint16_t)(p % 3), (uint16_t)live_id,
                       0, out);
}

static bool cfg_board_op(int board, uint16_t op)
{
    if (board >= 0 && board <= 3)
        return db_board_present(board) &&
               cfg_request((uint8_t)board, op, 0, 0, 0, NULL);
    bool ok = true;                              /* board == -1: all boards */
    for (int b = 0; b < 4; b++)
        if (db_board_present(b))
            ok &= cfg_request((uint8_t)b, op, 0, 0, 0, NULL);
    return ok;
}

bool driver_board_save_config(int board)    { return cfg_board_op(board, CFG_OP_SAVE); }
bool driver_board_factory_restore(int board){ return cfg_board_op(board, CFG_OP_RESTORE); }

/* Send one board's frame rebuilt from the shadow, refresh feedback cache. */
static bool board_resend(int board)
{
    int b = board * 3;
    WORD_ALIGNED_ATTR host_SMS_t frame;
    WORD_ALIGNED_ATTR SMS_host_t rx;
    frame.start = START_FIELD;
    frame.mode  = MODE_FIELD;
    servo_cmd_sub_t *sub[3] = { &frame.s1, &frame.s2, &frame.s3 };
    for (int j = 0; j < 3; j++) {
        sub[j]->mode     = sh_mode[b + j];
        sub[j]->position = sh_posdd[b + j];
        sub[j]->torque   = sh_cur[b + j];
        sub[j]->kp = 0;
        sub[j]->kd = 0;
    }
    frame.check_sum = 0;

    if (spi_xfer((uint8_t)board, sizeof frame, (uint8_t *)&frame, (uint8_t *)&rx) != ESP_OK)
        return false;

    servo_fb_sub_t *fb[3] = { &rx.s1, &rx.s2, &rx.s3 };
    for (int j = 0; j < 3; j++) {
        fb_position[b + j] = (uint16_t)((uint32_t)fb[j]->position * 1024u / 2700u);
        fb_current[b + j]  = fb[j]->torque;
        fb_temp_dc[b + j]  = (int16_t)fb[j]->res1;
    }
    return true;
}

bool driver_board_direct(int servo, uint16_t mode, float pos_deg, int16_t current_mA)
{
    if (servo < 1 || servo > 12) return false;
    if (pos_deg < 0)   pos_deg = 0;
    if (pos_deg > 270) pos_deg = 270;

    int idx = db_phys(servo) - 1;                  /* logical -> physical */
    sh_mode[idx]  = mode;
    sh_posdd[idx] = (uint16_t)(pos_deg * 10.0f);   /* RAW angle, no flip */
    sh_cur[idx]   = current_mA;
    return board_resend(idx / 3);
}

bool driver_board_poll(int servo)
{
    if (servo < 1 || servo > 12) return false;
    return board_resend((db_phys(servo) - 1) / 3);
}

int16_t driver_board_present_current(int ch)
{
    if (ch < 1 || ch > 12) return 0;
    return fb_current[db_phys(ch) - 1];
}

uint16_t driver_board_present_position(int ch)
{
    if (ch < 1 || ch > 12) return 0;
    return fb_position[db_phys(ch) - 1];
}

float driver_board_present_temperature(int ch)
{
    if (ch < 1 || ch > 12) return 0.0f;
    return (float)fb_temp_dc[db_phys(ch) - 1] / 10.0f;
}

/* ---- bring-up scan ------------------------------------------------------
 * Probes ALL FOUR chip selects (not just the fitted ones) with a harmless
 * IDLE frame and reports what comes back on MISO. A healthy 3IN1 answers:
 *     start = 0xFEED   status = 0x0003   check_sum = 0xDEAD
 * Nothing moves: mode is IDLE and the current limit is 0.
 *
 * Everything here is deliberately written the dumb, defensive way:
 *   - all scratch is static, not stack. The console task has a small stack
 *     and this function needs ~450 bytes of buffers plus whatever vsnprintf
 *     wants underneath it.
 *   - static also guarantees the DMA buffers sit in DRAM and stay 4-byte
 *     aligned, which spi_device_transmit() requires.
 *   - the reply is decoded with explicit byte loads (rd16), never by
 *     reading fields of the #pragma pack(1) structs. Sub-word access to a
 *     mis-derived pointer is what raises LoadStoreError on the ESP32-S3,
 *     so this path simply never does one.
 * Not reentrant - it is a one-at-a-time bring-up tool.                    */
#if !DB_POWER_ONLY

static WORD_ALIGNED_ATTR host_SMS_t  scan_tx;
static WORD_ALIGNED_ATTR cfg_frame_t scan_cfg;
static WORD_ALIGNED_ATTR SMS_host_t  scan_rx;
static char scan_line[224];          /* holds the 36-byte hex dump: 8 + 108 */
static char scan_hex[3 * 36 + 1];

/* little-endian 16-bit read, byte at a time - no alignment assumptions */
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* byte offset of servo j's feedback subframe inside SMS_host_t */
#define FB_BASE(j)  (4 + 10 * (j))
#define FB_POS(j)   (FB_BASE(j) + 2)
#define FB_CUR(j)   (FB_BASE(j) + 4)
#define FB_RES1(j)  (FB_BASE(j) + 6)
#define FB_START    0
#define FB_STATUS   2
#define FB_CKSUM    34

#endif /* !DB_POWER_ONLY */

void driver_board_scan(void (*out)(const char *))
{
#define SCAN_OUT(...) do{ snprintf(scan_line,sizeof scan_line,__VA_ARGS__); \
                          if(out) out(scan_line); \
                          else ESP_LOGI(TAG,"%s",scan_line); }while(0)

#if DB_POWER_ONLY
    static char scan_line[128];
    SCAN_OUT("DB_POWER_ONLY is 1 - the SPI bus was never initialised.\n");
    SCAN_OUT("Set it to 0 in driver_board.h and re-flash before scanning.\n");
#else
    const uint8_t *r = (const uint8_t *)&scan_rx;

    SCAN_OUT("SPI scan: CLK=%d MOSI=%d MISO=%d, mode 0, 36-byte frame\n",
             SPI_MASTER_CLK, SPI_MASTER_MOSI, SPI_MASTER_MISO);
    SCAN_OUT("expect start=FEED status=0003 cksum=DEAD from a live board\n\n");

    for (int b = 0; b < 4; b++) {
        /* harmless command frame: idle mode, zero current limit */
        memset(&scan_tx, 0, sizeof scan_tx);
        scan_tx.start = START_FIELD;
        scan_tx.mode  = MODE_FIELD;
        scan_tx.s1.mode = DB_MODE_IDLE;
        scan_tx.s2.mode = DB_MODE_IDLE;
        scan_tx.s3.mode = DB_MODE_IDLE;
        scan_tx.s1.position = sh_posdd[b * 3 + 0];
        scan_tx.s2.position = sh_posdd[b * 3 + 1];
        scan_tx.s3.position = sh_posdd[b * 3 + 2];

        /* The AT32 builds its reply inside the NSS-rising IRQ, so the answer
         * to frame N is clocked out during frame N+1. Send twice, keep #2. */
        esp_err_t e1 = spi_xfer_any((uint8_t)b, sizeof scan_tx,
                                    (uint8_t *)&scan_tx, (uint8_t *)&scan_rx);
        esp_rom_delay_us(500);
        memset(&scan_rx, 0, sizeof scan_rx);
        esp_err_t e2 = spi_xfer_any((uint8_t)b, sizeof scan_tx,
                                    (uint8_t *)&scan_tx, (uint8_t *)&scan_rx);

        SCAN_OUT("--- board %d  (CS = GPIO %d, servos %d-%d) ---\n",
                 b, db_cs_gpio(b), b * 3 + 1, b * 3 + 3);

        if (e1 != ESP_OK || e2 != ESP_OK) {
            SCAN_OUT("  transmit failed: %s / %s\n",
                     esp_err_to_name(e1), esp_err_to_name(e2));
            continue;
        }

        int n = 0;
        for (int i = 0; i < 36; i++)
            n += snprintf(scan_hex + n, sizeof scan_hex - (unsigned)n, "%02X ", r[i]);
        SCAN_OUT("  raw : %s\n", scan_hex);

        unsigned start  = rd16(r + FB_START);
        unsigned status = rd16(r + FB_STATUS);
        unsigned cksum  = rd16(r + FB_CKSUM);
        SCAN_OUT("  hdr : start=%04X status=%04X cksum=%04X\n", start, status, cksum);

        int all_ff = 1, all_00 = 1;
        for (int i = 0; i < 36; i++) {
            if (r[i] != 0xFF) all_ff = 0;
            if (r[i] != 0x00) all_00 = 0;
        }

        if (all_ff) {
            SCAN_OUT("  -> ALL 0xFF: MISO floating high.\n");
            SCAN_OUT("     Check CN1 pin 6 -> ESP GPIO %d, and that this board\n",
                     SPI_MASTER_MISO);
            SCAN_OUT("     really is the one on CS GPIO %d.\n", db_cs_gpio(b));
            SCAN_OUT("\n");
            continue;
        }
        if (all_00) {
            SCAN_OUT("  -> ALL 0x00: MISO stuck low, board unpowered, or AT32 in reset.\n");
            SCAN_OUT("\n");
            continue;
        }
        if (start != 0xFEED || cksum != 0xDEAD) {
            int shift = -1;
            for (int i = 0; i < 35; i++)
                if (r[i] == 0xED && r[i + 1] == 0xFE) { shift = i; break; }
            if (shift >= 0) {
                SCAN_OUT("  -> 0xFEED found at byte %d, not 0: frame is SHIFTED.\n", shift);
                SCAN_OUT("     Drop clock_speed_hz (try 1 MHz) before anything else.\n");
            } else {
                SCAN_OUT("  -> garbage. Likely MOSI/MISO swapped, no common ground,\n");
                SCAN_OUT("     or the clock is too fast for hand-soldered wiring.\n");
            }
            SCAN_OUT("\n");
            continue;
        }

        SCAN_OUT("  -> MISO/SCK/CS OK, board is answering.\n");
        for (int j = 0; j < 3; j++) {
            unsigned pos = rd16(r + FB_POS(j));
            int      cur = (int16_t)rd16(r + FB_CUR(j));
            int      td  = (int16_t)rd16(r + FB_RES1(j));   /* temp, 0.1 degC */
            int      tw  = td / 10, tf = td % 10;
            if (tf < 0) tf = -tf;
            SCAN_OUT("     servo %2d: pos %u.%u deg  cur %d mA  temp %d.%d C\n",
                     b * 3 + j + 1, pos / 10u, pos % 10u, cur, tw, tf);
        }

        /* ---- MOSI proof --------------------------------------------------
         * A good reply only proves the board can TALK. It says nothing about
         * whether it HEARD us - and a dead MOSI looks exactly like this:
         * healthy feedback, but the AT32 decodes an all-zero command frame,
         * so mode = IDLE and no servo ever moves. The config GET path echoes
         * param_id and servo_index straight back, so it is a clean loopback. */
        memset(&scan_cfg, 0, sizeof scan_cfg);
        scan_cfg.start       = START_CONFIG;
        scan_cfg.op          = CFG_OP_GET;
        scan_cfg.servo_index = 2;     /* distinctive, not 0 */
        scan_cfg.param_id    = 3;     /* range_position_deg */
        spi_xfer_any((uint8_t)b, sizeof scan_cfg,
                     (uint8_t *)&scan_cfg, (uint8_t *)&scan_rx);
        esp_rom_delay_us(500);

        memset(&scan_cfg, 0, sizeof scan_cfg);
        scan_cfg.start = START_CONFIG;            /* NOP, just clocks the reply out */
        memset(&scan_rx, 0, sizeof scan_rx);
        spi_xfer_any((uint8_t)b, sizeof scan_cfg,
                     (uint8_t *)&scan_cfg, (uint8_t *)&scan_rx);

        unsigned echo_status = rd16(r + FB_STATUS);
        unsigned echo_param  = rd16(r + FB_POS(0));
        int      echo_index  = (int16_t)rd16(r + FB_CUR(0));

        if (echo_status == START_CONFIG && echo_param == 3 && echo_index == 2) {
            SCAN_OUT("     MOSI echo OK. Both directions work.\n");
        } else {
            SCAN_OUT("     *** MOSI ECHO FAILED (status=%04X param=%u index=%d)\n",
                     echo_status, echo_param, echo_index);
            SCAN_OUT("     The board TALKS but does not HEAR you: MOSI is the fault.\n");
            SCAN_OUT("     Check CN1 pin 4 -> ESP GPIO %d.\n", SPI_MASTER_MOSI);
            SCAN_OUT("     On the old 4IN1 pin 4 was MISO, so a harness reused by\n");
            SCAN_OUT("     pin position has MOSI and MISO crossed.\n");
        }
        SCAN_OUT("\n");
    }
#endif
#undef SCAN_OUT
}

/* ---- pin walk for multimeter tracing ------------------------------------
 * Detaches SCK, MOSI and the four CS lines from the SPI peripheral and
 * drives them as plain GPIO, one at a time: every pin idles HIGH, then the
 * pin under test is held LOW for hold_s seconds. Probe the driver-board
 * connector while it runs - exactly one pin should read ~0 V at a time, and
 * that tells you which ESP GPIO each wire really lands on.
 *
 * MISO is not in the walk: it is an input on this side, driven by the AT32.
 * The `scan` command already proves it end to end.
 *
 * DESTRUCTIVE: once the pins are detached the SPI driver can no longer
 * reach the boards. Reboot to put them back ('reboot' in the CLI).       */
#define PINTEST_STEPS 6

static int pintest_gpio(int step)
{
    switch (step) {
        case 0:  return SPI_MASTER_CS1;   /* board 0 */
        case 1:  return SPI_MASTER_CS0;   /* board 1 */
        case 2:  return SPI_MASTER_CS3;   /* board 2 */
        case 3:  return SPI_MASTER_CS2;   /* board 3 */
        case 4:  return SPI_MASTER_CLK;
        default: return SPI_MASTER_MOSI;
    }
}

static const char *pintest_label(int step)
{
    switch (step) {
        case 0:  return "CS board 0 (servos 1-3)   -> that board's CN1 pin 5";
        case 1:  return "CS board 1 (servos 4-6)   -> that board's CN1 pin 5";
        case 2:  return "CS board 2 (servos 7-9)   -> that board's CN1 pin 5";
        case 3:  return "CS board 3 (servos 10-12) -> that board's CN1 pin 5";
        case 4:  return "SCK  (shared)             -> every CN1 pin 3";
        default: return "MOSI (shared)             -> every CN1 pin 4";
    }
}

static char pintest_line[160];

void driver_board_pintest(void (*out)(const char *), int hold_s)
{
#define PT_OUT(...) do{ snprintf(pintest_line,sizeof pintest_line,__VA_ARGS__); \
                        if(out) out(pintest_line); \
                        else ESP_LOGI(TAG,"%s",pintest_line); }while(0)

    if (hold_s < 1)  hold_s = 3;
    if (hold_s > 30) hold_s = 30;

    /* take every pin in the walk away from the SPI peripheral and idle it
     * HIGH (CS idles high, and a high SCK/MOSI is harmless while no CS is
     * asserted, so nothing on the AT32 side sees a transaction) */
    for (int s = 0; s < PINTEST_STEPS; s++) {
        int g = pintest_gpio(s);
        gpio_reset_pin(g);
        gpio_set_direction(g, GPIO_MODE_OUTPUT);
        gpio_set_level(g, 1);
    }

    PT_OUT("pin walk: each line held LOW for %d s, all others HIGH.\n", hold_s);
    PT_OUT("probe the connector - exactly one pin reads ~0 V at a time.\n");
    PT_OUT("(black lead on CN1 pin 2 = GND)\n\n");

    for (int s = 0; s < PINTEST_STEPS; s++) {
        int g = pintest_gpio(s);
        PT_OUT("[%d/%d] GPIO %-2d LOW for %ds - measure now : %s\n",
               s + 1, PINTEST_STEPS, g, hold_s, pintest_label(s));
        gpio_set_level(g, 0);
        vTaskDelay(pdMS_TO_TICKS(hold_s * 1000));
        gpio_set_level(g, 1);
    }

    PT_OUT("\nall pins back HIGH.\n");
    PT_OUT("SPI is detached from these pins - type 'reboot' to restore it.\n");
#undef PT_OUT
}
