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
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us */

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
    /* bench mode: boards beyond DB_BOARD_COUNT are not fitted - skip quietly */
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
    ESP_LOGI(TAG, "driver board SPI init OK (%d board(s) fitted, %d servos active)",
             DB_BOARD_COUNT, DB_BOARD_COUNT * 3);
#if DB_BOARD_COUNT < 4
    ESP_LOGW(TAG, "BENCH MODE: only board 0 (CS GPIO %d, servos 1-3) is driven. "
                  "Set DB_BOARD_COUNT to 4 in driver_board.h for the full robot.",
             SPI_MASTER_CS1);
#endif
#endif /* DB_POWER_ONLY */
}

void driver_board_sync_write(const uint16_t pos[12], const uint16_t cur_mA[12])
{
    host_SMS_t frame;
    SMS_host_t rx;

    for (int i = 0; i < DB_BOARD_COUNT; i++) {
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
    cfg_frame_t f;
    SMS_host_t rx;
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
    for (int b = 0; b < DB_BOARD_COUNT; b++)
        ok &= cfg_request((uint8_t)b, op, 0, 0, 0, NULL);
    return ok;
}

bool driver_board_save_config(int board)    { return cfg_board_op(board, CFG_OP_SAVE); }
bool driver_board_factory_restore(int board){ return cfg_board_op(board, CFG_OP_RESTORE); }

/* Send one board's frame rebuilt from the shadow, refresh feedback cache. */
static bool board_resend(int board)
{
    int b = board * 3;
    host_SMS_t frame;
    SMS_host_t rx;
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
