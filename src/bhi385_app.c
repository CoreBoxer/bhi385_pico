// src/bhi385_app.c
#include "bhi385_app.h"

#include "pico_i2c_bridge.h"
#include "verbose.h"

#include "bhi385.h"
#include "bhi385_defs.h"
#include "bhi385_parse.h"
#include "bhi385_event_data.h"
#include "bhi385_virtual_sensor_conf_param.h"

/* 切换为 ndof 固件：带 BMM350 磁力计驱动，才能枚举出 34 号 Rotation Vector */
#include "firmware/bhi385/Bosch_Shuttle3_BHI385_BMM350_BMP58X_BME688_bsxsam_ndof.fw.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BHI385_ADDR_0              UINT8_C(0x28)
#define BHI385_ADDR_1              UINT8_C(0x29)

#define BHI385_RD_WR_LEN           UINT32_C(32)

#define WORK_BUFFER_SIZE           UINT32_C(2048)
#define BOOT_READY_TIMEOUT_MS      UINT32_C(1000)
#define BOOT_POLL_INTERVAL_MS      UINT32_C(10)

#define BHI385_BST_FW_VERIFY_DONE   (1 << 5)
#define BHI385_BST_FW_VERIFY_ERROR  (1 << 6)

static struct bhi385_dev bhy;
static uint8_t dev_addr = BHI385_ADDR_0;
static uint8_t work_buffer[WORK_BUFFER_SIZE];

static bool pressure_first_sample_seen = false;

static int8_t platform_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    int rc = pico_i2c_bridge_read(addr, reg_addr, reg_data, len);
    return (rc == 0) ? 0 : 1;
}

static int8_t platform_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    int rc = pico_i2c_bridge_write(addr, reg_addr, reg_data, len);
    return (rc == 0) ? 0 : 1;
}

static void platform_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    pico_delay_us(period_us);
}

/* ---------------- 34号 Rotation Vector 解析：payload格式与 Game RV 完全一致 ---------------- */
static void parse_rotation_vector_data(const struct bhi385_fifo_parse_data_info *cb, void *ref)
{
    (void)ref;

    if (cb->data_size != 11) /* x,y,z,w(各2字节) + accuracy(2字节) + sensor_id(1字节) */
    {
        return;
    }

    struct bhi385_event_data_quaternion q;
    bhi385_event_data_parse_quaternion(cb->data_ptr, &q);

    uint32_t s = 0, ns = 0;
    if (cb->time_stamp)
    {
        uint64_t t = *cb->time_stamp;
        t = t * 15625;
        s  = (uint32_t)(t / UINT64_C(1000000000));
        ns = (uint32_t)(t - (uint64_t)s * UINT64_C(1000000000));
    }

    PRINT_D("SID:%u(RV) T:%lu.%09lu x:%f y:%f z:%f w:%f acc:%0.2f\r\n",
            cb->sensor_id,
            (unsigned long)s, (unsigned long)ns,
            q.x / 16384.0f,
            q.y / 16384.0f,
            q.z / 16384.0f,
            q.w / 16384.0f,
            ((q.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

/* ---------------- 150号 气压：ndof固件同样存在，保留 ---------------- */
static void parse_bmp_pressure_data(const struct bhi385_fifo_parse_data_info *cb, void *ref)
{
    (void)ref;

    if (!cb || cb->data_size != 4) /* 3字节payload + 1字节sensor_id */
    {
        return;
    }

    const float scaling_factor = 1.0f / 128.0f;
    uint32_t data = BHI385_LE2U24(cb->data_ptr);
    float pressure_pa = (float)data * scaling_factor; /* 单位是 Pa */

    if (!pressure_first_sample_seen)
    {
        pressure_first_sample_seen = true;
        PRINT_D("Pressure: discarding first sample after enable (raw=%f Pa)\r\n", pressure_pa);
        return;
    }

    if (pressure_pa < 30000.0f || pressure_pa > 120000.0f)
    {
        PRINT_E("Pressure: out-of-range sample ignored (%f Pa)\r\n", pressure_pa);
        return;
    }

    uint32_t s = 0, ns = 0;
    if (cb->time_stamp)
    {
        uint64_t timestamp = *cb->time_stamp * 15625;
        s  = (uint32_t)(timestamp / UINT64_C(1000000000));
        ns = (uint32_t)(timestamp - (uint64_t)s * UINT64_C(1000000000));
    }

    PRINT_D("SID:%u T:%lu.%09lu Pressure: %f Pa (%.2f hPa)\r\n",
            cb->sensor_id, (unsigned long)s, (unsigned long)ns,
            pressure_pa, pressure_pa / 100.0f);
}

/* 注：ndof固件里没有138号BMP Temperature，只有128号Temperature(来自BME688)，
   payload格式大概率与BMP不同（位宽/缩放系数需另行确认），这里暂不注册，
   避免"注册成功但收不到数据"的困惑。如需启用请先确认128号的实际数据格式。 */

static int8_t bhi385_wait_host_if_ready(uint32_t timeout_ms, uint8_t *boot_status_out)
{
    int8_t rslt;
    uint8_t bs = 0;

    const uint32_t tries = (timeout_ms + BOOT_POLL_INTERVAL_MS - 1) / BOOT_POLL_INTERVAL_MS;
    for (uint32_t i = 0; i < tries; i++)
    {
        rslt = bhi385_get_boot_status(&bs, &bhy);
        if (rslt == BHI385_OK)
        {
            if (bs & BHI385_BST_HOST_INTERFACE_READY)
            {
                if (boot_status_out) { *boot_status_out = bs; }
                return BHI385_OK;
            }
        }

        platform_delay_us(BOOT_POLL_INTERVAL_MS * 1000, NULL);
    }

    if (boot_status_out) { *boot_status_out = bs; }
    return BHI385_E_TIMEOUT;
}

static int8_t bhi385_probe_and_init(uint8_t addr)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    dev_addr = addr;

    rslt = bhi385_init(BHI385_I2C_INTERFACE,
                       platform_i2c_read,
                       platform_i2c_write,
                       platform_delay_us,
                       BHI385_RD_WR_LEN,
                       &dev_addr,
                       &bhy);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    rslt = bhi385_get_chip_id(&chip_id, &bhy);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    if (chip_id != BHI385_CHIP_ID)
    {
        PRINT_E("Addr 0x%02X replied but chip_id=0x%02X (expect 0x%02X)\r\n",
                addr, chip_id, BHI385_CHIP_ID);
        return BHI385_E_INVALID_DATA;
    }

    PRINT_D("BHI385 detected at 0x%02X, chip_id=0x%02X\r\n", addr, chip_id);
    return BHI385_OK;
}

void bhi385_app_init(void)
{
    int8_t rslt;
    uint8_t boot_status = 0;
    uint16_t kernel_ver = 0;

    pico_i2c_bridge_init(1000000); /* 固件上传阶段用高速 */

    rslt = bhi385_probe_and_init(BHI385_ADDR_0);
    if (rslt != BHI385_OK)
    {
        rslt = bhi385_probe_and_init(BHI385_ADDR_1);
    }
    if (rslt != BHI385_OK)
    {
        PRINT_E("BHI385 probe failed (rslt=%d).\r\n", rslt);
        return;
    }

    rslt = bhi385_soft_reset(&bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("bhi385_soft_reset failed, rslt=%d\r\n", rslt);
        return;
    }

    rslt = bhi385_wait_host_if_ready(BOOT_READY_TIMEOUT_MS, &boot_status);
    if (rslt != BHI385_OK)
    {
        PRINT_E("Host IF not ready after %lu ms (rslt=%d, boot_status=0x%02X)\r\n",
                (unsigned long)BOOT_READY_TIMEOUT_MS, rslt, boot_status);
        return;
    }
    PRINT_D("Boot status=0x%02X (Host IF ready)\r\n", boot_status);

    {
        uint8_t hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;
        rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
        if (rslt != BHI385_OK) { PRINT_E("set_host_interrupt_ctrl failed, rslt=%d\r\n", rslt); return; }

        rslt = bhi385_set_host_intf_ctrl(0, &bhy);
        if (rslt != BHI385_OK) { PRINT_E("set_host_intf_ctrl failed, rslt=%d\r\n", rslt); return; }
    }

    PRINT_D("Uploading firmware (ndof: BMM350+BMP58X+BME688+bsxsam)...\r\n");
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), &bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("Firmware upload failed, rslt=%d\r\n", rslt);
        return;
    }

    uint8_t bs = 0;
    rslt = bhi385_get_boot_status(&bs, &bhy);
    PRINT_D("Post-upload boot_status=0x%02X (VerifyDone=%d, VerifyError=%d)\r\n",
            bs,
            (bs & BHI385_BST_FW_VERIFY_DONE)  ? 1 : 0,
            (bs & BHI385_BST_FW_VERIFY_ERROR) ? 1 : 0);

    if (bs & BHI385_BST_FW_VERIFY_ERROR)
    {
        PRINT_E("Firmware CRC/signature verification FAILED.\r\n");
        return;
    }
    if (!(bs & BHI385_BST_FW_VERIFY_DONE))
    {
        PRINT_E("Firmware verification not done yet.\r\n");
        return;
    }

    PRINT_D("Booting from RAM...\r\n");
    rslt = bhi385_boot_from_ram(&bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("boot_from_ram failed, rslt=%d\r\n", rslt);
        return;
    }

    rslt = bhi385_get_kernel_version(&kernel_ver, &bhy);
    if (rslt == BHI385_OK && kernel_ver != 0)
    {
        PRINT_D("Boot OK. Kernel version=%u\r\n", kernel_ver);
    }
    else
    {
        PRINT_E("Kernel version read failed or zero (rslt=%d, ver=%u)\r\n", rslt, kernel_ver);
    }

    /* 注册回调：系统事件 + 34号Rotation Vector + 150号气压 */
    (void)bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, bhi385_parse_meta_event, NULL, &bhy);
    (void)bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, bhi385_parse_meta_event, NULL, &bhy);
    (void)bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_RV, parse_rotation_vector_data, NULL, &bhy);
    (void)bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_PRESSURE, parse_bmp_pressure_data, NULL, &bhy);

    rslt = bhi385_update_virtual_sensor_list(&bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("update_virtual_sensor_list failed, rslt=%d\r\n", rslt);
        return;
    }

    struct bhi385_virtual_sensor_conf_param_conf sensor_conf;
    memset(&sensor_conf, 0, sizeof(sensor_conf));

    /* 34号 Rotation Vector：9轴融合，含绝对航向 */
    sensor_conf.sample_rate = 100.0f;
    sensor_conf.latency = 0;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_RV, &sensor_conf, &bhy);
    if (rslt != BHI385_OK) { PRINT_E("Enable RV failed, rslt=%d\r\n", rslt); return; }
    PRINT_D("Rotation Vector enabled at %.2f Hz\r\n", sensor_conf.sample_rate);

    /* 150号 气压 */
    sensor_conf.sample_rate = 10.0f;
    sensor_conf.latency = 0;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_PRESSURE, &sensor_conf, &bhy);
    if (rslt != BHI385_OK) { PRINT_E("Enable PRESSURE failed, rslt=%d\r\n", rslt); return; }
    PRINT_D("Pressure enabled at %.2f Hz\r\n", sensor_conf.sample_rate);
}

void bhi385_app_loop(void)
{
    int8_t rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("get_and_process_fifo failed, rslt=%d\r\n", rslt);
    }
}