// src/bhi385_app.c
#include "bhi385_app.h"

#include "pico_i2c_bridge.h"
#include "verbose.h"

#include "bhi385.h"
#include "bhi385_parse.h"
#include "bhi385_event_data.h"
#include "bhi385_virtual_sensor_conf_param.h"

#include "firmware/bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BHI385_ADDR_0              UINT8_C(0x28)
#define BHI385_ADDR_1              UINT8_C(0x29)

// 先保守一点：保证在不同 Arduino/Wire 实现下都尽量不踩 buffer 限制
#define BHI385_RD_WR_LEN           UINT32_C(32)

#define WORK_BUFFER_SIZE           UINT32_C(2048)
#define BOOT_READY_TIMEOUT_MS      UINT32_C(1000)
#define BOOT_POLL_INTERVAL_MS      UINT32_C(10)

static struct bhi385_dev bhy;
static uint8_t dev_addr = BHI385_ADDR_0;
static uint8_t work_buffer[WORK_BUFFER_SIZE];

static int8_t platform_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    int rc = pico_i2c_bridge_read(addr, reg_addr, reg_data, len);
    return (rc == 0) ? 0 : 1; // 0=OK; 非0=失败 -> API 会映射成 BHI385_E_IO(-3)
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

static void parse_quaternion_data(const struct bhi385_fifo_parse_data_info *cb, void *ref)
{
    (void)ref;

    if (cb->data_size != 11) // 官方例程同样检查 11 bytes（含 sensor id）
    {
        return;
    }

    struct bhi385_event_data_quaternion q;
    bhi385_event_data_parse_quaternion(cb->data_ptr, &q);

    // 时间戳可选：如果你想和官方输出一致，可打开
    uint32_t s = 0, ns = 0;
    if (cb->time_stamp)
    {
        uint64_t t = *cb->time_stamp;
        t = t * 15625; // 官方 quaternion.c 做法：tick -> ns
        s  = (uint32_t)(t / UINT64_C(1000000000));
        ns = (uint32_t)(t - (uint64_t)s * UINT64_C(1000000000));
    }

    PRINT_D("SID:%u T:%lu.%09lu x:%f y:%f z:%f w:%f acc:%0.2f\r\n",
            cb->sensor_id,
            (unsigned long)s, (unsigned long)ns,
            q.x / 16384.0f,
            q.y / 16384.0f,
            q.z / 16384.0f,
            q.w / 16384.0f,
            ((q.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

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

    // 先用更低速确保信号完整性；稳定后再提到 400k/1MHz
    pico_i2c_bridge_init(400000);

    // 1) 探测地址 + 读 chip id
    rslt = bhi385_probe_and_init(BHI385_ADDR_0);
    if (rslt != BHI385_OK)
    {
        rslt = bhi385_probe_and_init(BHI385_ADDR_1);
    }
    if (rslt != BHI385_OK)
    {
        PRINT_E("BHI385 probe failed (rslt=%d). Check: VDDIO=1.8V + level shift + CS(HCSB) pulled high + wiring.\r\n", rslt);
        return;
    }

    // 2) Soft reset（如果这里 -3，说明“最基本 1byte 写寄存器”都失败）
    rslt = bhi385_soft_reset(&bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("bhi385_soft_reset failed, rslt=%d\r\n", rslt);
        return;
    }

    // 3) 等 Host Interface Ready（datasheet 建议 reset 后 poll 0x25 bit4）<!--citation:3-->
    rslt = bhi385_wait_host_if_ready(BOOT_READY_TIMEOUT_MS, &boot_status);
    if (rslt != BHI385_OK)
    {
        PRINT_E("Host IF not ready after %lu ms (rslt=%d, boot_status=0x%02X)\r\n",
                (unsigned long)BOOT_READY_TIMEOUT_MS, rslt, boot_status);
        return;
    }
    PRINT_D("Boot status=0x%02X (Host IF ready)\r\n", boot_status);

    // 4) Host interrupt / interface ctrl（和官方例程一致）<!--citation:2-->
    {
        uint8_t hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;
        rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
        if (rslt != BHI385_OK) { PRINT_E("set_host_interrupt_ctrl failed, rslt=%d\r\n", rslt); return; }

        rslt = bhi385_set_host_intf_ctrl(0, &bhy);
        if (rslt != BHI385_OK) { PRINT_E("set_host_intf_ctrl failed, rslt=%d\r\n", rslt); return; }
    }

    // 5) 上传固件 + boot from RAM
    PRINT_D("Uploading firmware...\r\n");
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), &bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("Firmware upload failed, rslt=%d\r\n", rslt);
        return;
    }

    #define BHI385_BST_FW_VERIFY_DONE   (1 << 5)
    #define BHI385_BST_FW_VERIFY_ERROR  (1 << 6)

    uint8_t bs = 0;
    rslt = bhi385_get_boot_status(&bs, &bhy);
    PRINT_D("Post-upload boot_status=0x%02X (VerifyDone=%d, VerifyError=%d)\r\n",
            bs,
            (bs & BHI385_BST_FW_VERIFY_DONE)  ? 1 : 0,
            (bs & BHI385_BST_FW_VERIFY_ERROR) ? 1 : 0);

    if (bs & BHI385_BST_FW_VERIFY_ERROR)
    {
        PRINT_E("Firmware CRC/signature verification FAILED. Check firmware image / upload integrity.\r\n");
        return;
    }
    if (!(bs & BHI385_BST_FW_VERIFY_DONE))
    {
        PRINT_E("Firmware verification not done yet (unexpected after upload returns OK).\r\n");
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
        // 这里不直接 return：有时固件起来慢，你也可以再加一次延时+重读
    }

    // 6) 注册回调 + 更新虚拟传感器列表
    (void)bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, bhi385_parse_meta_event, NULL, &bhy);
    (void)bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, bhi385_parse_meta_event, NULL, &bhy);
    (void)bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GAMERV, parse_quaternion_data, NULL, &bhy);

    rslt = bhi385_update_virtual_sensor_list(&bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("update_virtual_sensor_list failed, rslt=%d\r\n", rslt);
        return;
    }

    // 7) 使能 Game Rotation Vector（quaternion.c 同款）<!--citation:6-->
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf;
    memset(&sensor_conf, 0, sizeof(sensor_conf));
    sensor_conf.sample_rate = 100.0f;
    sensor_conf.latency = 0;

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GAMERV, &sensor_conf, &bhy);
    if (rslt != BHI385_OK)
    {
        PRINT_E("virtual_sensor_conf_param_set_cfg failed, rslt=%d\r\n", rslt);
        return;
    }

    PRINT_D("Game Rotation Vector enabled at %.2f Hz\r\n", sensor_conf.sample_rate);
}

void bhi385_app_loop(void)
{
    int8_t rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
    if (rslt != BHI385_OK)
    {
        // 不要刷屏，可以改成计数/间隔打印
        PRINT_E("get_and_process_fifo failed, rslt=%d\r\n", rslt);
    }
}