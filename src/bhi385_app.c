#include "bhi385_app.h"
#include "pico_i2c_bridge.h"
#include "bhi385.h"
#include "bhi385_defs.h"
#include "bhi385_parse.h"
#include "bhi385_event_data.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "verbose.h"
#include "firmware/bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"
#include <string.h>

/* 先用 I2C Scanner 确认实际地址，再改这里！ */
#define BHI385_I2C_ADDR     0x28
#define BHI385_RD_WR_LEN    256
#define WORK_BUFFER_SIZE    2048

static struct bhi385_dev bhy;
static uint8_t dev_addr = BHI385_I2C_ADDR;
static uint8_t work_buffer[WORK_BUFFER_SIZE];

/* ---------------- 平台适配层 ---------------- */

static int8_t platform_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    return (pico_i2c_bridge_read(addr, reg_addr, reg_data, len) == 0) ? 0 : -1;
}

static int8_t platform_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;
    return (pico_i2c_bridge_write(addr, reg_addr, reg_data, len) == 0) ? 0 : -1;
}

static void platform_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    pico_delay_us(period_us);
}

/* ---------------- 统一错误打印（模仿官方 common.c 的 print_api_error 风格） ---------------- */

static bool check_rslt(const char *api_name, int8_t rslt)
{
    if (rslt != BHI385_OK) {
        PRINT_E("%s failed, rslt=%d (-3=BHI385_E_IO 总线无应答，请检查硬件接线：VDD/VDDIO/CS(HCSB)/RESET/上拉电阻)\r\n",
                api_name, rslt);
        return false;
    }
    return true;
}

/* ---------------- 四元数解析回调 ---------------- */

static void parse_quaternion_data(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi385_event_data_quaternion data;

    if (callback_info->data_size != 11) {
        return;
    }

    bhi385_event_data_parse_quaternion(callback_info->data_ptr, &data);

    PRINT_D("SID:%u x:%f y:%f z:%f w:%f acc:%.2f\r\n",
            callback_info->sensor_id,
            data.x / 16384.0f,
            data.y / 16384.0f,
            data.z / 16384.0f,
            data.w / 16384.0f,
            ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

/* ---------------- 主初始化流程（对齐官方 quaternion.c 顺序） ---------------- */

void bhi385_app_init(void)
{
    int8_t rslt;
    uint16_t version = 0;
    uint8_t chip_id = 0;

    pico_i2c_bridge_init(400000); /* Shuttle board 支持最高3.4MHz I2C，这里先用400k更稳 */

    /* 硬件自检：开机先扫一遍总线，确认设备真的在应答 */
    pico_i2c_bridge_scan();

    rslt = bhi385_init(BHI385_I2C_INTERFACE,
                        platform_i2c_read,
                        platform_i2c_write,
                        platform_delay_us,
                        BHI385_RD_WR_LEN,
                        &dev_addr,
                        &bhy);
    if (!check_rslt("bhi385_init", rslt)) return;
    PRINT_D("BHI385 init OK\r\n");

    /* 1. 软复位，使芯片进入已知状态 */
    rslt = bhi385_soft_reset(&bhy);
    if (!check_rslt("bhi385_soft_reset", rslt)) return;

    /* 2. 复位后立刻验证通信是否仍然正常（chip id） */
    rslt = bhi385_get_chip_id(&chip_id, &bhy);
    if (!check_rslt("bhi385_get_chip_id", rslt)) return;
    PRINT_D("chip_id=0x%02X\r\n", chip_id);

    /* 3. 上传固件到RAM（内部会自行轮询 boot status，等待 Host Interface Ready） */
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), &bhy);
    if (!check_rslt("bhi385_upload_firmware_to_ram", rslt)) return;

    /* 4. 从RAM启动 */
    rslt = bhi385_boot_from_ram(&bhy);
    if (!check_rslt("bhi385_boot_from_ram", rslt)) return;

    /* 5. 验证固件已跑起来 */
    rslt = bhi385_get_kernel_version(&version, &bhy);
    if (rslt == BHI385_OK && version != 0) {
        PRINT_D("Boot successful. Kernel version %u.\r\n", version);
    } else {
        PRINT_E("get_kernel_version failed or version=0, rslt=%d\r\n", rslt);
        return;
    }

    /* 6. 注册虚拟传感器回调 */
    rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GAMERV, parse_quaternion_data, NULL, &bhy);
    if (!check_rslt("bhi385_register_fifo_parse_callback", rslt)) return;

    rslt = bhi385_update_virtual_sensor_list(&bhy);
    if (!check_rslt("bhi385_update_virtual_sensor_list", rslt)) return;

    /* 7. 固件启动完成后，再配置host interrupt ctrl / intf ctrl（顺序对齐官方示例：
          启动前不要动这些寄存器，等固件跑起来后再配置FIFO/中断行为） */
    uint8_t hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;
    rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    check_rslt("bhi385_set_host_interrupt_ctrl", rslt);

    rslt = bhi385_set_host_intf_ctrl(0, &bhy);
    check_rslt("bhi385_set_host_intf_ctrl", rslt);

    /* 8. 配置Game Rotation Vector采样率 */
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    sensor_conf.sample_rate = 100.0f;
    sensor_conf.latency = 0;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GAMERV, &sensor_conf, &bhy);
    if (!check_rslt("bhi385_virtual_sensor_conf_param_set_cfg", rslt)) return;

    PRINT_D("Game Rotation Vector enabled at %.2fHz, streaming...\r\n", sensor_conf.sample_rate);
}

void bhi385_app_loop(void)
{
    bhi385_get_and_process_fifo(work_buffer, sizeof(work_buffer), &bhy);
}