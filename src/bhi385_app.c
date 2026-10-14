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

#define BHI385_I2C_ADDR     0x28   // 先用 I2C Scanner 确认实际地址
#define BHI385_RD_WR_LEN    256    // 单次最大读写长度，I2C下常用256字节

static struct bhi385_dev bhy;
static uint8_t dev_addr = BHI385_I2C_ADDR;
static uint8_t work_buffer[2048];

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

static void parse_quaternion_data(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi385_event_data_quaternion data;

    if (callback_info->data_size != 11) /* 校验payload长度，含Sensor ID字节 */
    {
        return;
    }

    /* 直接用官方解析函数，不再自己手写memcpy偏移，减少出错概率 */
    bhi385_event_data_parse_quaternion(callback_info->data_ptr, &data);

    PRINT_D("SID:%u x:%f y:%f z:%f w:%f acc:%.2f\r\n",
            callback_info->sensor_id,
            data.x / 16384.0f,
            data.y / 16384.0f,
            data.z / 16384.0f,
            data.w / 16384.0f,
            ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

void bhi385_app_init(void)
{
    int8_t rslt;
    uint16_t version = 0;
    uint8_t boot_status = 0;

    pico_i2c_bridge_init(400000);

    /*
     * 关键修正：bhi385_init() 完整参数列表（参考BHI360同架构真实案例验证）：
     *   intf, read_fptr, write_fptr, delay_us_fptr,
     *   read_write_len,   <-- 之前遗漏的参数！
     *   intf_ptr,         <-- 指向保存I2C地址的变量
     *   dev
     */
    rslt = bhi385_init(BHI385_I2C_INTERFACE,
                        platform_i2c_read,
                        platform_i2c_write,
                        platform_delay_us,
                        BHI385_RD_WR_LEN,
                        &dev_addr,
                        &bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("bhi385_init failed, rslt=%d\r\n", rslt);
        return;
    }
    PRINT_D("BHI385 init OK, chip id verified (0x7C)\r\n");

    bhi385_soft_reset(&bhy);
    pico_delay_us(10000);

    rslt = bhi385_get_boot_status(&boot_status, &bhy);
    if (rslt != BHI385_OK || !(boot_status & BHI385_BST_HOST_INTERFACE_READY)) {
        PRINT_E("Host interface not ready, boot_status=0x%02X\r\n", boot_status);
        return;
    }

    /* 修正：固件数组变量名是 bhi385_firmware_image，不是 bhy_firmware_image */
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), &bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("firmware upload failed, rslt=%d\r\n", rslt);
        return;
    }

    rslt = bhi385_boot_from_ram(&bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("boot_from_ram failed, rslt=%d\r\n", rslt);
        return;
    }

    rslt = bhi385_get_kernel_version(&version, &bhy);
    if (rslt == BHI385_OK && version != 0) {
        PRINT_D("Boot successful. Kernel version %u.\r\n", version);
    }

    /* 注册四元数回调；BHI385_SENSOR_ID_GAMERV = 游戏旋转矢量（不含地磁校正），
       若想用含地磁校正的完整旋转矢量，换成 BHI385_SENSOR_ID_RV 即可 */
    rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GAMERV, parse_quaternion_data, NULL, &bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("register_fifo_parse_callback failed, rslt=%d\r\n", rslt);
        return;
    }

    rslt = bhi385_update_virtual_sensor_list(&bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("update_virtual_sensor_list failed, rslt=%d\r\n", rslt);
        return;
    }

    /* 修正：没有 bhi385_set_virt_sensor_cfg() 这个函数，
       正确做法是填充 conf 结构体，调用 bhi385_virtual_sensor_conf_param_set_cfg() */
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    sensor_conf.sample_rate = 100.0f;   /* 100Hz */
    sensor_conf.latency = 0;            /* 立即上报 */

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GAMERV, &sensor_conf, &bhy);
    if (rslt != BHI385_OK) {
        PRINT_E("set_cfg failed, rslt=%d\r\n", rslt);
        return;
    }

    PRINT_D("Game Rotation Vector enabled at %.2fHz, streaming...\r\n", sensor_conf.sample_rate);
}

void bhi385_app_loop(void)
{
    bhi385_get_and_process_fifo(work_buffer, sizeof(work_buffer), &bhy);
}