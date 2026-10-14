#include "pico_i2c_bridge.h"
#include <Arduino.h>
#include <Wire.h>

// 请根据你实际接线修改引脚号（Pico默认Wire0是 GP4=SDA, GP5=SCL）
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

/*
 * I2C总线恢复程序：当从机（BHI385）在传输过程中意外复位，
 * 可能会把SDA卡在低电平，导致整条总线永久失联。
 * 标准恢复方法：手动产生最多9个时钟脉冲，强制从机释放SDA，
 * 再手动产生一个STOP条件，最后重新初始化Wire。
 */
static void i2c_bus_recovery()
{
    Wire.end();

    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);

    // 如果SDA被从机拉低，手动打9个时钟，强迫它在某个时钟沿释放总线
    for (int i = 0; i < 9; i++) {
        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            break; // SDA已经释放，提前结束
        }
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
    }

    // 手动产生一个STOP条件：SCL高电平期间，SDA从低变高
    pinMode(I2C_SDA_PIN, OUTPUT);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(5);

    // 恢复成I2C功能并重新初始化
    Wire.begin();
    Wire.setClock(400000);

    Serial.println("[I2C] 总线恢复程序已执行");
}

extern "C" int pico_i2c_bridge_init(uint32_t clock_hz)
{
    Wire.begin();
    Wire.setClock(clock_hz);
    return 0;
}

extern "C" int pico_i2c_bridge_write(uint8_t dev_addr, uint8_t reg_addr,
                                      const uint8_t *data, uint32_t len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (len > 0 && data != nullptr) {
        Wire.write(data, (size_t)len);
    }
    int result = (Wire.endTransmission(true) == 0) ? 0 : -1;

    if (result != 0) {
        // 写入失败，很可能是总线被卡死，立即尝试恢复
        i2c_bus_recovery();
    }
    return result;
}

extern "C" int pico_i2c_bridge_read(uint8_t dev_addr, uint8_t reg_addr,
                                     uint8_t *data, uint32_t len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        i2c_bus_recovery();
        return -1;
    }
    uint32_t received = Wire.requestFrom((int)dev_addr, (int)len);
    if (received != len) {
        i2c_bus_recovery();
        return -1;
    }
    for (uint32_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return 0;
}

extern "C" void pico_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}
extern "C" void pico_i2c_bridge_scan(void)
{
    Serial.println("[I2C] 开始扫描总线...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] 发现设备，地址=0x");
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[I2C] 未扫描到任何设备！请检查供电/CS(HCSB)/复位/上拉电阻等硬件连接。");
    } else {
        Serial.print("[I2C] 扫描完成，共发现设备数量");
        Serial.println(found);
    }
}