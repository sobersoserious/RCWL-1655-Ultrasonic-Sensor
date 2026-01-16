# RCWL-1655 超声波传感器驱动（Linux 5.15 / QCS8550）

## 📌项目简介

本仓库提供了一个**基于Linux5.15内核的RCWL-1655超声波传感器驱动**，适配**Qualcomm QCS8550平台**。该驱动以**I2C字符设备驱动**的形式实现，使用**工作队列**周期轮询测量数据，同时通过**sysfs**暴露测量周期和原始距离数据。也可在用户空间通过标准文件接口（`read/poll`）获取距离测量数据。

驱动已在**Linux5.15**环境中完成调试验证。

---

## ✨ 特性（Features）

* ✅ 支持RCWL-1655超声波传感器
* ✅ 基于**I2C总线**通信
* ✅ Linux **Character Device**接口
* ✅ 使用工作队列定时轮询测量数据
* ✅ 使用**sysfs**暴露参数：
  * `poll_interval`：轮询周期（ms）
  * `distance_raw`：原始距离值
* ✅ 支持阻塞/非阻塞`read()`
* ✅ 支持`poll/select`机制
* ✅ 适配**Qualcomm QCS8550**
* ✅ 使用DeviceTree进行硬件描述
* ✅ 支持模块方式加载（`.ko`）

---

## 🧩 硬件与软件环境

### 硬件平台

* SoC：**Qualcomm QCS8550**
* 传感器：**RCWL-1655超声波传感器**
* 接口：I2C
* 电压：3.3V-5V

### 软件环境

* LinuxKernel：**5.15（Qualcomm BSP）**
* 编译器：aarch64-linux-gnu-gcc
* 构建方式：KernelModule(Out-of-tree)

---

## 🌲 DeviceTree配置示例

```dts
&i2c2 {
    status="okay";

    rcwl1655@57 {
        compatible="rcwl,rcwl1655";
        reg=<0x57>;
        status="okay";
    };
};
```

>⚠️ 请根据实际硬件修改I2C控制器节点及设备地址

---

## ⚠️ 注意事项

* 若出现`SDA被拉低`的情况，大概率是传感器正处于测量忙状态
* 本驱动采用固定频率轮询读取的方式，驱动返回`"rcwl1655 i2c_master_recv error"`代表此次read时传感器还未准备好数据(`SDA被拉低`)，属于正常现象。

---

## 👤 作者

* Author: sober
* Platform: Qualcomm QCS8550
* Kernel: Linux5.15

欢迎Issue/PR/交流讨论🙌
