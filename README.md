## 工程说明
SDK11,适用于app无蓝牙的支持OTA的bootloader程序,对应软件协议栈 s132_nrf52_2.0.0_softdevice.hex

## 对应APP操作
```c
    //跳转到bootloader
    NRF_POWER->GPREGRET = 0xffffffff;  
    NRF_POWER->GPREGRET = 0xB1;      			 // write 
    SEGGER_RTT_printf(0, "\r\nnrf_pwr_mgmt_shutdown\r\n");		
    NVIC_SystemReset();
```

## 操作流程
0. 擦除FLASH
1. 写入三个固件 
2. 写APP标志

```c
//写 flash 中的一个 APP 有效标志位，
nrfjprog --memwr 0x0007F000 --val 0x01 --verify -f NRF52 
```
3. 复位

```c
nrfjprog -r -f NRF52
```
