# riscv32-simemu
 A simple minimal riscv32imac virtual machine, support Linux MMU+SMP booting.

![alt text](imgs/b8c441772f004cf114c82497b9674acd.png)

 - [x] RV32I Basic integer instruction
 - [x] RV32A atomic-instruction extension
 - [x] RV32M integer multiplication
 - [x] RV32C compressed instruction-set extension
 - [x] 8250 UART
 - [x] SV32 MMU
 - [x] SMP muilt-cpu/harts supported
 

```
├─config     # kernel and buildroot config
├─firmware   # opensbi boot firmware
├─simemu     # emulator src
└─prebuild   # prebuild sbi firmware and linux image, 
               initramfs with buildroot contained.
```


### bugs
  SMP may deadlock in muilt-threading cpu threads.