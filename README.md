# riscv32-simemu
 A simple minimal riscv32imac virtual machine, support Linux MMU+SMP booting.

![alt text](imgs/b8c441772f004cf114c82497b9674acd.png)

![alt text](imgs/ed4ec3515b25bb5677ba347c2649add3.png)

 - [x] RV32I Basic integer instruction
 - [x] RV32A atomic-instruction extension
 - [x] RV32M integer multiplication
 - [x] RV32C compressed instruction-set extension
 - [x] 8250 UART
 - [x] SV32 MMU
 - [x] SMP multithreading cpu/harts simulating supported
 - [x] virtio input keyboard and mouse
 - [x] virtio block disk
 - [x] simple framebuffer display
 

```
├─config     # kernel and buildroot config
├─firmware   # opensbi boot firmware
├─simemu     # emulator src
└─prebuild   # prebuild sbi firmware and linux image, 
               initramfs with buildroot contained.
```

SMP multi-process coremark test:

![alt text](imgs/8e46f8d7419711c76f030eb311958278.png)

![alt text](imgs/c39d6525b46c69ed5ab939da9877fefc.png)
