

upper = '''
/dts-v1/;

/ {
	#address-cells = <1>;
	#size-cells = <1>;
	compatible = "riscv-minimal";
	model = "riscv-minimal";

	chosen {
		bootargs = "earlycon=uart8250,mmio,0x10000000,115200n8 console=ttyS0 ignore_loglevel root=/dev/vda";
		//bootargs = "earlycon=uart8250,mmio,0x10000000,115200n8 console=tty0 ignore_loglevel  root=/dev/vda";
		// bootargs = "earlycon=sbi console=ttyS0 ignore_loglevel ";
		/*  earlycon=uart8250,mmio,0x10000000,1000000  */ 
		// stdout-path = &uart0;
		rng-seed = <0xa164c316 0xb7f2d9f9 0x9817e840 0x8ece459e 0xcd0c79a3 0xc6a4af73 0x9e455d12 0x6c3db362>;
		// rng-seed = <1 2 3 4 5 6 7 8>;
		
		#address-cells = <2>;
		#size-cells = <2>;
		ranges;

		stdout-path = "framebuffer0";

		framebuffer0: framebuffer@40000000 {
			compatible = "simple-framebuffer";
			reg = <0 0x40000000 0 (1280 * 760 * 4)>;
			width = <1280>;
			height = <760>;
			stride = <(1280 * 4)>;
			format = "a8b8g8r8";
			status = "okay";
		};

	};

	memory@80000000 {
		device_type = "memory";
		reg = <0x80000000 0x0FF00000> ;
	};
 
	poweroff {
		value = <0x5555>;
		offset = <0x00>;
		regmap = <0x06>;
		compatible = "syscon-poweroff";
	};

	reboot {
		value = <0x7777>;
		offset = <0x00>;
		regmap = <0x06>;
		compatible = "syscon-reboot";
	};

'''

soc_start = '''
	soc {
		#address-cells = <1>;
		#size-cells = <1>;
		compatible = "simple-bus";
		ranges;
'''


soc_end = '''

        virtio_input@50000000 {
			compatible = "virtio,mmio";
			reg = <0x50000000 0x1000>;
			interrupt-parent = <&plic0>;
			interrupts = <1>;
        };
        
        virtio_block@60000000 {
			compatible = "virtio,mmio";
			reg = <0x60000000 0x1000>;
			interrupt-parent = <&plic0>;
			interrupts = <2>;
        };
		
		uart0: uart@10000000 {
			clock-frequency = <0x1000000>;
			reg = <0x10000000 0x100>;
			compatible = "ns16850"; 
			interrupt-parent = <&plic0>;
			interrupts = <10>;
			status = "okay";
		};

	};
};

'''

def append_cpu(nr_cpus):
    t = '''
	cpus {
		#address-cells = <1>;
		#size-cells = <0>;
		timebase-frequency = <1000000>;
'''
    for i in range(nr_cpus):
        t += '''
		cpu%d: cpu@%d { 
			device_type = "cpu";
			reg = <%d>;
			status = "okay";
			compatible = "riscv";
			riscv,isa = "rv32imac"; 
			mmu-type = "riscv,sv32";

			cpu%d_intc: interrupt-controller {
				#interrupt-cells = <1>; 
				#address-cells = <1>;
				interrupt-controller;
				compatible = "riscv,cpu-intc";  
			};
		};
        ''' % (i,i,i,i)
    
    t+= "};"
    return t

def append_clint(nr_cpus):
    t = '''
		clint0: timer@20000000 {
			interrupts-extended = 
'''
    for i in range(nr_cpus - 1):
        t += "                        <&cpu%d_intc 3>, <&cpu%d_intc 7>,\n" % (i,i)
    t += "                        <&cpu%d_intc 3>, <&cpu%d_intc 7>;\n" % (nr_cpus - 1,nr_cpus - 1)

    t += '''
		reg = <0x20000000 0x10000>;
		compatible =  "riscv,clint0";
	};
'''
    return t


def append_plic(nr_cpus):
    t = '''
		plic0: interrupt-controller@30000000 {
			#address-cells = <0>;
			#interrupt-cells = <1>;
			compatible = "sifive,plic-1.0.0", "riscv,plic0";
			reg = <0x30000000 0x4000000>;
			interrupt-controller;
			interrupts-extended = //11=MEXTI  9=SEXTI, 
'''
    for i in range(nr_cpus - 1):
        t += "                        <&cpu%d_intc 11>, <&cpu%d_intc 9>,\n" % (i,i)
    t += "                        <&cpu%d_intc 11>, <&cpu%d_intc 9>;\n" % (nr_cpus - 1,nr_cpus - 1)

    t += '''
			riscv,ndev = <16>;
		};
'''
    return t

nr_cpus = 4
dts = upper + append_cpu(nr_cpus) + soc_start + append_clint(nr_cpus) + append_plic(nr_cpus) +soc_end 
with open("simemu-mmu.dts", "w+") as f:
    f.write(dts)
print(dts)
