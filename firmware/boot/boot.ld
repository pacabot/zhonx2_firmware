ENTRY(Reset_Handler)
MEMORY {
 rom (rx) : ORIGIN = 0x08000000, LENGTH = 16K
 ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}
SECTIONS {
 .text : { KEEP(*(.isr_vector)) *(.text*) *(.rodata*) } > rom
 .ARM.exidx : { *(.ARM.exidx*) } > rom
 .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > ram AT>rom
 _sidata = LOADADDR(.data);
 .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(8); _ebss = .; } > ram
 _stack = ORIGIN(ram) + LENGTH(ram);
 ASSERT(_ebss + 8192 <= _stack, "Bootloader stack overflow")
}
