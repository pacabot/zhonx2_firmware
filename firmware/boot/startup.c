#include <stdint.h>
extern uint32_t _stack, _sidata, _sdata, _edata, _sbss, _ebss;
int boot_main(void);
void Reset_Handler(void);
static void fault(void) { for (;;) {} }
__attribute__((section(".isr_vector"),used))
const uintptr_t vectors[16] = {
 (uintptr_t)&_stack, (uintptr_t)Reset_Handler, (uintptr_t)fault, (uintptr_t)fault,
 (uintptr_t)fault, (uintptr_t)fault, (uintptr_t)fault, 0,0,0,0,
 (uintptr_t)fault, (uintptr_t)fault, 0, (uintptr_t)fault, (uintptr_t)fault
};
void Reset_Handler(void)
{
 uint32_t *s=&_sidata;
 for (uint32_t *p=&_sdata;p<&_edata;) *p++=*s++;
 for (uint32_t *p=&_sbss;p<&_ebss;) *p++=0;
 boot_main();
 for (;;) {}
}
