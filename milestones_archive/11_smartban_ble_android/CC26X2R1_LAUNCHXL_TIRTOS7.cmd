/*
 * ============================================================================
 * CC26X2R1_LAUNCHXL_TIRTOS7.cmd
 * Linker Command File for CC2652R1 LaunchPad with TI-RTOS7 & tiarmclang
 * ============================================================================
 */

--stack_size=0x800   /* C stack is also used for ISR stack (2048 bytes) */

HEAPSIZE = 0x8000;   /* Size of heap buffer used by HeapMem (32 KB) */

--retain "*(.resetVecs)"

/* Override default entry point. */
--entry_point ResetISR
/* Allow main() to take args */
--args 0x8
/* Suppress warnings and errors:
 * - 10063: Warning about entry point not being _c_int00
 * - 16011, 16012: 8-byte alignment errors.
 */
--diag_suppress=10063,16011,16012

/* Set severity of diagnostics to Remark instead of Warning
 * - 10068: Warning about no matching log_ptr* sections
 */
--diag_remark=10068

#define FLASH_BASE              0x0
#define FLASH_SIZE              0x58000
#define RAM_BASE                0x20000000
#define RAM_SIZE                0x14000
#define GPRAM_BASE              0x11000000
#define GPRAM_SIZE              0x2000

/* System memory map */
MEMORY
{
    /* Application stored in and executes from internal flash (352 KB) */
    FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE
    /* Application uses internal RAM for data (80 KB) */
    SRAM (RWX) : origin = RAM_BASE, length = RAM_SIZE
    /* Cache as RAM if disabled in CCFG */
    GPRAM (RWX): origin = GPRAM_BASE, length = GPRAM_SIZE

    LOG_DATA (R) : origin = 0x90000000, length = 0x40000
    LOG_PTR  (R) : origin = 0x94000008, length = 0x40000
}

/* Section allocation in memory */
SECTIONS
{
    .resetVecs      :   > FLASH_BASE
    .text           :   >> FLASH
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
    .const          :   >> FLASH
    .constdata      :   >> FLASH
    .rodata         :   >> FLASH
    .binit          :   > FLASH
    .cinit          :   > FLASH
    .pinit          :   > FLASH
    .init_array     :   > FLASH
    .emb_text       :   >> FLASH
    .ccfg           :   > FLASH (HIGH)

    .ramVecs        :   > SRAM, type = NOLOAD, ALIGN(256)
    .data           :   > SRAM
    .bss            :   > SRAM
    .sysmem         :   > SRAM
    .stack          :   > SRAM (HIGH)
    .nonretenvar    :   > SRAM
    /* Heap buffer used by HeapMem */
    .priheap   : {
        __primary_heap_start__ = .;
        . += HEAPSIZE;
        __primary_heap_end__ = .;
    } > SRAM align 8
    .gpram          :   > GPRAM
    .log_data       :   > LOG_DATA, type = COPY
    .log_ptr        : { *(.log_ptr*) } > LOG_PTR align 4, type = COPY
}

--symbol_map __TI_STACK_SIZE=__STACK_SIZE
--symbol_map __TI_STACK_BASE=__stack

-u_c_int00
