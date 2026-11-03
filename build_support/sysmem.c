#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <reent.h>
#include "cmsis_compiler.h"

extern unsigned char __heap_start__;
extern unsigned char __heap_end__;

void *_sbrk(ptrdiff_t increment)
{
    static uintptr_t current;
    const uintptr_t start = (uintptr_t)&__heap_start__;
    const uintptr_t limit = (uintptr_t)&__heap_end__;
    uintptr_t previous;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (current == 0)
    {
        current = start;
    }
    previous = current;

    if (increment >= 0)
    {
        if ((uintptr_t)increment > limit - current)
        {
            __set_PRIMASK(primask);
            errno = ENOMEM;
            return (void *)-1;
        }
        current += (uintptr_t)increment;
    }
    else
    {
        uintptr_t decrement = (uintptr_t)(-(increment + 1)) + 1U;
        if (decrement > current - start)
        {
            __set_PRIMASK(primask);
            errno = ENOMEM;
            return (void *)-1;
        }
        current -= decrement;
    }
    __set_PRIMASK(primask);
    return (void *)previous;
}

/* Newlib's allocator is shared by tasks; preserve nested interrupt masking. */
static uint32_t malloc_primask;
static unsigned int malloc_lock_depth;

void __malloc_lock(struct _reent *reent)
{
    uint32_t primask = __get_PRIMASK();
    (void)reent;
    __disable_irq();
    if (malloc_lock_depth++ == 0)
    {
        malloc_primask = primask;
    }
}

void __malloc_unlock(struct _reent *reent)
{
    (void)reent;
    if (--malloc_lock_depth == 0)
    {
        __set_PRIMASK(malloc_primask);
    }
}
