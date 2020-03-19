#include "macAddOns.h"

#include <mach/mach_init.h>
#include <mach/thread_policy.h>

#include <stddef.h>
#include <string.h>

int sched_setscheduler(__attribute__ ((unused)) pid_t pid, int policy, __attribute__ ((unused)) const struct sched_param* param)
{
int ret;

struct task_category_policy tcatpolicy;

if(10 <= policy)
	{
	tcatpolicy.role = TASK_FOREGROUND_APPLICATION;
	}
else if(3 <= policy)
	{
	tcatpolicy.role = TASK_UNSPECIFIED;
	}
else
	{
	tcatpolicy.role = TASK_BACKGROUND_APPLICATION;
	}

if ((ret=task_policy_set(mach_task_self(), TASK_CATEGORY_POLICY, (task_policy_t)&tcatpolicy, TASK_CATEGORY_POLICY_COUNT)) != KERN_SUCCESS)
	{return -1;}

return 0;
}


#ifndef _LIBC
# define __builtin_expect(expr, val)   (expr)
#endif

#undef memmem

/* Return the first occurrence of NEEDLE in HAYSTACK.  */
void *
memmem (const void* haystack, size_t haystack_len, const void* needle, size_t needle_len)
{
  const char *begin;
  const char *const last_possible
    = (const char *) haystack + haystack_len - needle_len;

  if (needle_len == 0)
    /* The first occurrence of the empty string is deemed to occur at
       the beginning of the string.  */
    return (void *) haystack;

  /* Sanity check, otherwise the loop might search through the whole
     memory.  */
  if (__builtin_expect (haystack_len < needle_len, 0))
    return NULL;

  for (begin = (const char *) haystack; begin <= last_possible; ++begin)
    if (begin[0] == ((const char *) needle)[0] &&
	!memcmp ((const void *) &begin[1],
		 (const void *) ((const char *) needle + 1),
		 needle_len - 1))
      return (void *) begin;

  return NULL;
}

