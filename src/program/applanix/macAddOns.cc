#include <string.h>
#include <ctype.h>

/* Return the first occurrence of NEEDLE in HAYSTACK.  */
void* memmem(const void* haystack, size_t haystack_len, const void* needle, size_t needle_len)
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
