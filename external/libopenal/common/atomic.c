
#include "config.h"

#include "atomic.h"


/*extern inline void InitRef(RefCount *ptr, uint value);
extern inline uint ReadRef(RefCount *ptr);
extern inline uint IncrementRef(RefCount *ptr);
extern inline uint DecrementRef(RefCount *ptr);*/

inline void InitRef(RefCount *ptr, uint value)
{ ATOMIC_INIT(ptr, value); }
inline uint ReadRef(RefCount *ptr)
{ return ATOMIC_LOAD_SEQ(ptr); }
inline uint IncrementRef(RefCount *ptr)
{ return ATOMIC_ADD_SEQ(ptr, 1)+1; }
inline uint DecrementRef(RefCount *ptr)
{ return ATOMIC_SUB_SEQ(ptr, 1)-1; }
