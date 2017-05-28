#ifndef LOCK_H
#define LOCK_H

#include <pthread.h>

#ifndef __LOCK_TYPE__
#define __LOCK_TYPE__ pthread_mutex_t
#define __INIT_LOCK__(S) pthread_mutex_init(S, NULL)
#define __LOCK__(S) pthread_mutex_lock(S)
#define __UNLOCK__(S) pthread_mutex_unlock(S)
#define __DESTROY_LOCK__(S) pthread_mutex_destroy(S)
#endif

class Lock {
public:
    __LOCK_TYPE__ * lock_t;
    Lock(__LOCK_TYPE__* lock) {
        this-> lock_t = lock;
        __LOCK__(this-> lock_t);
    };
    
    ~Lock() {
        __UNLOCK__(this-> lock_t);
    };
};
#endif
