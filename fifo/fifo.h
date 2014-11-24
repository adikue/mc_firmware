

//Size must be power of 2!!!!!
//Here is a circle buffer. If overrun happens, tail will be the same.(faster)
#define __MAKE_FIFO(name, type, size) \
    volatile static unsigned char name##_head=0;\
    volatile static unsigned char name##_tail=0;\
    static type name##_buf[size];\
    unsigned char name##_isempty(void){ return name##_tail == name##_head? 1 : 0;}\
    unsigned char name##_isfull(void){ return (((name##_head+1)&(size-1)) == name##_tail)?1:0;}\
    void name##_push(type obj){name##_buf[name##_head] = obj;\
                            name##_head = (name##_head+1)&(size-1);}\
    type name##_pop(void){type tmp = name##_buf[name##_tail];\
                        name##_tail = (name##_tail+1-name##_isempty())&(size-1);\
                        return tmp;}
//Here is a circle buffer. If overrun happens, tail will be shifted.
//So buffer will be full of last data. (slower)

#define MAKE_FIFO(name, type, size) \
    volatile static unsigned int name##_head=0;\
    volatile static unsigned int name##_tail=0;\
    static type name##_buf[size];\
    unsigned int name##_isempty(void){ return name##_tail == name##_head? 1 : 0;}\
    unsigned int name##_isfull(void){ return (((name##_head+1)&(size-1)) == name##_tail)?1:0;}\
    void name##_push(type obj){name##_buf[name##_head] = obj;\
                            if(name##_isfull())\
                                name##_tail = (name##_tail+1)&(size-1);\
                            name##_head = (name##_head+1)&(size-1);}\
    type name##_pop(void){type tmp = name##_buf[name##_tail];\
                        name##_tail = (name##_tail+1-name##_isempty())&(size-1);\
                        return tmp;}\
    unsigned int name##_count(void){return ((size-1) & (name##_head - name##_tail));}\
    unsigned int name##_free(void){return (size -name##_count()-1) ;}

#define MAKE_FUNCTION_FIFO(name, size) \
    volatile static unsigned char name##_head=0;\
    volatile static unsigned char name##_tail=0;\
    static void (*name##_buf[size])(void);\
    unsigned char name##_isempty(void){ return name##_tail == name##_head? 1 : 0;}\
    unsigned char name##_isfull(void){ return (((name##_head+1)&(size-1)) == name##_tail)?1:0;}\
    void name##_add(void (*obj)(void)){name##_buf[name##_head] = obj;\
                            name##_head = (name##_head+1)&(size-1);}\
    void name##_call(void){ (*name##_buf[name##_tail])();\
                        name##_tail = (name##_tail+1-name##_isempty())&(size-1);\
                        }
