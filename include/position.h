#ifndef POSITION_H
#define POSITION_H

typedef struct {
    int x;
    int y;
    int z;
} position_t;

typedef struct {
    int (*get_x)();
    int (*get_y)();
    int (*get_z)();
} position_ops_t;

#endif  /* POSITION_H */