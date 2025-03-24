# Sample C Programs to Profile

### seed1.c – CLI-based Memory Usage with Code Paths
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Default memory block size per path */
#define BLOCK_SIZE_MB 128
#define MB (1024 * 1024)

/*
 * path1_func - allocates 128MB on the heap, fills it with data
 */
void path1_func() {
    char *block = malloc(BLOCK_SIZE_MB * MB);
    if (block) {
        memset(block, 'A', BLOCK_SIZE_MB * MB);
    }
    // simulate usage
    printf("Path1 used %dMB\n", BLOCK_SIZE_MB);
    free(block);
}

/*
 * path2_func - allocates 256MB with mmap-style sim
 */
void path2_func() {
    char *block = malloc(256 * MB);
    if (block) {
        memset(block, 'B', 256 * MB);
    }
    printf("Path2 used 256MB\n");
    free(block);
}

/*
 * path3_func - uses global/static + local stack buffer
 */
void path3_func() {
    static char static_block[64 * MB]; // 64MB static
    char stack_block[64 * 1024];       // 64KB stack
    memset(static_block, 'C', sizeof(static_block));
    memset(stack_block, 0, sizeof(stack_block));
    printf("Path3 used 64MB static + 64KB stack\n");
}

int main(int argc, char *argv[]) {
    const char *mode = argc > 1 ? argv[1] : "path1";
    if (strcmp(mode, "path2") == 0) path2_func();
    else if (strcmp(mode, "path3") == 0) path3_func();
    else path1_func();
    return 0;
}
```

### seed2.c – Heavy Heap Allocation with Fragmentation
```c
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

/*
 * Allocates multiple 1MB heap blocks in random order,
 * deallocates some of them to simulate fragmentation.
 * Total: ~512MB allocated
 */
int main() {
    srand(time(NULL));
    char *blocks[512];
    for (int i = 0; i < 512; i++) {
        blocks[i] = malloc(1024 * 1024); // 1MB each
        if (blocks[i]) blocks[i][0] = i;
    }
    // Free half of them randomly
    for (int i = 0; i < 512; i++) {
        if (rand() % 2 == 0 && blocks[i]) {
            free(blocks[i]);
            blocks[i] = NULL;
        }
    }
    printf("seed2: Done allocating and freeing 512MB heap\n");
    return 0;
}
```

### seed3.c – Mixed Stack, Global, and Heap Usage
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char global_data[128 * 1024 * 2]; // ~256KB static/global

/*
 * Uses both stack and heap to fill memory types.
 * Total memory: 256KB static + 64KB stack + 128MB heap
 */
void use_stack_and_heap() {
    char stack_buffer[64 * 1024]; // 64KB
    memset(stack_buffer, 1, sizeof(stack_buffer));
    char *heap_buffer = malloc(128 * 1024 * 1024); // 128MB
    memset(global_data, 2, sizeof(global_data));
    if (heap_buffer) {
        memset(heap_buffer, 3, 128 * 1024 * 1024);
        printf("Allocated mixed memory: heap + stack + static\n");
        free(heap_buffer);
    }
}

int main() {
    use_stack_and_heap();
    return 0;
}
```

Compile each with:
```bash
gcc -o seed1 seed1.c
gcc -o seed2 seed2.c
gcc -o seed3 seed3.c
```

Run under Valgrind:
```bash
valgrind --tool=memcheck --xtree-memory=full ./seed1 path3
valgrind --tool=memcheck --xtree-memory=full ./seed2
valgrind --tool=memcheck --xtree-memory=full ./seed3
```

