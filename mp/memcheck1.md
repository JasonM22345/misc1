# Valgrind Memcheck Memory Usage Profiling

Memcheck uses a two-level shadow memory system to “mirror” every byte of application memory with a few metadata bits. These bits indicate whether a byte is addressable and whether it holds a defined value. In addition, an origin–tracking mechanism is used so that when an uninitialised (or invalid) value is used the originating allocation (heap, stack, etc.) can be reported. The profiling of memory usage is divided into four categories:

- Heap memory
- Stack memory
- Static data
- Memory mappings (mmap)

Below is a detailed explanation for each.



## 1. Heap Memory Profiling

Heap allocations are intercepted by Memcheck’s malloc wrappers. The key idea is to record every allocation (and later free or realloc) so that the corresponding shadow memory is updated and later errors (such as using uninitialised heap memory) can be reported.

### Main Steps and Functions

- **Intercepting Allocations:**  
  When a client calls a heap allocation (for example, via `malloc`, `calloc`, or C++’s `new`), Memcheck’s wrapper function (e.g. `MC_(malloc)`) is invoked.  
  - **Algorithm:**  
    1. Validate the allocation size (and, if needed, record a “fishy value” error if the size is suspicious).  
    2. Call `MC_(new_block)` which internally invokes `VG_(cli_malloc)` to allocate the real memory.
    3. Once the memory is allocated, if the block is not zeroed, call `MC_(make_mem_undefined_w_otag)` to mark the new block as undefined (along with an “origin tag” computed from the allocation’s context).  
    4. Record the allocated block in a hash table (typically `MC_(malloc_list)`) for later lookup.

- **Heap Free and Reallocation:**  
  When a free is performed (via `MC_(free)` or the C++ delete wrappers), Memcheck calls `MC_(handle_free)`:
  - **Algorithm:**  
    1. Look up the block in the allocation table.
    2. If found, remove it and call `MC_(make_mem_noaccess)` to mark the entire block (including its redzones) as noaccess.
    3. The block is then placed on a “freed queue” (to help detect dangling pointer accesses) using functions such as `add_to_freed_queue`.

- **Shadow Memory Updates:**  
  When allocating or freeing a block, the shadow memory system is updated via functions such as:
  - `set_address_range_perms`: used to update the “V+A bits” (the validity and addressability information) over a range of memory.
  - `MC_(copy_address_range_state)`: used during reallocations to copy shadow state from the old block to the new one.

### Visual Flow (Heap Allocation)

```mermaid
flowchart TD
    A[Client calls malloc/new]
    B[MC_(malloc) wrapper invoked]
    C[Call MC_(new_block)]
    D[VG_(cli_malloc) allocates memory]
    E[Memory block returned]
    F[Call MC_(make_mem_undefined_w_otag)]
    G[Record allocation in MC_(malloc_list)]
    H[Return pointer to client]
    
    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
    G --> H
```

**Function Details:**

- **MC_(new_block):**  
  Allocates a new block (via `VG_(cli_malloc)`), optionally zeros it (if `calloc` or similar), and then updates the shadow memory by marking the block as undefined (using `MC_(make_mem_undefined_w_otag)`). It also creates an MC_Chunk record (using a pool allocator) to track this block.

- **MC_(make_mem_undefined_w_otag):**  
  Sets the shadow bits (via `set_address_range_perms`) for the allocated memory and attaches an origin tag derived from the allocation’s context (using an ECU obtained by `VG_(get_ECU_from_ExeContext)`). This origin tag is used later to trace back the source of an uninitialised value.

- **MC_(handle_free):**  
  Looks up the block in the hash table and, if found, marks the memory as noaccess (again via `MC_(make_mem_noaccess)`), then queues the block for delayed release.



## 2. Stack Memory Profiling

Stack memory profiling in Memcheck differs from heap profiling because the stack is allocated per function call and typically not freed in the same way. Memcheck instruments stack allocations and deallocations and also takes care to mark “redzones” around stack frames.

### Main Steps and Functions

- **Stack Frame Instrumentation:**  
  At function entry, Memcheck instruments the code (via the translation engine) to record the stack frame.  
  - **Algorithm:**  
    1. Reserve a stack frame for local variables.
    2. Mark the new stack area as “undefined” using a call such as `MC_(make_mem_undefined_w_otag)` (but often using a lighter instrumentation that records only a one-level backtrace).
    3. Reserve “redzones” below the stack pointer to detect illegal accesses by marking them as noaccess (via `MC_(make_mem_noaccess)`).

- **Stack Redzone Handling:**  
  When a function exits, the redzones are reset to noaccess so that any later access to those areas is caught.  
  - **Algorithm:**  
    1. Identify the redzone (typically a fixed number of bytes below the stack pointer).
    2. Call `MC_(make_mem_noaccess)` to mark the redzone.
    3. For values coming from the stack, a one-level origin (the function name, or the address of the opening brace) is recorded to help the user locate uninitialised variables.

### Visual Flow (Stack Allocation)

```mermaid
flowchart TD
    A[Function Entry]
    B[Reserve stack frame for locals]
    C[Instrument stack frame]
    D[Mark local region as undefined (MC_(make_mem_undefined_w_otag))]
    E[Mark redzone as noaccess (MC_(make_mem_noaccess))]
    F[Execute function body]
    G[Function Exit: Restore redzones]
    
    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
```

**Function Details:**

- **MC_(make_mem_undefined_w_otag) (Stack Case):**  
  Although the same function is used for heap blocks, for the stack it is invoked during translation (at function entry) to mark the allocated stack area as undefined. In this case the origin is recorded using a simple (depth‑1) backtrace.

- **MC_(make_mem_noaccess):**  
  Used to mark redzones – these are regions of the stack that should never be accessed (to catch buffer overruns or use-after-return). The function updates the shadow memory so that any access will be flagged.



## 3. Static Data Profiling

Static (or global) memory regions are set up when the program starts. Memcheck initializes the shadow memory for the data and BSS segments so that all static memory is correctly marked as defined (if it was initialized) or undefined (if not).

### Main Steps and Functions

- **Initialization at Startup:**  
  During program startup, Memcheck iterates over the static data segments.
  - **Algorithm:**  
    1. Identify the ranges that correspond to static data.
    2. Use the function `set_address_range_perms` to mark these ranges with the appropriate V+A bits. For example, data that is initialised is marked “defined” whereas BSS may be marked “undefined.”
  
- **Shadow Memory for Static Data:**  
  The static data areas are covered by the same two-level shadow memory system that is used for dynamic allocations.
  
### Visual Flow (Static Data Setup)

```mermaid
flowchart TD
    A[Program Startup]
    B[Identify static data segments (data and BSS)]
    C[For each segment, determine initial state]
    D[Call set_address_range_perms]
    E[Shadow memory initialized for static areas]
    
    A --> B
    B --> C
    C --> D
    D --> E
```

**Function Details:**

- **set_address_range_perms:**  
  This function is central to updating the shadow memory. It is used during initialization of static data as well as in other parts of Memcheck. It works by determining the secondary map (or DSM) corresponding to a range and then “painting” the memory with the appropriate 2‑bit codes (e.g. VA_BITS16_DEFINED for defined memory).

- **VALGRIND_MAKE_MEM_DEFINED / VALGRIND_MAKE_MEM_UNDEFINED macros:**  
  In client code these macros are used to mark static memory regions accordingly. Under the hood, they invoke client requests that eventually call functions like `MC_(make_mem_defined)`.



## 4. Memory-Mapped (mmap) Regions Profiling

Memory mappings (from the system call mmap) are handled separately since they can map both file–backed and anonymous memory. Memcheck intercepts these calls to update its shadow memory accordingly.

### Main Steps and Functions

- **Intercepting mmap:**  
  When a process makes an mmap system call, Valgrind’s tool interface (through headers such as `pub_tool_aspacemgr.h`) intercepts the call.
  - **Algorithm:**  
    1. The mapping is created in the guest’s address space.
    2. Memcheck then calls functions (often the same as used for static or heap regions) such as `MC_(make_mem_defined)` if the mapping is initialized (for code or file–mapped regions) or `MC_(make_mem_undefined)` if it is anonymous memory.
    3. The shadow memory for the mapped region is updated by setting the V+A bits appropriately using `set_address_range_perms`.

- **Unmapping and Remapping:**  
  When an mmap region is unmapped (via munmap) or changed (via mremap), the shadow memory is updated to reflect the change.
  
### Visual Flow (mmap Region Setup)

```mermaid
flowchart TD
    A[mmap system call intercepted]
    B[Mapping created in guest address space]
    C[Determine mapping type (file-backed or anonymous)]
    D[Call MC_(make_mem_defined) or MC_(make_mem_undefined)]
    E[Call set_address_range_perms to update shadow memory]
    F[Region is now profiled by Memcheck]
    
    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
```

**Function Details:**

- **set_address_range_perms (mmap case):**  
  Just as with static data, this function “paints” the entire range of the mapped region with the proper status. For example, if a file is mapped, the memory may be marked as defined (and later verified during reads) whereas an anonymous mapping (obtained via MAP_ANONYMOUS) may be initially undefined.

- **MC_(make_mem_defined)/MC_(make_mem_undefined):**  
  These functions are reused here to update the shadow state of the mmap region according to the mapping’s purpose.



# Summary

Memcheck profiles memory usage by maintaining a parallel “shadow” copy of the entire address space. For each type of memory:

- **Heap:** Allocation wrappers (e.g. `MC_(new_block)`, `MC_(malloc)`) update the shadow memory to mark new blocks as undefined (with an origin tag), and frees update the shadow to noaccess.
- **Stack:** Instrumentation at function entry marks stack frames as undefined (and redzones as noaccess) while recording a lightweight backtrace for origin tracking.
- **Static Data:** On program startup, the data and BSS segments are scanned and the shadow memory is set to reflect the defined or undefined state of static variables.
- **mmap:** System call interception ensures that memory mappings are “painted” in shadow memory with the correct status, using similar functions as for static data.

Each of these steps relies on functions such as `set_address_range_perms`, `MC_(make_mem_undefined_w_otag)`, `MC_(make_mem_defined)`, and their variants. These functions compute the proper secondary map (or DSM) entries corresponding to each byte of the application’s memory and update the shadow bits (the “V+A bits”) accordingly. This design allows Memcheck to later detect errors like using uninitialised values, invalid reads/writes, and dangling pointers, all while providing detailed origin information to the user.


