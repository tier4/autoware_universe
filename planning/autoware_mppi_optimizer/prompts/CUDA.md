When writing or reviewing CUDA code, strictly adhere to the following architectural, performance, and style guidelines:

### 1. Architectural & Memory Safety

- **Mandatory Error Checking:** Every single CUDA runtime API call must be wrapped in an error-checking macro (e.g., `CUDA_CHECK(cudaMalloc(...))`). Every kernel launch must be followed by a check of `cudaGetLastError()` and, when applicable, `cudaDeviceSynchronize()`.
- **Modern C++ RAII:** Avoid raw `cudaMalloc` and `cudaFree` calls directly inside business logic. Encapsulate device memory allocations using RAII patterns (e.g., custom smart pointer deleters, `thrust::device_vector`, or custom allocation wrappers).
- **Explicit Execution Spaces:** Ensure functions are strictly decorated with `__host__`, `__device__`, or `__global__`.

### 2. Performance & Memory Optimization

- **Arithmetic Intensity:** Only recommend GPU acceleration for algorithms with high compute-to-memory ratios. If a problem is inherently memory-bound or small, explicitly warn the user.
- **Memory Hierarchy:**
  - Maximize global memory bandwidth via **coalesced access patterns** (consecutive threads accessing consecutive memory locations).
  - Explicitly use **Shared Memory (`__shared__`)** for block-level data reuse and reduction patterns to minimize global memory round-trips.
  - Leverage unified memory (`cudaMallocManaged`) or pinned host memory (`cudaHostAlloc`) only when strategically appropriate for page-locked transfers or simplified codebases.
- **Concurrency & Streams:** Never rely on the default (blocking) stream unless explicitly requested. Maximize hardware utilization by overlapping compute and memory transfers using non-blocking CUDA streams and `cudaMemcpyAsync`.
- **Warp Level Primitives:** Prefer warp-shuffle instructions (`__shfl_sync`, `__reduce_add_sync`, etc.) over block-level shared memory reductions when data sharing happens within a single warp (32 threads).

### 3. Execution Configuration & Robustness

- **Grid/Block Invariant Coding:** Never hardcode block or grid dimensions inside kernels. Use `blockDim.x`, `threadIdx.x`, and `blockIdx.x` defensively.
- **Grid-Stride Loops:** When writing kernels intended for arbitrary data sizes, use grid-stride loops to ensure the kernel handles arrays larger than the total thread count gracefully and safely.
- **Divisibility Guards:** Always include boundary guards (`if (idx < N)`) unless the grid size is mathematically guaranteed to perfectly divide the problem size.

### 4. Output Formatting & Tone

- Provide complete, compilable, and modular code blocks. Avoid placeholders like `// ... do logic here ...` in critical sections.
- For every performance optimization you suggest, explain _why_ it helps in terms of hardware execution (e.g., "prevents warp divergence," "eliminates uncoalesced memory transactions").
- Maintain a direct, technical, and engineering-focused tone.
