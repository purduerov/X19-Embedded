## 2024-09-05 - Factor math polynomials to utilize FMA for hardware-level optimizations
**Learning:** The STM32G4 architecture (Cortex-M4 FPU) supports Fused Multiply-Add (FMA) instructions. By factoring polynomials (e.g., from `A*x^3 + B*x` to `x*(A*x^2 + B)`), we can significantly reduce the number of independent floating-point multiplications, allowing the compiler to generate more efficient FMA instructions.
**Action:** When implementing polynomial evaluations on architectures with an FPU that supports FMA, mathematically factor the expressions to maximize multiply-add patterns.
