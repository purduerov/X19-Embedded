## 2024-05-18 - Factoring Polynomials for FMA on STM32G4
**Learning:** Factoring polynomials like `a*x^3 + (1-a)*x` into `x * (a*x^2 + (1-a))` reduces the number of multiplications and allows the Cortex-M4 FPU to utilize Fused Multiply-Add (FMA) instructions, improving performance without sacrificing readability.
**Action:** When working with polynomials or math curves on STM32G4, check if factoring can map the equation to better utilize hardware instructions like FMA.
