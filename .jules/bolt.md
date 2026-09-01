## 2024-05-24 - Horner's Method for Polynomial Evaluation
**Learning:** In microcontroller environments (like the STM32G4 FPU), reducing floating-point operations in tight loops (e.g., a 1 kHz PWM loop) is a critical optimization pattern.
**Action:** Always evaluate mathematical expressions for opportunities to factor out terms, such as using Horner's method to reduce the number of multiplications.
