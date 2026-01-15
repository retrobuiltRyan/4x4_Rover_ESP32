/*
| Button / Control           | Type             | Range or Values           | Function Call                |
| -------------------------- | ---------------- | ------------------------- | ---------------------------- |
| **Cross (X)**              | Digital          | `true` / `false`          | `PS4.Cross()`                |
| **Circle (O)**             | Digital          | `true` / `false`          | `PS4.Circle()`               |
| **Square (☐)**             | Digital          | `true` / `false`          | `PS4.Square()`               |
| **Triangle (△)**           | Digital          | `true` / `false`          | `PS4.Triangle()`             |
| **Up (D-Pad)**             | Digital          | `true` / `false`          | `PS4.Up()`                   |
| **Down (D-Pad)**           | Digital          | `true` / `false`          | `PS4.Down()`                 |
| **Left (D-Pad)**           | Digital          | `true` / `false`          | `PS4.Left()`                 |
| **Right (D-Pad)**          | Digital          | `true` / `false`          | `PS4.Right()`                |
| **L1**                     | Digital          | `true` / `false`          | `PS4.L1()`                   |
| **R1**                     | Digital          | `true` / `false`          | `PS4.R1()`                   |
| **L2**                     | Analog + Digital | 0–255 or `true` / `false` | `PS4.L2Value()` / `PS4.L2()` |
| **R2**                     | Analog + Digital | 0–255 or `true` / `false` | `PS4.R2Value()` / `PS4.R2()` |
| **L3 (Left Stick Click)**  | Digital          | `true` / `false`          | `PS4.L3()`                   |
| **R3 (Right Stick Click)** | Digital          | `true` / `false`          | `PS4.R3()`                   |
| **Share**                  | Digital          | `true` / `false`          | `PS4.Share()`                |
| **Options**                | Digital          | `true` / `false`          | `PS4.Options()`              |
| **PS (Logo)**              | Digital          | `true` / `false`          | `PS4.PS()`                   |
| **Touchpad Press**         | Digital          | `true` / `false`          | `PS4.Touchpad()`             |

| Stick Axis        | Type   | Range           | Function Call   |
| ----------------- | ------ | --------------- | --------------- |
| **Left Stick X**  | Analog | `-127` to `127` | `PS4.LStickX()` |
| **Left Stick Y**  | Analog | `-127` to `127` | `PS4.LStickY()` |
| **Right Stick X** | Analog | `-127` to `127` | `PS4.RStickX()` |
| **Right Stick Y** | Analog | `-127` to `127` | `PS4.RStickY()` |

| Info              | Type    | Range/Values | Function Call       |
| ----------------- | ------- | ------------ | ------------------- |
| **Battery Level** | Analog  | `0–100` (%)  | `PS4.Battery()`     |
| **Connected?**    | Digital | `true/false` | `PS4.isConnected()` |

*/
