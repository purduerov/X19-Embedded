# Renode Simulation Environment for X19-Embedded

This directory contains the [Antmicro Renode](https://renode.io/) simulation platform definitions and execution scripts for running and automating tests on X19 STM32 ARM binaries without physical hardware.

---

## 1. Directory Structure

```
sim/renode/
├── README.md                  # Documentation and automated testing guide
├── nucleo_f411re.repl          # Platform hardware definition for NUCLEO-F411RE
├── nucleo_g474re.repl          # Platform hardware definition for NUCLEO-G474RE
├── run_node1.resc              # Simulation script for Node 1 Pi Shield (F411)
├── run_node2.resc              # Simulation script for Node 2 Control Board (G474)
├── run_g474_scanner.resc       # Simulation script for Standalone I2C Bus Scanner (G474)
├── multi_node_network.resc     # Multi-node simulation script (Node 1 + Node 2 on CAN bus)
├── run_renode.ps1              # Unified PowerShell launcher script
└── tests/
    ├── test_renode.py          # Automated Python test runner (individual + multi-node)
    └── node_tests.robot        # Keyword-driven Robot Framework test suite
```

---

## 2. Installed Renode Engine

Renode v1.16.0 (.NET 8 runtime) is installed locally on this machine at:
```
C:\Users\aman\tools\renode_1.16.0-dotnet_portable\renode.exe
```

---

## 3. Running Simulations

From `Embedded/X19-Embedded/`:

### A. Individual Nodes (Headless CLI)
```powershell
# Run Node 1 Pi Shield
.\sim\renode\run_renode.ps1 -Target node1 -Duration "00:00:03"

# Run Node 2 Control Board
.\sim\renode\run_renode.ps1 -Target node2 -Duration "00:00:03"

# Run G474 I2C diagnostic scanner
.\sim\renode\run_renode.ps1 -Target scanner -Duration "00:00:03"
```

### B. Multi-Node Network (Nodes Together)
Boots Node 1 and Node 2 simultaneously on a shared virtual CAN bus (`rovCanBus`):
```powershell
.\sim\renode\run_renode.ps1 -Target multi -Duration "00:00:03"
```

### C. Interactive GUI
Opens the graphical Monitor terminal and live UART analyzers:
```powershell
.\sim\renode\run_renode.ps1 -Target node2 -Gui
.\sim\renode\run_renode.ps1 -Target multi -Gui
```

---

## 4. Automated Testing

### A. One-Command Python Test Runner
Runs automated regression tests across all individual nodes and the multi-node CAN cluster, verifying clean bootup and asserting zero CPU HardFaults:
```powershell
python sim/renode/tests/test_renode.py
```

### B. Robot Framework (`renode-test`)
For keyword-driven verification of UART outputs and virtual CAN packet assertions:
```powershell
C:\Users\aman\tools\renode_1.16.0-dotnet_portable\renode-test.bat sim/renode/tests/node_tests.robot
```
