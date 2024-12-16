# Market-Based Task Allocation with Heterogeneous Teams of Robots

## Project Overview
This project focuses on implementing **market-based task allocation** strategies for a **multi-robot system**. It was developed as part of the **Distributed Intelligent Systems** course. The main goal is to efficiently allocate tasks to a fleet of robots in a simulated environment using two methods:
1. **Centralized Strategy**: A supervisor assigns tasks to robots.
2. **Distributed Strategy**: Robots autonomously bid and organize task allocation.

For each method, the project incorporates a **three-step task planning** approach to optimize task scheduling.

---

## Context of the Project
- **Scenario**:  
  A rescue mission after a natural disaster where 5 robots must complete **10 tasks** that appear dynamically:
   - **Task A**: Medical treatment (Red tasks).  
   - **Task B**: Psychological support (Blue tasks).  

- **Robots**:  
  The system involves 5 heterogeneous robots:  
   - **2 robots** specialize in **Task A**.  
   - **3 robots** specialize in **Task B**.

- **Constraints**:  
   - **Limited energy**: Robots can operate for 2 minutes.  
   - Robots must avoid collisions and have limited communication range (0.3 m).  
   - Tasks are assigned dynamically based on priorities.  

---

## Methods Implemented
### 1. Centralized Algorithm  
A supervisor assigns tasks to robots using a **centralized market-based strategy**. The supervisor calculates costs and selects the best robot for each task sequentially.

### 2. Distributed Algorithm  
Robots bid for tasks autonomously. Each robot calculates the cost of performing a task and competes to minimize its individual time while maximizing overall efficiency.

### 3. Multi-Step Planning  
Both strategies include a **three-step task planning** mechanism:  
   - Robots plan up to **3 tasks** in advance to optimize task execution.  
   - The system evaluates the total cost of task assignments for better efficiency.

---

## Metrics  
To compare the performance of both strategies, the following metrics are reported:  
- **Total number of tasks completed**.  
- **Average activation time**: Percentage of time the robots are moving or completing tasks.  
- **Total number of collisions**.

Each metric is averaged over **5 runs** to ensure consistency.

---

## Results and Analysis
In the presentation, the following aspects are covered:  
- Overview of the **task allocation problem** and the **two implemented methods**.  
- Discussion of the metrics and comparison of performance for both methods.  
- Quantitative and qualitative comparison using metrics, videos, and visualizations.

---

## Provided Materials
The following materials are included for the project:
- **Worlds**: `topic4.wbt`  
- **Protos**: `epuck_range.proto`  
- **Controllers**:  
   - Supervisor: `supervisor.c`  
   - e-Puck robots: `e-puck.c`
