# Market-Based Task Allocation with Heterogeneous Teams of Robots

## Project Overview
This project focuses on implementing **market-based task allocation** strategies for a **multi-robot system**. It was developed as part of the **Distributed Intelligent Systems** course. The main goal is to efficiently allocate tasks to a fleet of robots in a simulated environment using two methods:
1. **Centralized Strategy**: A supervisor assigns tasks to robots.
2. **Distributed Strategy**: Robots autonomously bid and organize task allocation.

For each method, the project also incorporates a **three-step task planning** approach to optimize task scheduling.

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


For exact guideline please refer to `DIS_24-25_course_project_assignment.pdf`

---

## Methods Implemented
### 1. Centralized Algorithm  
A supervisor assigns tasks to robots using a **centralized market-based strategy**. The supervisor calculates costs and selects the best robot for each task sequentially. The code for this algorithm is in the *Centralized* folder.

### 2. Distributed Algorithm  
Robots bid for tasks autonomously. Each robot calculates the cost of performing a task and competes to minimize its individual time while maximizing overall efficiency. The code for this algorithm is in the *Distributed* folder.

### 3. Multi-Step Planning  
Both strategies include a **three-step task planning** mechanism:  
   - Robots plan up to **3 tasks** in advance to optimize task execution.  
   - The system evaluates the total cost of task assignments for better efficiency.
   - For exact implementation or results please refer to `report_slides.pdf`
     - The code for the centralized algorithm is in the *Centralized_bundle* folder.
     - The code for the distributed algorithm is in the *Distributed_planning* folder.
     - An attempt to make another centralized planning strategy is contained in the *[Unachieved] Centralized planning optimized* folder.
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

## Non fixed issues

- In the centralized algorithms, the reported activation time in the supervisor may occasionally exceed 66.66%. This is not an actual error in the robot's behavior but rather a reporting error. A robot is considered 'activated' when it is either traveling to a goal or handeling a task. However, if the robot stops during one of these actions (due to the lack of energy), the activation counter continues to increment, leading to an overestimation of the activation time. Importantly, the robot cannot move without sufficient energy.
- In the distributed algorithm, the controller that is used for the centralized algorithm generates jittery and jerky behavior in the distributed algorithm, hence the implementation of a new Astolfi controller for smooth trajectories towards the tasks.
- In the distributed algorithm, some robots may crash during the simulation, possibly due to inter-robot communication channel conflicts causing segmentation faults in the code.

---

## References
- Astolfi controller : Astolfi, A. (1999). *Exponential stabilization of a wheeled mobile robot via discontinuous control*. Journal of Dynamic Systems, Measurement, and Control, **121**(1), 121-126.
