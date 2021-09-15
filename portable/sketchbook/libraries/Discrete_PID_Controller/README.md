# Discrete_PID_Controller
A PID library for Arduino formulated in a mathematically correct way.

-----------------------------

Considerations:
Discrete Derivative Filter with defined N equals to 100

-----------------------------

Modifications:
- No timing condition for compute the pid, because I considered that execution of pid is controlled high level like task planner, timer or something like this.
- Organize the structure of the project.
