## :bat: Inverse Kinematics by Bat algorithm 

Within this repository, the Bat Algorithm for solving IK (Inverse Kinematics) was implemented. The algorithm cannot scale with the speed of analytical or numerical methods of IK. The advantage of this implementation, that it is a very general approach, that does not need specific knowledge about the robot. Needed is only the number of joints represented by the **dimension** and D-H table (non=modified version) for calculation of FK (Forward Kinematics).

The algorithm as you can expect is not very efficient for solving the translational and rotational part of IK. In script is example of translational part IK calculation (demo), and translational + rotational example of calculation.

```javascript
Software
------------------------------------
| Python version 3.10
|   - Poetry version 1.2.1
```

### How-to-Start

Install libraries from **pyproject.toml/requirements.txt**.

* Set up Bat Algorithm: **params.py**.

* Change D-H table depanding on your robot: **FK.py**.

* Run computation: **main.py**

### Demo

![Demo](https://github.com/Steigner/UR3_Bat_Algo_IK/blob/main/docs/demo.gif)

:warning: I do not guarantee the most optimal variant of the Bat Algorithm.

## :information_source: Contacts

:mailbox: m.juricek@outlook.com
