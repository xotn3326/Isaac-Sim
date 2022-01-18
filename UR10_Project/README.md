# UR10 Task 
----------
## Grip Cube
### Quick Start
> + Add Isaac Example Tab
> 
>    1. Download gripper_ur.py and gripper_ur_extension.py
> 
>    2. Move files to ~/user_examples

> + Edit UR10 Class in Isaac Sim Library (omni.isaac.universal_robots.ur10)
>    1. In ur10.py, change .usd file to ur10_gripper.usd file
>    
>       Change this raw to
>
> ```python
> usd_path = nucleus_server + "/Isaac/Robots/UR10/ur10_gripper.usd"
> ```

> + Press Load Tab In Isaac Example Tab
