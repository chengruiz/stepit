# policy_neuro_motion_tracking

StepIt plugin for motion-tracking related modules in the neuro policy pipeline.

### Prerequisites

Install the [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library according to the [official guide](https://stack-of-tasks.github.io/pinocchio/download.html).

### Provided Factories

`stepit::neuro_policy::Module`:

- `forward_kinematics`: computes whole-body local and global link poses from URDF and joint states.
- `motion_alignment`: aligns target motion to robot pose using first-frame yaw and position offsets.
- `motion_trajectory`: loads frame-wise trajectory arrays from an `.npz` file and publishes configured fields each control step.
- `relative_ori`: computes relative orientation (`relative_ori`) and 6D rotation representation (`relative_ori_6d`) between current and target orientations.
- `relative_pos`: computes relative position in the current orientation frame.
