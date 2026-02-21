# policy_neuro_motion_tracking

StepIt plugin for motion-tracking related modules in the neuro policy pipeline.

### Provided Factories

`stepit::neuro_policy::Module`:

- `motion_trajectory`: loads frame-wise trajectory arrays from an `.npz` file and provides configured fields each control step.
- `forward_kinematics`: computes whole-body local/global link poses from URDF and joint states.
- `ori_error`: computes quaternion orientation error (and 6D orientation representation) between two orientation fields.
