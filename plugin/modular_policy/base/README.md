# modular_policy_base

StepIt plugin for composing control policies from modules.

### Provided Interfaces

- `stepit::modular_policy::Actuator`: defines how to apply actions to the robot.
- `stepit::modular_policy::Module`: provides fields according to input fields.

### Provided Factories

- `stepit::Policy`:
    - `modular`: module-based control policy.

- `stepit::modular_policy::Actuator`:

  | Name       | Description                                            |
  | :--------- | :----------------------------------------------------- |
  | `position` | Translates actions to joint position commands.         |
  | `velocity` | Translates actions to joint velocity commands.         |
  | `torque`   | Translates actions to joint torque commands.           |
  | `hybrid`   | Translates actions to a combination of joint commands. |

- `stepit::modular_policy::Module`:

  | Name                     | Description                                         |
  | :----------------------- | :-------------------------------------------------- |
  | `action_history`         | Provides history of action commands.                |
  | `action_filter`          | Applies low-pass filtering to action commands.      |
  | `action_reordering`      | Reorders action commands.                           |
  | `field_ops`              | Applies generic field operations.                   |
  | `joint_reordering`       | Reorders joint states.                              |
  | `time_source`            | Provides step count and current policy time.        |

## Mechanisms

### Module

Classes derived from `Module` require to declare their input dependencies (`requirements`) and output fields
(`provisions`) by registering named fields through a global FieldManager. FieldManager assigns a unique ID to each
field and merges its declared size and scalar data type. At runtime, modules sequentially produce or process values
identified by FieldId.

### ModularPolicy

`ModularPolicy` orchestrates a set of `Module` instances into a policy pipeline. It:

1. Loads YAML configuration, reads user‐specified modules and adds a configured `Actuator`.
2. Ensures the `action` field has a source, then resolves dependencies by creating modules from unresolved requirements.
3. Resolves execution order from declared `requirements`/`provisions`, and reports circular or duplicate providers.
4. On `reset()`, calls `reset()` on each resolved module.
5. In each `act()` step, sequentially invokes `update()`, takes `action` from the field map, and sends it to the
   robot through the `Actuator`.
6. After `act()` returns and the low-level command is handed off, runs a post-act phase for module state latching
   and slower work such as publishing configured fields.
