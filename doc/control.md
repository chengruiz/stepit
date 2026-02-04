# Control

The control system accepts command strings in the following format:

`[Channel/]Action[:Argument1,Argument2,...]`, where

- **Channel**: Target component (e.g., `Agent`).
- **Action**: The command to execute.
- **Arguments**: Parameters for the action.

## Control Commands

The `Agent` channel handles state transitions and policy management.

| Control String              | Description                          |
| :-------------------------- | :----------------------------------- |
| `Agent/StandUp`             | Transition to standing state.        |
| `Agent/LieDown`             | Transition to lying state.           |
| `Agent/StandUpOrLieDown`    | Toggle between standing and lying.   |
| `Agent/PolicyOn`            | Enable policy control.               |
| `Agent/PolicyOff`           | Disable policy control.              |
| `Agent/PolicyOnOrOff`       | Toggle policy control.               |
| `Agent/Freeze`              | Enter frozen safety state.           |
| `Agent/Unfreeze`            | Exit frozen state to resting.        |
| `Agent/CyclePolicy`         | Switch to the next available policy. |
| `Agent/SelectPolicy:<name>` | Switch to the specified policy.      |

The `Policy` channel is forwarded to the current policy for policy-specific commands.

