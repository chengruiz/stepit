# modular_policy_nn

NNRT-backed modules for `modular_policy_base`.

## Provided Factories

| Type        | Description                                          |
| :---------- | :--------------------------------------------------- |
| `neural`    | Runs a configured neural-network model.             |
| `actor`     | Runs a model and provides its `action` output field. |
| `estimator` | Runs a model that estimates auxiliary policy state.  |

The module configuration template is [actor.yml](template/actor.yml).
