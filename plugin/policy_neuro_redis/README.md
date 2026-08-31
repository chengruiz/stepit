# policy_neuro_redis

StepIt plugin for reading typed array fields from Redis and feeding them into the StepIt neuro policy.

### Prerequisites

This plugin depends on `redis_base`. Install [hiredis](https://github.com/redis/hiredis.git) and
[nlohmann-json3](https://github.com/nlohmann/json) via `apt`:

```shell
sudo apt install libhiredis-dev nlohmann-json3-dev
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `redis_field_subscriber`: reads configured Redis keys, expects each value to be a JSON object, extracts typed JSON arrays by member name, and provides the named StepIt fields declared in its config map.

### Field value types

Each configured field must declare its scalar type with `dtype`:

```yaml
fields:
  cmd_vel:
    key: "stepit:policy"
    size: 3
    dtype: "float32"

  contact:
    key: "stepit:state"
    size: 4
    dtype: "bool"
```

The Redis value must be a JSON object. The configured `field` member (or the StepIt field name when
`field` is omitted) must be an array whose length exactly matches `size`.

| `dtype` | Accepted JSON elements |
| --- | --- |
| `float32` | JSON numbers, including integer-valued numbers |
| `int32` | JSON integers within the `int32` range |
| `int64` | JSON integers within the `int64` range |
| `bool` | JSON booleans: `true` or `false` only |

For example, `{"cmd_vel": [0.1, 0.2, 0.3], "contact": [true, false, true, false]}` is valid.
Boolean fields do not accept numeric `0` or `1`, and integer fields do not accept floating-point JSON
numbers such as `1.0`.
