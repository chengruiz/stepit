# policy_neuro_redis

StepIt plugin for reading vector fields from Redis and feeding them into the StepIt neuro policy.

### Prerequisites

This plugin depends on `redis_base`. Install [hiredis](https://github.com/redis/hiredis.git) and
[nlohmann-json3](https://github.com/nlohmann/json) via `apt`:

```shell
sudo apt install libhiredis-dev nlohmann-json3-dev
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `redis_field_subscriber`: reads configured Redis keys, expects each value to be a JSON object, extracts numeric JSON arrays by member name, and provides the named StepIt fields declared in its config map.
