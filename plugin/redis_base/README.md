# redis_base

Base StepIt plugin that provides a reusable Redis client built on top of `hiredis` and `nlohmann_json`.

### Prerequisites

Install [hiredis](https://github.com/redis/hiredis.git) and [nlohmann-json3](https://github.com/nlohmann/json) via `apt`:

```shell
sudo apt install libhiredis-dev nlohmann-json3-dev
```

### Provided Components

- `stepit::redis::RedisClientConfig`
- `stepit::redis::RedisReadStatus`
- `stepit::redis::RedisClient`
