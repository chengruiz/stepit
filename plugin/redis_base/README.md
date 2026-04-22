# redis_base

Base StepIt plugin that provides a reusable Redis client built on top of `hiredis` and `nlohmann_json`.

### Prerequisites

Install [hiredis](https://github.com/redis/hiredis.git) and [nlohmann-json3](https://github.com/nlohmann/json) via `apt`:

```shell
sudo apt install libhiredis-dev nlohmann-json3-dev
```

### Environment Variables

- `STEPIT_REDIS_HOST` (string, default: `127.0.0.1`)
- `STEPIT_REDIS_PORT` (int, default: `6379`)
- `STEPIT_REDIS_DB` (int, default: `0`)
- `STEPIT_REDIS_USERNAME` (string, default: empty)
- `STEPIT_REDIS_PASSWORD` (string, default: empty)
- `STEPIT_REDIS_CONNECT_TIMEOUT_MS` (int, default: `1000`)
- `STEPIT_REDIS_COMMAND_TIMEOUT_MS` (int, default: `50`)
