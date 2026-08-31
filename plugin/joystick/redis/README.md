# joystick_redis

StepIt plugin for reading joystick data from Redis.

### Provided Factories

- `stepit::joystick::Joystick`: `redis`
- `stepit::ControlInput`: `joystick_redis`

### Environment Variables

- `STEPIT_JOYSTICK_REDIS_KEY` (string, default: `joystick`)
- `STEPIT_JOYSTICK_REDIS_POLL_INTERVAL_MS` (int, default: `10`)
