#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <stepit/joystick/joystick.h>
#include <stepit/redis/redis_client.h>

namespace stepit {
namespace joystick {
namespace {
using JsonDict = redis::RedisClient::JsonDict;

void readBoolField(const JsonDict &payload, const char *key, bool &value) {
  auto it = payload.find(key);
  if (it == payload.end()) return;
  if (it->is_boolean()) {
    value = it->get<bool>();
    return;
  }
  if (it->is_number()) {
    value = it->get<float>() > 0.5F;
    return;
  }
}

void readNumberField(const JsonDict &payload, const char *key, float &value) {
  auto it = payload.find(key);
  if (it == payload.end() or not it->is_number()) return;
  value = it->get<float>();
}
}  // namespace

class RedisJoystick final : public Joystick {
 public:
  RedisJoystick() {
    getenv("STEPIT_JOYSTICK_REDIS_KEY", redis_key_);
    getenv("STEPIT_JOYSTICK_REDIS_POLL_INTERVAL_MS", poll_interval_ms_);
    if (redis_key_.empty()) redis_key_ = "joystick";
    STEPIT_ASSERT(poll_interval_ms_ >= 0, "'poll_interval_ms' must be non-negative, got {}.", poll_interval_ms_);

    disconnect_timeout_ms_ = poll_interval_ms_ * 5;
    status_.store(true, std::memory_order_release);
    poll_thread_ = std::thread([this] { pollThread(); });
  }

  ~RedisJoystick() override {
    status_.store(false, std::memory_order_release);
    if (poll_thread_.joinable()) poll_thread_.join();
  }

  bool connected() const override {
    std::lock_guard<std::mutex> _(mutex_);
    return isConnected();
  }

  void getState(State &state) override {
    std::lock_guard<std::mutex> _(mutex_);
    if (not isConnected()) {
      state = {};
      return;
    }

    state = state_;
    for (auto &button : state_.buttons()) button.resetTransientStates();
  }

 private:
  void pollThread() {
    MSec interval = MSec(poll_interval_ms_ > 0 ? poll_interval_ms_ : 1);
    while (status_.load(std::memory_order_acquire)) {
      pollOnce();
      std::this_thread::sleep_for(interval);
    }
  }

  void pollOnce() {
    JsonDict payload;
    redis::RedisReadStatus status = redis::getDefaultRedisClient().get(redis_key_, payload);
    if (status != redis::RedisReadStatus::kOk) {
      std::lock_guard<std::mutex> _(mutex_);
      state_    = {};
      received_ = false;
      return;
    }

    std::lock_guard<std::mutex> _(mutex_);
    State next_state = state_;
    parsePayload(payload, next_state);

    state_       = std::move(next_state);
    last_update_ = SteadyClock::now();
    received_    = true;
  }

  bool isConnected() const {
    return received_ and getElapsedTime<MSec>(last_update_) < disconnect_timeout_ms_;
  }

  void parsePayload(const JsonDict &payload, State &state) const {
    bool a{false}, b{false}, x{false}, y{false}, start{false}, select{false}, las{false}, ras{false};
    bool lb{false}, rb{false};
    float lt{0.0F}, rt{0.0F}, las_x{0.0F}, las_y{0.0F}, ras_x{0.0F}, ras_y{0.0F};

    readBoolField(payload, "A", a);
    readBoolField(payload, "B", b);
    readBoolField(payload, "X", x);
    readBoolField(payload, "Y", y);
    readBoolField(payload, "Start", start);
    readBoolField(payload, "Select", select);
    readBoolField(payload, "LAS", las);
    readBoolField(payload, "RAS", ras);
    readBoolField(payload, "LB", lb);
    readBoolField(payload, "RB", rb);
    readNumberField(payload, "LT", lt);
    readNumberField(payload, "RT", rt);
    readNumberField(payload, "las_x", las_x);
    readNumberField(payload, "las_y", las_y);
    readNumberField(payload, "ras_x", ras_x);
    readNumberField(payload, "ras_y", ras_y);

    state.A().update(a);
    state.B().update(b);
    state.X().update(x);
    state.Y().update(y);
    state.LB().update(lb);
    state.RB().update(rb);
    state.Select().update(select);
    state.Start().update(start);
    state.LAS().update(las);
    state.RAS().update(ras);
    state.Up().update(false);
    state.Down().update(false);
    state.Left().update(false);
    state.Right().update(false);

    state.las_x() = las_x;
    state.las_y() = las_y;
    state.ras_x() = ras_x;
    state.ras_y() = ras_y;
    state.lt()    = lt;
    state.rt()    = rt;
  }

  std::string redis_key_;
  int poll_interval_ms_{10};
  int disconnect_timeout_ms_{50};

  std::atomic<bool> status_{false};
  std::thread poll_thread_;

  State state_;
  TimePoint last_update_{};
  bool received_{false};
  mutable std::mutex mutex_;
};

STEPIT_REGISTER_JOYSTICK(redis, kDefPriority, Joystick::make<RedisJoystick>);
STEPIT_REGISTER_CTRLINPUT(joystick_redis, kDefPriority, []() { return std::make_unique<JoystickControl>("redis"); });
}  // namespace joystick
}  // namespace stepit
