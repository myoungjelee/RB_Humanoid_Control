#pragma once

#include <string>

#include "controller_types.hpp"

namespace rb_controller::internal
{

/**
 * @brief 첫 유효 제어 시점 sync marker 로그 한 줄을 포맷한다.
 */
std::string format_control_active_sync(const ControlActiveSnapshot &snapshot);

/**
 * @brief loop_stats 로그 한 줄을 포맷한다.
 */
std::string format_loop_stats(const LoopStatsSnapshot &snapshot);

}  // namespace rb_controller::internal
