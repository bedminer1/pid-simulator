#pragma once

namespace pid_sim {
	template <typename T>
	constexpr T clamp(T val, T lo, T hi) {
		return val < lo
			? lo
			: val > hi
			? hi
			: val;
	}
}