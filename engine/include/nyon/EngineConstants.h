#pragma once

namespace Nyon {

/**
 * @brief Engine-wide timing constants
 * 
 * Centralized constants to prevent duplication and ensure consistency
 * across all engine systems.
 */

/// Fixed timestep for physics simulation (60 Hz) - double precision for time accumulation
inline constexpr double FIXED_TIMESTEP_D = 1.0 / 60.0;

/// Fixed timestep for physics simulation (60 Hz) - float precision for physics calculations  
inline constexpr float FIXED_TIMESTEP = 1.0f / 60.0f;

/// Maximum frame time to prevent spiral of death in fixed timestep loop
inline constexpr double MAX_FRAME_TIME_D = 0.25;

/// Maximum frame time to prevent spiral of death in fixed timestep loop
inline constexpr float MAX_FRAME_TIME = 0.25f;

} // namespace Nyon
