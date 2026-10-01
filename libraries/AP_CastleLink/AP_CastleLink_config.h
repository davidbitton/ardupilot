#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ESC_Telem/AP_ESC_Telem_config.h>

#ifndef AP_CASTLELINK_ENABLED
#define AP_CASTLELINK_ENABLED (HAL_WITH_ESC_TELEM && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif
