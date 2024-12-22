/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#ifdef USE_PINIOBOX

#include "build/debug.h"

#include "common/time.h"
#include "common/utils.h"

#include "msp/msp_box.h"

#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "scheduler/scheduler.h"

#include "piniobox.h"

#include "drivers/motor.h"
#include "flight/mixer.h"

typedef struct pinioBoxRuntimeConfig_s {
    uint8_t boxId[PINIO_COUNT];
    uint16_t motor_speed[MOTOR_COUNT];
} pinioBoxRuntimeConfig_t;

static pinioBoxRuntimeConfig_t pinioBoxRuntimeConfig;

void pinioBoxInit(const pinioBoxConfig_t *pinioBoxConfig)
{
    // Convert permanentId to boxId_e

    for (int i = 0; i < PINIO_COUNT; i++) {
        const box_t *box = findBoxByPermanentId(pinioBoxConfig->permanentId[i]);

        pinioBoxRuntimeConfig.boxId[i] = box ? box->boxId : BOXID_NONE;
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pinioBoxRuntimeConfig.motor_speed[i] = pinioBoxConfig->motor_speed[i];
    }
}

void pinioBoxUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxRuntimeConfig.boxId[i] != BOXID_NONE) {
            bool state = getBoxIdState(pinioBoxRuntimeConfig.boxId[i]);

            if (pinioBoxRuntimeConfig.boxId[i] >= BOXMOTOR1 &&
                pinioBoxRuntimeConfig.boxId[i] <= BOXMOTOR4) {
                uint16_t spd;
                uint16_t motor;

                motor = pinioBoxRuntimeConfig.boxId[i] - BOXMOTOR1;
                spd = motorConvertFromExternal(
                      state ? pinioBoxRuntimeConfig.motor_speed[motor] : 0);
                motor_disarmed[motor] = spd;
            } else {
                pinioSet(i, state);
            }
        }
    }
}

void pinioBoxTaskControl(void)
{
    bool enableTask = false;
    for (int i = 0; i < PINIO_COUNT; i++) {
        if (pinioBoxRuntimeConfig.boxId[i] != BOXID_NONE && isModeActivationConditionPresent(pinioBoxRuntimeConfig.boxId[i])) {
            enableTask = true;
            break;
        }
    }
    setTaskEnabled(TASK_PINIOBOX, enableTask);
}
#endif
