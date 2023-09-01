/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

//****************************************** INCLUDES **********************************************
#include <stdint.h>
#include <stdbool.h>
#include "common/strtol.h"
#include "drivers/motor.h"
#include "drivers/dshot.h"
#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "sensors/battery.h"
#include "flight/mixer.h"

#include "hil_dshot.h"

//****************************************** DEFINES ***********************************************
// Command index NULL
#define INDEX_NULL                          (0xFFFFFFFFu)

// Size of the command queue
#define HIL_DSHOT_COMMANDS_QUEUE_SIZE       (500u)

// Max log entries for 40 seconds
#define HIL_DSHOT_MAX_LOG_ENTRIES           (100u * 40u)

// Milliseconds per tick
#define HIL_DSHOT_MILLISECONDS_PER_TICK     (10u)

// Maximum number of motors
#define HIL_DSHOT_MAX_MOTOR_COUNT           (1u)


//****************************************** DATATYPES *********************************************
typedef enum {
    HIL_DSHOT_COMMAND_TYPE_NONE = 0,
    HIL_DSHOT_COMMAND_TYPE_MOTOR = 1,
    HIL_DSHOT_COMMAND_TYPE_PROG = 2,
    HIL_DSHOT_COMMAND_TYPE_DELAY = 3,
    HIL_DSHOT_COMMAND_TYPE_LOGGING_ENABLE = 4,
    HIL_DSHOT_COMMAND_TYPE_LOGGING_DISABLE = 5
} hil_motor_command_type_t;

typedef struct {
    hil_motor_command_type_t type;
    uint8_t data1;
    uint16_t data2;
} hil_motor_command_t;

typedef enum {
    HIL_DSHOT_STATE_IDLE = 0,
    HIL_DSHOT_STATE_PRINTING = 1,
    HIL_DSHOT_STATE_RUNNING = 2,
    HIL_DSHOT_STATE_RESULTS = 3,
} hil_motor_state_t;

typedef struct {
    uint16_t erpm;
    uint8_t escTemperaturePacked;
    uint8_t escVoltagePacked;
    uint8_t escCurrentPacked;
    uint8_t escStatePacked;
} hil_dshot_data_t;

typedef struct {
    uint16_t time;
    uint8_t fcVoltagePacked;
    uint8_t fcCurrentPacked;
    hil_dshot_data_t dshot_data[HIL_DSHOT_MAX_MOTOR_COUNT];
} hil_dshot_log_entry_t;


//****************************************** PRIVATE VARIABLES **************************************
// Commands queue
static hil_motor_command_t commands[HIL_DSHOT_COMMANDS_QUEUE_SIZE];
static uint32_t commandsCount = 0u;

// Heartbeat
static uint32_t heartbeat = 0u;

// Delay
static uint16_t delay = 0u;

// Internal stm state
static hil_motor_state_t state = HIL_DSHOT_STATE_IDLE;
static uint32_t commandIndex = INDEX_NULL;
static bool logging = false;

// Logging
static hil_dshot_log_entry_t log[HIL_DSHOT_MAX_LOG_ENTRIES];
static uint32_t logIndex = 0;
static uint32_t logCount = 0;


//****************************************** EXTERN FUNCTION DECLARATIONS ***************************
extern void cliPrintLinef(const char *format, ...);


//****************************************** PRIVATE FUNCTION DEFINITIONS ***************************
static void showHelpAdd(void)
{
    cliPrintLinef("hil_dshot_add motor [0-8,255] [0-2000]");
    cliPrintLinef("hil_dshot_add dshotprog [0-8,255] [0-48]");
    cliPrintLinef("hil_dshot_add delay [10-60000]");
    cliPrintLinef("hil_dshot_add logging_on");
    cliPrintLinef("hil_dshot_add logging_off");
}

static bool validateCommand(hil_motor_command_type_t type, uint32_t data1, uint32_t data2)
{
    bool res;

    switch (type)
    {

    case HIL_DSHOT_COMMAND_TYPE_MOTOR:
        res = (data1 <= 7 || data1 == 255) && (data2 < 2000);
        break;

    case HIL_DSHOT_COMMAND_TYPE_PROG:
        res = (data1 <= 7 || data1 == 255) && (data2 < 48);
        break;

    case HIL_DSHOT_COMMAND_TYPE_DELAY:
        res = (data2 <= 60000);
        break;

    case HIL_DSHOT_COMMAND_TYPE_LOGGING_ENABLE:
    case HIL_DSHOT_COMMAND_TYPE_LOGGING_DISABLE:
        res = true;
        break;

    default:
        res = false;
        break;

    }

    return res;
}

static void printCommand(hil_motor_command_t *c)
{
    switch (c->type)
    {

    case HIL_DSHOT_COMMAND_TYPE_MOTOR:
        cliPrintLinef("motor %d %d", c->data1, c->data2);
        break;

    case HIL_DSHOT_COMMAND_TYPE_PROG:
        cliPrintLinef("dshotprog %d %d", c->data1, c->data2);
        break;

    case HIL_DSHOT_COMMAND_TYPE_DELAY:
        cliPrintLinef("delay %d", c->data2);
        break;

    case HIL_DSHOT_COMMAND_TYPE_LOGGING_ENABLE:
        cliPrintLinef("logging_on");
        break;

    case HIL_DSHOT_COMMAND_TYPE_LOGGING_DISABLE:
        cliPrintLinef("logging_off");
        break;

    default:
        // Do nothing
        break;

    }
}

static void printLogEntry(hil_dshot_log_entry_t *l)
{
    for (uint32_t k = 0u; k < getMotorCount() && k < HIL_DSHOT_MAX_MOTOR_COUNT; k++) {
        cliPrintLinef(
                "M%d %5d %3d.%02dV %3dA %8drpm %3d.%02dV %3dA %3ddegC %d %d %d %2d",
                k,
                l->time,
                (l->fcVoltagePacked * 25u) / 100u,
                25u * (((l->fcVoltagePacked * 25u) % 100u) / 25u),
                l->fcCurrentPacked,
                erpmToRpm(l->dshot_data[0].erpm),                   // ESC rpm
                (l->dshot_data[k].escVoltagePacked * 250u) / 1000u, // ESC voltage
                25u * (((l->dshot_data[k].escVoltagePacked * 250u) % 1000u) / 250u),
                l->dshot_data[k].escCurrentPacked,                  // ESC current
                l->dshot_data[k].escTemperaturePacked,              // ESC temperature
                (l->dshot_data[k].escStatePacked & 0x80u) >> 7,     // Warning
                (l->dshot_data[k].escStatePacked & 0x40u) >> 6,     //
                (l->dshot_data[k].escStatePacked & 0x20u) >> 5,     // Error
                l->dshot_data[k].escStatePacked & 0x0Fu             // Max stress level
        );
    }
}

//****************************************** PUBLIC FUNCTION DEFINITIONS ****************************
void hilDshotMainFunction(void)
{
    uint32_t motorIndex;
    uint32_t motorOutputValue;
    uint32_t command;
    hil_dshot_log_entry_t *e;
    uint32_t k;

    // Run the state machine
    switch (state)
    {

    case HIL_DSHOT_STATE_PRINTING:
        // Space printing every 40ms to avoid overflowing the uart
        // Just like heartbeat % 4 == 0, but slightly faster
        if ((heartbeat & 0x03u) == 0u) {
            // Print the command
            printCommand(commands + commandIndex);

            // Update commandIndex and check state is done
            commandIndex++;
            if (commandIndex >= commandsCount)
            {
                cliPrintLinef("Done");
                state = HIL_DSHOT_STATE_IDLE;
                commandIndex = INDEX_NULL;
            }
        }
        break;

    case HIL_DSHOT_STATE_RUNNING:
        if (delay > 0u) {
            // Always deplete delay before running any command
            delay = (delay > HIL_DSHOT_MILLISECONDS_PER_TICK) ? delay - HIL_DSHOT_MILLISECONDS_PER_TICK : 0u;
        } else if ((commandIndex < commandsCount) && (commandsCount < HIL_DSHOT_COMMANDS_QUEUE_SIZE)) {
            // Run following command
            switch (commands[commandIndex].type)
            {

            case HIL_DSHOT_COMMAND_TYPE_DELAY:
                // Set delay
                delay = commands[commandIndex].data2;

                // Trace command
                cliPrintLinef("%d D%d", heartbeat, commands[commandIndex].data2);

                // Increase command index
                commandIndex++;
                break;

            case HIL_DSHOT_COMMAND_TYPE_MOTOR:
                // Set motor command
                motorIndex = commands[commandIndex].data1;
                motorOutputValue = motorConvertFromExternal(commands[commandIndex].data2);

                if (motorIndex != ALL_MOTORS) {
                    motor_disarmed[motorIndex] = motorOutputValue;
                } else  {
                    for (int i = 0; i < getMotorCount(); i++) {
                        motor_disarmed[i] = motorOutputValue;
                    }
                }

                // Trace command
                cliPrintLinef("%d M%d:%d", heartbeat, commands[commandIndex].data1, commands[commandIndex].data2);

                // Increase command index
                commandIndex++;
                break;

            case HIL_DSHOT_COMMAND_TYPE_PROG:
                // Set dshot command
                motorIndex = commands[commandIndex].data1;
                command = commands[commandIndex].data2;

                // Write command
                dshotCommandWrite(motorIndex, getMotorCount(), command, DSHOT_CMD_TYPE_BLOCKING);

                // Trace command
                cliPrintLinef("%d P%d:%d", heartbeat, commands[commandIndex].data1, commands[commandIndex].data2);

                // Increase command index
                commandIndex++;
                break;

            case HIL_DSHOT_COMMAND_TYPE_LOGGING_ENABLE:
                // Enable logging
                logging = true;

                // Trace command
                cliPrintLinef("%d LE", heartbeat);

                // Increase command index
                commandIndex++;
                break;

            case HIL_DSHOT_COMMAND_TYPE_LOGGING_DISABLE:
                // Disable logging
                logging = false;

                // Trace command
                cliPrintLinef("%d LD", heartbeat);

                // Increase command index
                commandIndex++;
                break;

            default:
                // Stop running
                state = HIL_DSHOT_STATE_IDLE;
                commandIndex = 0u;

                // Stop motors
                motorDisable();

                // Print message
                cliPrintLinef("HIL dshot run stopped because of a memory bug");
                break;

            }
        } else {
            // Stop running
            state = HIL_DSHOT_STATE_IDLE;
            commandIndex = 0u;

            // Stop motors
            motorDisable();

            // Print message
            cliPrintLinef("HIL dshot run finished");
        }
        break;

    case HIL_DSHOT_STATE_RESULTS:
        // Space results every 40ms to avoid overflowing the uart
        // Just like heartbeat % 4 == 0, but slightly faster
        if ((heartbeat & 0x03u) == 0u) {
            // Print the log entry
            printLogEntry(log + logIndex);

            // Update commandIndex and check state is done
            logIndex++;
            if (logIndex >= logCount)
            {
                cliPrintLinef("Done");
                state = HIL_DSHOT_STATE_IDLE;
                logIndex = INDEX_NULL;
            }
        }
        break;

    default:
        // Do nothing
        break;

    }

    // Logging
    if (logging && logCount < HIL_DSHOT_MAX_LOG_ENTRIES)
    {
        e = log + logCount;
        e->time = (uint16_t)heartbeat;
        for (k = 0; k < HIL_DSHOT_MAX_MOTOR_COUNT && k < getMotorCount(); k++) {
            e->dshot_data[k].erpm = dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_eRPM];
            e->dshot_data[k].escTemperaturePacked = dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE];
            e->dshot_data[k].escVoltagePacked = dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE];
            e->dshot_data[k].escCurrentPacked = dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT];
            e->dshot_data[k].escStatePacked = dshotTelemetryState.motorState[k].telemetryData[DSHOT_TELEMETRY_TYPE_STATE_EVENTS];
        }
        e->fcCurrentPacked = getAmperageLatest() / 1000u;       // Assuming mAh -> ampere
        e->fcVoltagePacked = getBatteryVoltageLatest() / 25u;   // Assuming mV  -> volt quarter
        logCount++;
    }

    // Beat
    heartbeat++;
}

void hilDshotCliClr(const char *cmdName, char *cmdline)
{
    (void)cmdName;
    (void)cmdline;

    // If busy return
    if (state != HIL_DSHOT_STATE_IDLE) return;

    // Clear logging, heartbeat, commands and notify
    logCount = 0u;
    heartbeat = 0u;
    commandsCount = 0u;
    cliPrintLinef("HIL dshot command queue is empty.");
}

void hilDshotCliAdd(const char *cmdName, char *cmdline)
{
    (void)cmdName;
    char *p = cmdline;
    hil_motor_command_type_t type;
    uint32_t data1 = 0u;
    uint32_t data2 = 0u;

    // If busy return
    if (state != HIL_DSHOT_STATE_IDLE) return;

    // Check queue is full
    if (commandsCount >= HIL_DSHOT_COMMANDS_QUEUE_SIZE)
    {
        cliPrintLinef("HIL dshot command queue is full.");
        return;
    }

    // Skip to command
    while (*p == ' ') p++;
    if (*p == 0) {
        showHelpAdd();
        return;
    }

    // Read command
    if (p[0] == 'm' && p[1] == 'o' && p[2] == 't' && p[3] == 'o' && p[4] == 'r' && p[5] == ' ') {
        type = HIL_DSHOT_COMMAND_TYPE_MOTOR;
        p += 5;
    } else if (p[0] == 'd' &&
            p[1] == 's' && p[2] == 'h' && p[3] == 'o' && p[4] == 't' &&
            p[5] == 'p' && p[6] == 'r' && p[7] == 'o' && p[8] == 'g' && p[9] == ' ') {
        type = HIL_DSHOT_COMMAND_TYPE_PROG;
        p += 9;
    } else if (p[0] == 'd' && p[1] == 'e' && p[2] == 'l' && p[3] == 'a' && p[4] == 'y' && p[5] == ' ') {
        type = HIL_DSHOT_COMMAND_TYPE_DELAY;
        p += 5;
    } else if (p[0] == 'l' && p[1] == 'o' && p[2] == 'g' && p[3] == 'g' && p[4] == 'i' && p[5] == 'n' &&
            p[6] == 'g' && p[7] == '_' && p[8] == 'o' && p[9] == 'n' &&
            (p[10] == ' ' || p[10] == 0)) {
        type = HIL_DSHOT_COMMAND_TYPE_LOGGING_ENABLE;
        p += 10;
    } else if (p[0] == 'l' && p[1] == 'o' && p[2] == 'g' && p[3] == 'g' && p[4] == 'i' && p[5] == 'n' &&
            p[6] == 'g' && p[7] == '_' && p[8] == 'o' && p[9] == 'f' && p[10] == 'f' &&
            (p[11] == ' ' || p[11] == 0)) {
        type = HIL_DSHOT_COMMAND_TYPE_LOGGING_DISABLE;
        p += 11;
    } else {
        showHelpAdd();
        return;
    }

    if (type == HIL_DSHOT_COMMAND_TYPE_MOTOR ||
            type == HIL_DSHOT_COMMAND_TYPE_PROG ||
            type == HIL_DSHOT_COMMAND_TYPE_DELAY) {
        // Skip to data1
        while (*p == ' ') p++;
        if (*p == 0) {
            showHelpAdd();
            return;
        }

        // Read data1
        data1 = atoi(p);    // For motor or prog
        data2 = data1;      // For delay command

        // Read data 2
        if (type == HIL_DSHOT_COMMAND_TYPE_MOTOR ||
                type == HIL_DSHOT_COMMAND_TYPE_PROG) {
            // Pass over data 1
            while (*p != ' '  && *p != 0) p++;

            // Skip to data2
            while (*p == ' ') p++;
            if (*p == 0) {
                showHelpAdd();
                return;
            }

            // Read data2
            data2 = (uint16_t)atoi(p);
        }
    }

    // Validate command data
    if (!validateCommand(type, data1, data2))
    {
        showHelpAdd();
        return;
    }

    // Set command
    commands[commandsCount].type = type;
    commands[commandsCount].data1 = (uint8_t)data1;
    commands[commandsCount].data2 = (uint16_t)data2;

    // Confirm command
    cliPrintLinef("HIL dshot command added:");
    printCommand(commands + commandsCount);

    // Increase commands count
    commandsCount++;
}

void hilDshotCliPrint(const char *cmdName, char *cmdline)
{
    (void)cmdName;
    (void)cmdline;

    // If busy return
    if (state != HIL_DSHOT_STATE_IDLE) return;

    // Check command queue is empty
    if (commandsCount == 0u)
    {
        cliPrintLinef("HIL dshot command queue is empty.");
        return;
    }

    // Set state to start printing
    state = HIL_DSHOT_STATE_PRINTING;
    commandIndex = 0u;
}

void hilDshotCliRun(const char *cmdName, char *cmdline)
{
    (void)cmdName;
    (void)cmdline;

    // If busy return
    if (state != HIL_DSHOT_STATE_IDLE) return;

    // Check command queue is empty
    if (commandsCount == 0u)
    {
        cliPrintLinef("HIL dshot command queue empty.");
        return;
    }

    // Set state to run commands
    state = HIL_DSHOT_STATE_RUNNING;
    commandIndex = 0u;

    // Enable motors
    motorEnable();
}

void hilDshotCliResults(const char *cmdName, char *cmdline)
{
    (void)cmdName;
    (void)cmdline;

    // If busy return
    if (state != HIL_DSHOT_STATE_IDLE) return;

    // Check command queue is empty
    if (logCount == 0u)
    {
        cliPrintLinef("HIL dshot results empty.");
        return;
    }

    // Set state to start showing results
    state = HIL_DSHOT_STATE_RESULTS;
    logIndex = 0u;
}
