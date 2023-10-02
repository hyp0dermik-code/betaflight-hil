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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern "C" {
    #include "hil/hil_dshot.h"
    #include "drivers/dshot.h"
    #include "drivers/dshot_command.h"
    #include "flight/mixer.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


/* Motor pole count */
#define HIL_DSHOT_MOTOR_POLE_COUNT                  (14u)


static char buffer[1000];
static bool motorsEnabled = false;
static uint8_t dcwMotorIx = 0u;
static uint8_t dcwMotorCount = 0u;
static uint8_t dcwCommand = 0u;
static uint8_t dcwCommandType = 0u;
static uint32_t amperage = 0u;
static uint16_t voltage = 0;

float motor_disarmed[8];
dshotTelemetryState_t dshotTelemetryState;

static char add1[] = "motor 255 1020";
static char add2[] = "motor  0 1010";
static char add3[] = "motor    1     1040";
static char add4[] = "motor    3 1080";
static char add5[] = "dshotprog 255 13";
static char add6[] = "dshotprog   0 11";
static char add7[] = "dshotprog 1  10";
static char add8[] = "dshotprog  4   15";
static char add9[] = "delay 1000";
static char add10[] = "delay   60";
static char add11[] = "logging_on   ";
static char add12[] = "logging_on";
static char add13[] = "logging_off   ";
static char add14[] = "logging_off";

static char cmp1[] = "motor 255 1020";
static char cmp2[] = "motor 0 1010";
static char cmp3[] = "motor 1 1040";
static char cmp4[] = "motor 3 1080";
static char cmp5[] = "dshotprog 255 13";
static char cmp6[] = "dshotprog 0 11";
static char cmp7[] = "dshotprog 1 10";
static char cmp8[] = "dshotprog 4 15";
static char cmp9[] = "delay 1000";
static char cmp10[] = "delay 60";
static char cmp11[] = "logging_on";
static char cmp12[] = "logging_on";
static char cmp13[] = "logging_off";
static char cmp14[] = "logging_off";

TEST(HilMotorsUnittest, TestHilDshotAdd)
{
    hilDshotCliClr(NULL, NULL);

    hilDshotCliAdd("hit_dshot_add", add1);
    EXPECT_EQ(0, strcmp(buffer, cmp1));

    hilDshotCliAdd("hit_dshot_add", add2);
    EXPECT_EQ(0, strcmp(buffer, cmp2));

    hilDshotCliAdd("hit_dshot_add", add3);
    EXPECT_EQ(0, strcmp(buffer, cmp3));

    hilDshotCliAdd("hit_dshot_add", add4);
    EXPECT_EQ(0, strcmp(buffer, cmp4));

    hilDshotCliAdd("hit_dshot_add", add5);
    EXPECT_EQ(0, strcmp(buffer, cmp5));

    hilDshotCliAdd("hit_dshot_add", add6);
    EXPECT_EQ(0, strcmp(buffer, cmp6));

    hilDshotCliAdd("hit_dshot_add", add7);
    EXPECT_EQ(0, strcmp(buffer, cmp7));

    hilDshotCliAdd("hit_dshot_add", add8);
    EXPECT_EQ(0, strcmp(buffer, cmp8));

    hilDshotCliAdd("hit_dshot_add", add9);
    EXPECT_EQ(0, strcmp(buffer, cmp9));

    hilDshotCliAdd("hit_dshot_add", add10);
    EXPECT_EQ(0, strcmp(buffer, cmp10));

    hilDshotCliAdd("hit_dshot_add", add11);
    EXPECT_EQ(0, strcmp(buffer, cmp11));

    hilDshotCliAdd("hit_dshot_add", add12);
    EXPECT_EQ(0, strcmp(buffer, cmp12));

    hilDshotCliAdd("hit_dshot_add", add13);
    EXPECT_EQ(0, strcmp(buffer, cmp13));

    hilDshotCliAdd("hit_dshot_add", add14);
    EXPECT_EQ(0, strcmp(buffer, cmp14));
}

TEST(HilMotorsUnittest, TestHilDshotPrint)
{
    uint32_t i;

    // Check after clearing
    hilDshotCliClr("hil_motors_clr", NULL);
    hilDshotCliPrint("hil_dshot_print", NULL);
    EXPECT_EQ(0, strcmp(buffer, "HIL dshot command queue is empty."));

    // Check after adding many commands
    hilDshotCliAdd("hit_dshot_add", add1);
    hilDshotCliAdd("hit_dshot_add", add2);
    hilDshotCliAdd("hit_dshot_add", add3);
    hilDshotCliAdd("hit_dshot_add", add4);
    hilDshotCliAdd("hit_dshot_add", add5);
    hilDshotCliAdd("hit_dshot_add", add6);
    hilDshotCliAdd("hit_dshot_add", add7);
    hilDshotCliAdd("hit_dshot_add", add8);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add10);
    hilDshotCliAdd("hit_dshot_add", add11);
    hilDshotCliAdd("hit_dshot_add", add12);
    hilDshotCliAdd("hit_dshot_add", add13);
    hilDshotCliAdd("hit_dshot_add", add14);
    hilDshotCliPrint("hil_dshot_print", NULL);

    // Check command 1 is printed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(0, strcmp(buffer, cmp1));

    // Check command 2 is printed
    buffer[0] = 0u;
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp2));

    // Check command 3 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp3));

    // Check command 4 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp4));

    // Check command 5 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp5));

    // Check command 6 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp6));

    // Check command 7 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp7));

    // Check command 8 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp8));

    // Check command 9 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp9));

    // Check command 10 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp10));

    // Check command 11 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp11));

    // Check command 12 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp12));

    // Check command 13 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, cmp13));

    // Check command 14 is printed
    for (i = 0u; i < 4u; i++) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }
    EXPECT_EQ(0, strcmp(buffer, "Done"));
}

TEST(HilMotorsUnittest, TestHilDshotRun)
{
    uint32_t i;

    memset(motor_disarmed, 0, sizeof(motor_disarmed));

    // Add commands to the queue
    hilDshotCliClr("hil_motors_clr", NULL);
    hilDshotCliAdd("hit_dshot_add", add1);
    hilDshotCliAdd("hit_dshot_add", add2);
    hilDshotCliAdd("hit_dshot_add", add3);
    hilDshotCliAdd("hit_dshot_add", add4);
    hilDshotCliAdd("hit_dshot_add", add5);
    hilDshotCliAdd("hit_dshot_add", add6);
    hilDshotCliAdd("hit_dshot_add", add7);
    hilDshotCliAdd("hit_dshot_add", add8);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add10);
    hilDshotCliAdd("hit_dshot_add", add11);
    hilDshotCliAdd("hit_dshot_add", add12);
    hilDshotCliAdd("hit_dshot_add", add13);
    hilDshotCliAdd("hit_dshot_add", add14);
    hilDshotCliRun("hil_dshot_run", NULL);

    // Check command 1 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1020);
    EXPECT_EQ(motor_disarmed[1], 1020);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1020);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(0, strcmp(buffer, "0 M255:1020"));

    // Check command 2 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1020);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1020);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(0, strcmp(buffer, "1 M0:1010"));

    // Check command 3 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1020);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(0, strcmp(buffer, "2 M1:1040"));

    // Check command 4 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(0, strcmp(buffer, "3 M3:1080"));

    // Check command 5 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 255);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 13);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "4 P255:13"));

    // Check command 6 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 0);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 11);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "5 P0:11"));

    // Check command 7 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 1);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 10);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "6 P1:10"));

    // Check command 8 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "7 P4:15"));

    // Check command 9 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "8 D1000"));

    // Consume delay
    for (i = 0; i < 1000; i += 10) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }

    // Check command 10 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "109 D60"));

    // Consume delay
    for (i = 0; i < 60; i += 10) {
        buffer[0] = 0u;
        hilDshotMainFunction();
    }

    // Check command 11 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "116 LE"));

    // Check command 12 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "117 LE"));

    // Check command 13 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "118 LD"));

    // Check command 14 is executed
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "119 LD"));

    // Check run is done
    buffer[0] = 0u;
    hilDshotMainFunction();
    EXPECT_EQ(motor_disarmed[0], 1010);
    EXPECT_EQ(motor_disarmed[1], 1040);
    EXPECT_EQ(motor_disarmed[2], 1020);
    EXPECT_EQ(motor_disarmed[3], 1080);
    EXPECT_EQ(motor_disarmed[4], 0);
    EXPECT_EQ(motor_disarmed[5], 0);
    EXPECT_EQ(motor_disarmed[6], 0);
    EXPECT_EQ(motor_disarmed[7], 0);
    EXPECT_EQ(dcwMotorIx, 4);
    EXPECT_EQ(dcwMotorCount, 4);
    EXPECT_EQ(dcwCommand, 15);
    EXPECT_EQ(dcwCommandType, 1);
    EXPECT_EQ(0, strcmp(buffer, "HIL dshot run finished"));
}


TEST(HilMotorsUnittest, TestHilDshotResults)
{
    char strtst[1000];
    uint32_t k, r, s;

    // Add commands to the queue
    hilDshotCliClr("hil_motors_clr", NULL);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add3);
    hilDshotCliAdd("hit_dshot_add", add11);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add2);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliAdd("hit_dshot_add", add13);
    hilDshotCliAdd("hit_dshot_add", add9);
    hilDshotCliRun("hil_dshot_run", NULL);

    // Simulate a run command
    for (k = 0u; k < 510u; k++) {
        // Feed FC data
        voltage = 3000u - 4u * k;
        amperage = 1000u + 30u * k;

        // Feed ESC data
        for (r = 0u; r < 1u; r++) {
            dshotTelemetryState.motorState[r].telemetryData[DSHOT_TELEMETRY_TYPE_eRPM] = 300u + k;
            dshotTelemetryState.motorState[r].telemetryData[DSHOT_TELEMETRY_TYPE_TEMPERATURE] = 25u + k / 100u;
            dshotTelemetryState.motorState[r].telemetryData[DSHOT_TELEMETRY_TYPE_VOLTAGE] = (20000u - 40u * k) / 250u;
            dshotTelemetryState.motorState[r].telemetryData[DSHOT_TELEMETRY_TYPE_CURRENT] = (2000u + 30u * k) / 1000u;
            dshotTelemetryState.motorState[r].telemetryData[DSHOT_TELEMETRY_TYPE_STATE_EVENTS] = ((k & 0x0000000Fu) << 4u) + k / 30u;
        }

        hilDshotMainFunction();

        if (k == 0) {
            EXPECT_EQ(0, strcmp(buffer, "0 D1000"));
        } else if (k == 101) {
            EXPECT_EQ(0, strcmp(buffer, "101 M1:1040"));
        } else if (k == 102) {
            EXPECT_EQ(0, strcmp(buffer, "102 LE"));
        } else if (k == 103) {
            EXPECT_EQ(0, strcmp(buffer, "103 D1000"));
        } else if (k == 204) {
            EXPECT_EQ(0, strcmp(buffer, "204 D1000"));
        } else if (k == 305) {
            EXPECT_EQ(0, strcmp(buffer, "305 M0:1010"));
        } else if (k == 306) {
            EXPECT_EQ(0, strcmp(buffer, "306 D1000"));
        } else if (k == 407) {
            EXPECT_EQ(0, strcmp(buffer, "407 LD"));
        } else if (k == 408) {
            EXPECT_EQ(0, strcmp(buffer, "408 D1000"));
        }
    }

    EXPECT_EQ(0, strcmp(buffer, "HIL dshot run finished"));

    // Run results command
    hilDshotCliResults("hil_dshot_results", NULL);
    hilDshotMainFunction();
    hilDshotMainFunction();

    // Simulate a result command
    for (k = 0u; k < 4u * 3000u / 10u + 16u; k++) {
        hilDshotMainFunction();

        if (k % 4u == 0u) {
            s = (102u + k / 4u);
            sprintf(strtst, "M0 %5d %3d.%02dV %3d.%03dA %6drcp %8drpm %3d.%02dV %3dA %3ddegC %d %d %d %2d",
                    s * 10u,
                    (3000u - 4u * s) / 100u,
                    (3000u - 4u * s) % 100u,
                    (1000u + 30u * s) / 1000u,
                    (1000u + 30u * s) % 1000u,
                    1010,
                    erpmToRpm(300u + s),
                    (2000u - 4u * s) / 100u,
                    (((2000u - 4u * s) % 100u) / 25u) * 25u,
                    (2000u + 30u * s) / 1000u,
                    25u + s / 100u,
                    ((((s & 0x0000000Fu) << 4u) + s / 30u) & 0x80u) >> 7,
                    ((((s & 0x0000000Fu) << 4u) + s / 30u) & 0x40u) >> 6,
                    ((((s & 0x0000000Fu) << 4u) + s / 30u) & 0x20u) >> 5,
                    s / 30u
                    );

            //printf("%s\n", strtst);
            EXPECT_EQ(0, strcmp(buffer, strtst));
        }
    }
//
//    getc(stdin);
}

// STUBS
extern "C" {

void cliPrintLinef(const char *format, ...)
{
    va_list arglist;

    va_start(arglist, format);
    vsprintf(buffer, format, arglist);
    va_end(arglist);

    //printf("%s\n", buffer);
}

float motorConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

uint16_t motorConvertToExternal(float externalValue)
{
    return (uint16_t)externalValue;
}

uint8_t getMotorCount(void)
{
    return 4u;
}

void motorDisable(void)
{
    motorsEnabled = false;
}

void motorEnable(void)
{
    motorsEnabled = true;
}

void dshotCommandWrite(uint8_t index, uint8_t motorCount, uint8_t command, dshotCommandType_e commandType)
{
    dcwMotorIx = index;
    dcwMotorCount = motorCount;
    dcwCommand = command;
    dcwCommandType = (uint8_t)commandType;

    //printf("%d, %d, %d, %d\n", index, motorCount, command, (uint8_t)commandType);
}

int32_t getAmperageLatest(void)
{
    return amperage;
}

uint16_t getBatteryVoltageLatest(void)
{
    return voltage;
}

uint32_t erpmToRpm(uint16_t erpm)
{
    //  rpm = (erpm * 100) / (motorConfig()->motorPoleCount / 2)
    return (erpm * 200) / HIL_DSHOT_MOTOR_POLE_COUNT;
}

}
