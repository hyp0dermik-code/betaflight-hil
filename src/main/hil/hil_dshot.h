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

#ifndef HIL_DSHOT_H_
#define HIL_DSHOT_H_

/*
 * Run hardware in the loop motors task
 * To be run every 10ms according task attributes defined in tasks.c
 */
void hilDshotMainFunction(void);

/*
 * HIL command line parsing
 */
void hilDshotCliClr(const char *cmdName, char *cmdline);
void hilDshotCliAdd(const char *cmdName, char *cmdline);
void hilDshotCliPrint(const char *cmdName, char *cmdline);
void hilDshotCliRun(const char *cmdName, char *cmdline);
void hilDshotCliResults(const char *cmdName, char *cmdline);


#endif /* HIL_DSHOT_H_ */
