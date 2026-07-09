/*
 * IOTask.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Corey Minyard  AE5KM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#ifndef IOTASK_H
#define IOTASK_H

/*
 * This IOTask is basically something that is messaged from interrupts
 * to run IO operations in a task context.  It currently serves the
 * CAN and ACP drivers.
 */
void IOTask(void *pvParameters);

#endif /* CANTASK_H */
