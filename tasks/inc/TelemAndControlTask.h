/*
 * TelemAndControlTask.h
 *
 *  Created on: Jun 5, 2023
 *      Author: g0kla
 */

#ifndef TASKS_SRC_TELEMANDCONTROLTASK_H_
#define TASKS_SRC_TELEMANDCONTROLTASK_H_


/*
 * Routine prototypes
 */
void TelemAndControlTask(void *pvParameters);
void tac_roll_file(char *file_name_with_path, char *folder, char *prefix);

/* Test routines */
bool tac_test_wod_file();
bool tac_test_txt_file();

#endif /* TASKS_SRC_TELEMANDCONTROLTASK_H_ */
