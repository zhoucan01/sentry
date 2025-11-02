#ifndef __MOTOR_H
#define __MOTOR_H


void steer_normol_send();
void steer_error_send();
void chassis_normol_send();
void chassis_error_send();
void pitch_Enable(void);
void 	pitch_Unable(void);
void DM_start_run();
void dm_current_run(void);
#endif
