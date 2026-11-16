#ifndef LIDAR_H
#define LIDAR_H

uint8_t ReadLaserDistanceMm(uint8_t laser_id, uint16_t *distance_mm);
extern uint16_t distance_mm;
#endif
