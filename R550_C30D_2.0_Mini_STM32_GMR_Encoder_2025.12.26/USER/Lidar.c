#include "distance_task.h"
#include "Lidar.h"

/* laser_id 是要读的通道，取值 1～4。
 * distance_mm 指向用于接收距离的变量，单位为毫米。
 * 返回 1 表示取得有效距离，返回 0 表示参数无效或等待超时。
 * 调用期间由你的流程负责激光选通。
 */
uint16_t distance_mm;
uint8_t ReadLaserDistanceMm(uint8_t laser_id, uint16_t *distance_mm)
{
    DistanceTask_Data data;
    TickType_t start_tick;
    uint8_t success = 0U;

    if (laser_id < 1U || laser_id > 4U || distance_mm == NULL) {
        return 0U;
    }

    /* 只提交一次选通请求。后台 distance_task 会切换通道、
     * 初始化传感器并采集距离。
     */
    DistanceTask_SelectLaser(laser_id);
    start_tick = xTaskGetTickCount();

    while ((TickType_t)(xTaskGetTickCount() - start_tick)
           < pdMS_TO_TICKS(2000U)) {

        /* 读取后台已经发布的结果。 */
        DistanceTask_GetData(&data);

        /* 只有读数有效，而且属于刚才选择的通道，才取出距离。 */
        if (data.valid && data.laser_id == laser_id) {
            *distance_mm = data.distance_mm;
            success = 1U;
            break;
        }

        /* 让出 CPU，给后台测距任务时间完成采样。 */
        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    /* 本例只取一次距离，所以结束时请求关闭激光。 */
    DistanceTask_SelectLaser(0U);

    return success;
}
