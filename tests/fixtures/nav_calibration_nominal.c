/* Fixed nominal scene for navigation simulation; never used in firmware. */
#include "nav_plan.h"
#include "nav_config.h"

/* 全部是名义尺寸示例，绝不是实测值；NAV_CALIBRATED 默认 0。
 * 先把整车摆正，逐沟测量侧道 F/B、沟内居中 F/B。
 * 左侧 B 穿出入口，忽略它；side_back 由同一沟右侧实测。
 * 短垄在图示第4/5沟之间：第4沟前壁、第5沟后壁区间较短。
 * 若实物不同，只修改本表，不修改导航算法。
 */
const NavCalibration nav_calibration = {
    2800.0f, 100.0f, 100.0f,
    {40.0f, 40.0f, 40.0f, 40.0f},
    {
        /* 侧道F, 侧道B, 沟内F, 沟内B, 前壁开始L/结束L, 后壁开始L/结束L */
        {4300,  100, 100, 100, 350,2450, 350,2450}, /* 沟1 */
        {3600,  800, 100, 100, 350,2450, 350,2450}, /* 沟2 */
        {2900, 1500, 100, 100, 350,2450, 350,2450}, /* 沟3 */
        {2200, 2200, 100, 100, 550,2250, 350,2450}, /* 沟4：前侧短垄 */
        {1500, 2900, 100, 100, 350,2450, 550,2250}, /* 沟5：后侧短垄 */
        { 800, 3600, 100, 100, 350,2450, 350,2450}, /* 沟6 */
        { 100, 4300, 100, 100, 350,2450, 350,2450}  /* 沟7 */
    }
};

const NavPoint nav_plan[17] = {
    {0,NAV_SIDE_LEFT, NAV_MOVE_ENTRY},  /* P01 */
    {0,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P02 */
    {1,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P03 */
    {1,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P04 */
    {2,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P05 */
    {2,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P06 */
    {3,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P07 */
    {3,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P08 */
    {4,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P09 */
    {4,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P10 */
    {5,NAV_SIDE_RIGHT,NAV_MOVE_SIDE},   /* P11 */
    {5,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P12 */
    {6,NAV_SIDE_LEFT, NAV_MOVE_SIDE},   /* P13 */
    {6,NAV_SIDE_RIGHT,NAV_MOVE_CROSS},  /* P14 */
    {0,NAV_SIDE_RIGHT,NAV_MOVE_RETURN},/* P15：同 P02 位置，返程阶段 */
    {0,NAV_SIDE_LEFT, NAV_MOVE_CROSS},  /* P16：同 P01 位置，返程阶段 */
    {0,NAV_SIDE_EXIT, NAV_MOVE_EXIT}    /* P17：起跑前距 */
};

