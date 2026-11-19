#include "system.h"
#include "nav_stm32.h"
#include "nav_vofa.h"
#include "remote_control.h"
#include <assert.h>
#include <string.h>

TestMotor MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
float Voltage=12, Velocity_KP, Velocity_KI, LineDiffParam, Move_X, Move_Y, Move_Z;
int Allow_Recharge, Check, KEY=1, EN=1, Flag_Stop;

static NavOutput command;
static NavStatus status;
static uint16_t front;
static float imu_yaw, imu_rate;
static int imu_publishing=1;
static int tx_enabled;
static unsigned sent_bytes, sent_frames;
static uint8_t packet[NAV_VOFA_FRAME_BYTES];

int USART_GetITStatus(int port, int flag)
{
    assert(port == USART2 && flag == USART_IT_TXE);
    return tx_enabled;
}

void USART_ITConfig(int port, int flag, int enabled)
{
    assert(port == USART2 && flag == USART_IT_TXE);
    tx_enabled=enabled;
}

void USART_SendData(int port, uint8_t byte)
{
    assert(port == USART2);
    packet[sent_bytes++]=byte;
    if (sent_bytes == sizeof(packet)) {
        assert(!memcmp(packet + sent_bytes - 4, "\x00\x00\x80\x7f", 4));
        sent_frames++;
        sent_bytes=0;
    }
}

static void publish_lidar(void)
{
    uint16_t mm[] = {
        (uint16_t)nav_calibration.left_stop_mm, front,
        (uint16_t)(nav_calibration.lr_sum_mm-nav_calibration.left_stop_mm),
        (uint16_t)nav_calibration.lane[0].side_back_mm
    };
    uint8_t bytes[8];
    for (unsigned i=0; i<4; i++) {
        bytes[2*i]=(uint8_t)(mm[i] >> 8);
        bytes[2*i+1]=(uint8_t)mm[i];
    }
    NavBoard_CanRx(0x301, 0, 0, sizeof(bytes), bytes);
}

static void advance(unsigned elapsed)
{
    assert(elapsed % 10 == 0);
    for (unsigned time=0; time<elapsed; time+=10) {
        for (unsigned ms=0; ms<10; ms++) NavBoard_Tick1ms();
        if (NavBoard_Millis() % 50 == 10) publish_lidar();
        if(imu_publishing) NavBoard_PublishImu(imu_yaw, imu_rate, 1);
        NavBoard_ControlStep(&command);
        Move_X=command.vx; Move_Y=command.vy; Move_Z=command.wz;
        NavBoard_PublishDrive(command.drive_enabled);
        NavBle_Poll();
        for (unsigned i=0; i<9; i++) NavBle_TxIRQ();
    }
    NavBoard_ReadStatus(&status);
}

static void click_mode(uint8_t ly, uint8_t rx)
{
    RemoteBoard_Publish(ly, rx, 1, 0);
    advance(50);
    RemoteBoard_Publish(ly, rx, 0, 0);
    advance(40);
}

static void publish_nav_button(int pressed)
{
    RemoteBoard_Publish(127, 127, 0, pressed);
}

static void click_navigation(void)
{
    publish_nav_button(1);
    advance(50);
    publish_nav_button(0);
    advance(40);
}

static void click_board(void)
{
    KEY=0;
    advance(50);
    KEY=1;
    advance(40);
}

static void check_remote_start(void)
{
    uint16_t start_front=front;

    /* Enabling the receiver with the navigation key already held starts nothing. */
    publish_nav_button(1);
    advance(50);
    publish_nav_button(0);
    advance(40);
    assert(!command.drive_enabled);

    EN=0;
    advance(10);
    publish_nav_button(1);
    advance(50);
    EN=1;
    advance(10);
    publish_nav_button(0);
    advance(40);
    assert(!command.drive_enabled);

    front=(uint16_t)nav_calibration.lane[0].side_front_mm;
    advance(50*NAV_FILTER_FRAMES);
    publish_nav_button(0);
    advance(10);
    click_navigation();
    assert(status.state == NAV_READY && !command.drive_enabled);
    front=start_front;
    advance(50*NAV_FILTER_FRAMES);
    publish_nav_button(0);
    advance(10);

    /* A release from a new receiver session cannot complete a prior press. */
    publish_nav_button(1);
    advance(50);
    RemoteBoard_Disconnect();
    publish_nav_button(0);
    advance(40);
    assert(!command.drive_enabled);

    publish_nav_button(1);
    advance(REMOTE_TIMEOUT_MS);
    publish_nav_button(0);
    advance(40);
    assert(!command.drive_enabled);

    publish_nav_button(1);
    advance(50);
    assert(!command.drive_enabled);
    publish_nav_button(0);
    advance(40);
    assert(command.drive_enabled && status.target == 1 && status.start_front_mm == start_front);
    click_navigation();
    assert(command.drive_enabled && status.target == 1);

    KEY=0;
    advance(50);
    assert(!command.drive_enabled && status.state == NAV_STOPPED);
    KEY=1;
    advance(40);
}

int main(void)
{
    unsigned frames_before;
    uint32_t lidar_before;
    NavBoard_Init();
    front=(uint16_t)(nav_calibration.lane[0].side_front_mm + 400);
    advance(200);
    if (!NAV_CALIBRATED) {
        publish_nav_button(0);
        advance(10);
        click_navigation();
        assert(status.reason == NAV_NEED_CALIBRATION && !command.drive_enabled);
        puts("PASS remote navigation start requires calibration");
        return 0;
    }
    assert(status.state == NAV_READY && !status.manual_mode && !command.drive_enabled);
    check_remote_start();

    /* A receiver reconnected with the mode key held cannot take control. */
    RemoteBoard_Disconnect();
    RemoteBoard_Publish(255, 0, 1, 0);
    advance(50);
    RemoteBoard_Publish(255, 0, 0, 0);
    advance(50);
    assert(!status.manual_mode && !command.drive_enabled);

    click_board();
    assert(command.drive_enabled && !status.manual_mode);
    Nav_RequestWorkPause();
    advance(50);
    assert(Nav_WorkStopped());
    click_mode(127, 127);
    assert(Nav_WorkStopped() && !status.manual_mode);
    Nav_RequestWorkContinue();
    advance(20);
    assert(!Nav_WorkStopped() && command.drive_enabled);

    /* Continue the same UART packet and keep consuming CAN during takeover. */
    click_mode(255, 0);
    assert(status.manual_mode && status.state == NAV_STOPPED && command.reset_pi && !command.drive_enabled);
    advance(40);
    assert(!command.drive_enabled);
    MOTOR_A.Encoder=0.1f;
    RemoteBoard_Publish(127, 127, 0, 0);
    advance(40);
    RemoteBoard_Publish(255, 0, 0, 0);
    advance(40);
    assert(!command.drive_enabled);
    MOTOR_A.Encoder=0;
    RemoteBoard_Publish(127, 127, 0, 0);
    click_board();
    assert(status.manual_mode && !command.drive_enabled);
    click_navigation();
    assert(status.manual_mode && status.state == NAV_STOPPED && !command.drive_enabled);
    frames_before=sent_frames;
    lidar_before=status.lidar_sequence;
    front+=10;
    for (unsigned i=0; i<4; i++) {
        RemoteBoard_Publish(255, 0, 0, 0);
        advance(100);
        assert(command.drive_enabled && command.vx > 0 && command.vy > 0 && command.wz == 0);
    }
    assert(status.can_fresh && status.raw_mm[NAV_FRONT] == front && status.lidar_sequence > lidar_before);
    assert(sent_frames > frames_before);

    advance(REMOTE_TIMEOUT_MS - 100);
    assert(status.manual_mode && !command.drive_enabled);
    RemoteBoard_Publish(255, 0, 0, 0);
    advance(10);
    assert(!command.drive_enabled);
    RemoteBoard_Publish(127, 127, 0, 0);
    advance(10);
    RemoteBoard_Publish(255, 0, 1, 0);
    advance(50);
    assert(command.drive_enabled);
    RemoteBoard_Disconnect();
    RemoteBoard_Publish(255, 0, 0, 0);
    advance(50);
    assert(status.manual_mode && !command.drive_enabled);

    RemoteBoard_Publish(127, 127, 0, 0);
    advance(10);
    RemoteBoard_Publish(255, 127, 0, 0);
    advance(10);
    assert(command.drive_enabled);
    EN=0;
    advance(10);
    assert(!command.drive_enabled);
    EN=1;
    advance(10);
    assert(!command.drive_enabled);

    RemoteBoard_Publish(127, 127, 0, 0);
    advance(10);
    imu_yaw=0.20f;
    RemoteBoard_Publish(127, 0, 0, 0);
    advance(10);
    assert(command.drive_enabled && command.vy>0 && command.wz==0);
    imu_yaw=0.30f;
    advance(10);
    assert(command.wz<0 && status.heading_error_rad<0);
    float correction=command.wz;
    imu_rate=0.05f;
    advance(10);
    assert(command.wz<correction);
    imu_publishing=0;
    advance(NAV_IMU_TTL_MS);
    assert(!status.imu_ready && command.drive_enabled && command.wz==0);
    imu_rate=0;
    imu_publishing=1;
    advance(10);
    assert(status.imu_ready && command.drive_enabled && command.wz==0);
    RemoteBoard_Publish(127, 127, 0, 0);
    advance(10);
    assert(!command.drive_enabled && command.wz==0);
    imu_yaw=0;

    KEY=0;
    RemoteBoard_Publish(127, 127, 1, 1);
    advance(50);
    RemoteBoard_Publish(127, 127, 0, 1);
    advance(40);
    assert(!status.manual_mode && command.reset_pi && !command.drive_enabled);
    KEY=1;
    publish_nav_button(0);
    advance(50);
    assert(!command.drive_enabled);
    click_navigation();
    assert(command.drive_enabled && status.target == 1);
    puts("PASS remote start and mode: input gating, reconnection, IMU heading hold and expiry, CAN and VOFA continuity");
    return 0;
}
