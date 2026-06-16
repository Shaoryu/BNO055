# BNO055
```main.cpp
// 操縦方法

// omni_ps5 (黒コントローラー) 足回り、回収・発射BLDC担当
// 移動                                        左スティック      
// 旋回                                        右スティック   
// 回収・BLDCON/OFF                             DOWN
// 回収反転                                     LEFT

// turret_ps5 (白コントローラー) 発射動作、砲塔制御担当
// 自動照準ON/OFF                               UP             
// 発射シーケンス                                〇
// 仰角上げ/下げ                                 △/×
// 砲塔旋回左/右                                 L2/R2

// 操縦権の強奪                                  L1, R1を同時に2秒間長押し    操縦権の強奪



// CANMessage

// commands (id: 0x05)
//  data[0]: LSTICKX (omni)
//  data[1]: LSTICKY (omni)
//  data[2]: RSTICKx (omni)
//  data[3]: 
//  data[4]: 砲塔旋回 (turret)  (今はいったんauto_move_msgに移動)
//  data[5]: 仰角 (turret)  (今はいったんauto_move_msgに移動)
//  data[6]: 
//  data[7]:

// auto_move_msg (id: 0x06)
//  data[0]: 発射BLDCの制御
//  data[1]: 装填サーボの制御
//  data[2]: 回収チビマスの制御
//  data[3]: 
//  data[4]: 砲塔旋回 (turret)  (今はいったんauto_move_msgに移動)
//  data[5]: 仰角 (turret)  (今はいったんauto_move_msgに移動)
//  data[6]: 
//  data[7]: 自動照準のON/OFF


// pos_data (id: 0x09)
//  data[0]: x座標の上位8ビット
//  data[1]: x座標の下位8ビット
//  data[2]: y座標の上位8ビット
//  data[3]: y座標の下位8ビット
//  data[4]: yaw角の上位8ビット
//  data[5]: yaw角の下位8ビット
//  data[6]: 0
//  data[7]: 0



#include "mbed.h"
#include "ps5_can_lib.h"
#include "IncEnc_board.h"
#include "BNO055.h"
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159f
#endif

#define POS_ID 0x09
#define COMMAND_ID 0x05
#define AUTO_COMMAND_ID 0x06

enum Controllers { 
    OMNI_PS5,
    TURRET_PS5, 
    ALL_PS5 
};

enum ShootStep { 
    IDLE, 
    SPIN_UP, 
    PUSH_BALL, 
    RESET_AUTOMATION 
};

struct Position {
    float pos_x = 0.0f; 
    float pos_y = 0.0f;
    float angle = 0.0f;
};

// BufferedSerial pc(PA_2, PA_3, 921600);
DigitalOut leds[4] = { PC_0, PC_1, PC_2, PC_3 };
CAN command_can(PA_11, PA_12, 1e6);
CAN odom_can(PB_5, PB_6, 1e6);
EventQueue queue_cmd(32 * EVENTS_QUEUE_SIZE);
EventQueue queue_odom(32 * EVENTS_EVENT_SIZE);

can920 ps5_omni(command_can, 1);
can9202 ps5_turret(odom_can, 1);
BNO055 imu(PB_9, PB_8);
IncEnc_board odom(odom_can, 2);

Thread cmd_rcv_thread, odom_rcv_thread;
Timeout check_mode;
CANMessage auto_move_msg, pos_data;
ShootStep shoot_state = IDLE;
Timer shoot_timer;

int collector = 0x01, bldc_state = 0x01; // デフォルトで回す
int mode = ALL_PS5;
float imu_ang[3] = {0.f};
Position robot_pos = {0, 0, 0};
float _pos_x = 0, _pos_y = 0;
int64_t now_angles[2] = {0}, prev_angles[2] = {0, 0}; 
int data[ALL_PS5][PS5::ALL_BUTTON];
bool stop_signal[2];
bool emer_ctrlr1 = false, emer_ctrlr2 = false;
int _input_raw[ALL_PS5][8]; // CAN受信バッファ
int64_t encoder_raw[2] = {0, 0};
volatile bool omni_updated = false;
int counter = 0;
const float offset_x = 0.0f; // x軸エンコが左右にどれだけオフセットがあるか
const float offset_y = 0.0345f;// y軸エンコが前後にどれだけオフセットがあるか
float prev_yaw;
uint8_t data_for_co446[8] = {0};
int failsafe_cnt;
bool is_bldc_collect = false, is_auto_aim = true, is_inv_collect = false, is_auto_pitch = true; // 制御状態
bool pre_down = false, pre_share = false, pre_options = false, pre_left = false; // トグル用

extern volatile bool imu_i2c_busy; // IMUの非同期通信用

void safe_can_write(CANMessage msg) {
    int retry = 0;
    while (!command_can.write(msg) && retry < 3) {
        wait_us(250);
        retry++;
    }
}

void sort_cmd() {
    CANMessage tmp;
    while (command_can.read(tmp)) {
        if (tmp.id == 0x21) {
            for(int i=0; i<8; i++) _input_raw[OMNI_PS5][i] = tmp.data[i];
            omni_updated = true;
        }
    }
}

void sort_odom() {
    CANMessage tmp;
    while (odom_can.read(tmp)) {
        if (tmp.id == 0x11) {
            for(int i=0; i<8; i++) _input_raw[TURRET_PS5][i] = tmp.data[i];
        }
        else if (tmp.id >= 0x401 && tmp.id <= 0x402) {
            int node_idx = tmp.id - 0x401;
            int64_t val = 0;
            for (int i = 0; i < 8; i++) {
                val |= (int64_t)tmp.data[i] << (8 * (7 - i));
            }
            encoder_raw[node_idx] = val;
        }
    }
}

void onReceive_cmd() { queue_cmd.call(sort_cmd); }
void onReceive_odom() { queue_odom.call(sort_odom); }

void is_long() {
    if (emer_ctrlr2)      mode = OMNI_PS5;  
    else if (emer_ctrlr1) mode = TURRET_PS5; 
    else                  mode = ALL_PS5;    
}

void yaw_filter_easy() {
    float qw = imu.quat.w, qx = imu.quat.x, qy = imu.quat.y, qz = imu.quat.z;
    float t3 = +2.0f * (qw * qz + qx * qy);
    float t4 = +1.0f - 2.0f * (qy * qy + qz * qz);  
    imu_ang[0] = std::atan2(t3, t4); 
}

void odometry() { 
    __disable_irq();
    now_angles[0] = encoder_raw[0];
    now_angles[1] = encoder_raw[1];
    __enable_irq();

    float d_wheel_x = (now_angles[0] - prev_angles[0]) * M_PI * 0.0508f / 360.0f;
    float d_wheel_y = (now_angles[1] - prev_angles[1]) * M_PI * 0.0508f / 360.0f; 
    float yaw = imu_ang[0]; // R(-Θ)として計算するため

    float delta_yaw = yaw - prev_yaw;
    d_wheel_x -= offset_y * delta_yaw;
    d_wheel_y += offset_x * delta_yaw;

    _pos_x += d_wheel_x * cos(yaw) - d_wheel_y * sin(yaw);
    _pos_y += d_wheel_x * sin(yaw) + d_wheel_y * cos(yaw); 
    prev_angles[0] = now_angles[0]; prev_angles[1] = now_angles[1];
    prev_yaw = yaw;
}

// 二段階サーボ
/*
void update_auto_shoot(bool start_request) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(shoot_timer.elapsed_time()).count();
    static bool prev_request = false;

    switch (shoot_state) {
        case IDLE:
            auto_move_msg.data[1] = 0x00;
            //bldc_state = 0x01;
            collector = (is_inv_collect) ? 0x10 : 0x01;
            if (start_request && !prev_request) {
                shoot_timer.reset(); 
                shoot_timer.start();
                shoot_state = SPIN_UP;
            }
            break;
        case SPIN_UP:
            
            auto_move_msg.data[1] = 0x00; // サーボはそのまま
            collector = 0x11; // 吐き出し
            if (elapsed > 1000){
                shoot_state = PUSH_BALL; // 1秒待機
                shoot_timer.stop();
                shoot_timer.reset();
                shoot_timer.start();
            }
            break;
        case PUSH_BALL:
            auto_move_msg.data[1] = 0x10;
            if (elapsed > 500) auto_move_msg.data[1] = 0x01;
            if (elapsed > 1000){
                shoot_state = RESET_AUTOMATION;
                shoot_timer.stop();
                shoot_timer.reset();
                shoot_timer.start();
            }
            break;
        case RESET_AUTOMATION:
            auto_move_msg.data[1] = 0x00; // サーボ戻し
            collector = 0x01;
            if (elapsed > 500){ 
                shoot_state = IDLE; 
                shoot_timer.stop();
                shoot_timer.reset();
            }
            break;
    }

    prev_request = start_request;
}
*/

void update_auto_shoot(bool start_request) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(shoot_timer.elapsed_time()).count();
    static bool prev_request = false;

    switch (shoot_state) {
        case IDLE:
            auto_move_msg.data[1] = 0x00;
            //bldc_state = 0x01;
            collector = (is_inv_collect) ? 0x10 : 0x01;
            if (start_request && !prev_request) {
                shoot_timer.reset(); 
                shoot_timer.start();
                shoot_state = SPIN_UP;
            }
            break;
        case SPIN_UP:
            
            auto_move_msg.data[1] = 0x00; // サーボはそのまま
            collector = 0x11; // 吐き出し
            // if (elapsed > 1000){
                shoot_state = PUSH_BALL; // 1秒待機
                shoot_timer.stop();
                shoot_timer.reset();
                shoot_timer.start();
            // }
            break;
        case PUSH_BALL:
            auto_move_msg.data[1] = 0x01;
            if (elapsed > 1000){
                shoot_state = RESET_AUTOMATION;
                shoot_timer.stop();
                shoot_timer.reset();
                shoot_timer.start();
            }
            break;
        case RESET_AUTOMATION:
            auto_move_msg.data[1] = 0x00; // サーボ戻し
            collector = 0x01;
            if (elapsed > 500){ 
                shoot_state = IDLE; 
                shoot_timer.stop();
                shoot_timer.reset();
            }
            break;
    }

    prev_request = start_request;
}

// id: 0x06
// auto_move_msg[0] BLDC  0x00, 0x01
//              [1] servo 0x00, 0x10, 0x01
//              [2] miyazaki stpo 0x00, kaishu 0x01, hakidashi 0x10

int main() {
    Timer loop_timer;
    loop_timer.start();

    ps5_omni.setup(20);
    ps5_turret.setup(20); 
    for(auto& l : leds) l = false;
    leds[0] = true; 

    cmd_rcv_thread.start(callback(&queue_cmd, &EventQueue::dispatch_forever));
    command_can.attach(onReceive_cmd, CAN::RxIrq);
    odom_rcv_thread.start(callback(&queue_odom, &EventQueue::dispatch_forever));
    odom_can.attach(onReceive_odom, CAN::RxIrq);
    
    while(!imu.check()){ leds[1] = !leds[1]; ThisThread::sleep_for(100ms); }
    imu.reset(); ThisThread::sleep_for(500ms);
    imu.setmode(OPERATION_MODE_NDOF); ThisThread::sleep_for(100ms);
    imu.SetExternalCrystal(true);

        while(!imu.check()){ leds[1] = !leds[1]; ThisThread::sleep_for(100ms); }
    imu.reset(); ThisThread::sleep_for(500ms);
    imu.setmode(OPERATION_MODE_NDOF); ThisThread::sleep_for(100ms);
    imu.SetExternalCrystal(true);
    leds[1] = true; 
    pos_data.id = POS_ID; 
    auto_move_msg.id = AUTO_COMMAND_ID;
    imu.get_quat_async();
    //気休め
    for(int i=0;i<8;i++){
        _input_raw[OMNI_PS5][i] = _input_raw[TURRET_PS5][i] = (i>1&&i<6)?128:0;
    }

    while(true) { 
        loop_timer.reset();
        int val[2] = {0, 0};
        int local_input_omni[8], local_input_turret[8];//気休め
        bool current_updated;

        // auto_move_msg.data[0] = 0x01; // BLDCはずっと回転
        // auto_move_msg.data[2] = collector; // 回収もずっと回転

        __disable_irq(); 
        memcpy(local_input_omni, _input_raw[OMNI_PS5], sizeof(local_input_omni));
        memcpy(local_input_turret, _input_raw[TURRET_PS5], sizeof(local_input_turret));
        current_updated = omni_updated;
        omni_updated = false;
        __enable_irq();

        val[0] = ps5_omni.get_data_msg(data[0], &stop_signal[0], local_input_omni);
        val[1] = ps5_turret.get_data_msg(data[1], &stop_signal[1], local_input_turret);

        counter++;
        if (mode == ALL_PS5) {
            leds[1] = true;
        } else if (mode == OMNI_PS5) {
            if (counter % 50 == 0) leds[1] = !leds[1]; 
        } else if (mode == TURRET_PS5) {
            if (counter % 10 == 0) leds[1] = !leds[1]; 
        }
        if (counter >= 100) counter = 0;

        leds[2] = (val[0] == 1); 
        leds[3] = (val[1] == 1); 

        if (!imu_i2c_busy) { 
            imu.update_quat_data();
            yaw_filter_easy(); 
            imu.get_quat_async(); 
        }

        odometry();
        int16_t ox = (int16_t)(_pos_x * 100), oy = (int16_t)(_pos_y * 100), oa = (int16_t)(imu_ang[0] * 100);
        uint8_t p_data[8] = { (uint8_t)(ox>>8), (uint8_t)ox, (uint8_t)(oy>>8), (uint8_t)oy, (uint8_t)(oa>>8), (uint8_t)oa, 0, 0 };
        
        data_for_co446[3] = 0;
        data_for_co446[4] = 0;
        data_for_co446[5] = 0;
        data_for_co446[6] = 0;
        data_for_co446[7] = 0;
        auto_move_msg.data[0] = 0;
        auto_move_msg.data[1] = 0;
        auto_move_msg.data[2] = 0;
        // auto_move_msg.data[4] = 0;
        // auto_move_msg.data[5] = 0;
        auto_move_msg.data[7] = 0;

        int active_idx = (mode == ALL_PS5) ? OMNI_PS5 : mode;
        int turret_idx = (mode == ALL_PS5) ? TURRET_PS5 : mode;
        if (val[0] || val[1]) {
            if (data[OMNI_PS5][PS5::PSBUTTON] || data[TURRET_PS5][PS5::PSBUTTON]) {
                uint8_t reset_cmd[8]; 
                memset(reset_cmd, 0xff, 8);
                command_can.write(CANMessage(COMMAND_ID, reset_cmd, 8));
                ThisThread::sleep_for(10ms); 
                NVIC_SystemReset();
            }

            bool trigger1 = (data[TURRET_PS5][PS5::R1] && data[TURRET_PS5][PS5::L1]);
            bool trigger2 = (data[OMNI_PS5][PS5::R1] && data[OMNI_PS5][PS5::L1]);

            if (trigger1 || trigger2) {
                if (!emer_ctrlr1 && !emer_ctrlr2) check_mode.attach(is_long, 1000ms);
            } else {
                check_mode.detach();
            }
            emer_ctrlr1 = trigger1; emer_ctrlr2 = trigger2;

            data_for_co446[0] = (uint8_t)data[active_idx][PS5::LSTICKX];
            data_for_co446[1] = (uint8_t)data[active_idx][PS5::LSTICKY];
            data_for_co446[2] = (uint8_t)data[active_idx][PS5::RSTICKX];

            // if(data[turret_idx][PS5::CIRCLE]) data_for_co446[3] = 0x01;
            // if(data[turret_idx][PS5::SQUARE]) data_for_co446[7] = 0x01;
            
            auto_move_msg.data[4] = 0;//仮
            auto_move_msg.data[5] = 0;

            if(data[turret_idx][PS5::R2])      /*data_for_co446[4] = 0x01;*/ auto_move_msg.data[4] = 0x01;
            else if(data[turret_idx][PS5::L2]) /*data_for_co446[4] = 0x10;*/ auto_move_msg.data[4] = 0x10;

            if(data[turret_idx][PS5::TRIANGLE])    /*data_for_co446[5] = 0x01;*/ auto_move_msg.data[5] = 0x01;
            else if(data[turret_idx][PS5::CROSS])  /*data_for_co446[5] = 0x10;*/ auto_move_msg.data[5] = 0x10;
            
            if(data[turret_idx][PS5::SHARE] && !pre_share){
                is_auto_aim = !is_auto_aim; // 砲塔旋回の自動補正
                is_auto_pitch = !is_auto_pitch;
            }
            if(data[turret_idx][PS5::OPTIONS] && !pre_options) is_auto_pitch = !is_auto_pitch;

            // if(data[active_idx][PS5::LEFT])      data_for_co446[6] = 0x01;
            if(data[turret_idx][PS5::LEFT] && !pre_left) is_inv_collect = !is_inv_collect;
            // else if(data[active_idx][PS5::DOWN])     data_for_co446[6] = 0x20;
            if(data[turret_idx][PS5::DOWN] && !pre_down){
                // bldc_state = collector = 0x01;
                is_bldc_collect = !is_bldc_collect;
            }

            pre_share = data[turret_idx][PS5::SHARE];
            pre_options = data[turret_idx][PS5::OPTIONS];
            pre_down = data[turret_idx][PS5::DOWN];
            pre_left = data[turret_idx][PS5::RIGHT];
        }else{
            if(++failsafe_cnt > 10){
                leds[1] = false;
            }
        }

        if(shoot_state == IDLE){
            bldc_state = 0x01;
            collector = (is_inv_collect && is_bldc_collect) ? 0x10 : 0x01;
        }
        update_auto_shoot(data[turret_idx][PS5::CIRCLE]); // 発射動作
        
        if(is_bldc_collect){
            auto_move_msg.data[0] = bldc_state;
            auto_move_msg.data[2] = collector;
        }else{
            auto_move_msg.data[0] = 0x00;
            auto_move_msg.data[2] = 0x00;
        }
        auto_move_msg.data[6] = is_auto_pitch;
        auto_move_msg.data[7] = is_auto_aim;

        command_can.write(CANMessage(POS_ID, p_data, 8));
        wait_us(200);
        command_can.write(CANMessage(COMMAND_ID, data_for_co446, 8));
        wait_us(200);
        command_can.write(auto_move_msg);

        int sleep_time = 10000 - std::chrono::duration_cast<std::chrono::microseconds>(loop_timer.elapsed_time()).count();
        if (sleep_time > 0) wait_us(sleep_time);
    }
}













// 920通信, オドメトリ

// WiFi(チビルータ)に変えるかも
// 時間があるときにcmdのCANMessageを一つに統一する<<2/19準備完了。動いたら教えて。完全に統一する

//2026/2/21 砲塔の角度制限/角度指定 の準備

//ps5押されながら通信切れたら現状topは判断つかない
//対策はim920のファームウェアになる

//平行移動  :左スティック
//足回り旋回:右スティックx軸
//回収      :右ボタン
//排出      :左ボタン
//発射      :〇
//砲塔旋回  :L2/R2
//仰角調整  :△/✕   <<デバッグ用?オフセットとして残すつもりではいる

//led早見//2/25
//top   0:起動  1:ループ    2:ps5       3:yaw角
//mini1 0:起動  1:gm受信    2:旋回ps5   3:gm送信/ループ
//mini2 0:起動  1:ループ    2:cmd変更   3:なし
//deka  0:起動  1:ループ    2:cmd変更   3:なし

// 3/5時点のコード

// #include "mbed.h"
// #include "ps5_can_lib.h"
// #include "IncEnc_board.h"
// #include "BNO055.h"
// #include <cstring>
// #include <cmath>

// #ifndef M_PI
// #define M_PI 3.14159f
// #endif

// #define POS_ID 0x09
// #define COMMAND_ID 0x05

// struct Position{
//     float pos_x = 0.0f; 
//     float pos_y = 0.0f;
//     float angle = 0.0f;
// };

// extern volatile bool imu_i2c_busy;

// constexpr int co446_id[3] = {0x05, 0x06, 0x07}; 
// DigitalOut leds[4]={ PC_0, PC_1, PC_2, PC_3 };
// CAN command_can(PA_11, PA_12, 1e6);
// CAN odom_can(PB_5, PB_6, 1e6);

// can920 ps5(command_can, 1);
// BNO055 imu(PB_9, PB_8); 
// IncEnc_board odom(odom_can, 2);

// void yaw_filter_easy();

// float quat[4]={0.f};
// float imu_ang[3]={0.f}; // yaw, roll, pitch
// int64_t now_angles[2] = {0};
// Position robot_pos = {0, 0, 0};
// float current_yaw_rad = 0.0f; 
// float _pos_x = 0, _pos_y = 0;


// // void odometry(){ 
// //     odom.conv_data_all(now_angles);
// //     // printf("%d, %d\r\n, ")
// //     float ang_x = now_angles[0] * M_PI * 0.1 / 360.0f; 
// //     float ang_y = now_angles[1] * M_PI * 0.1 / 360.0f; 

// //     current_yaw_rad = -imu_ang[0];
// //     // R(-theta) 展開
// //     _pos_x =  ang_x * cos(current_yaw_rad) + ang_y * sin(current_yaw_rad);
// //     _pos_y = -ang_x * sin(current_yaw_rad) + ang_y * cos(current_yaw_rad); 
// // }

// int64_t prev_angles[2] = {0, 0}; 

// void odometry(){ 
//     odom.conv_data_all(now_angles);

//     int64_t diff_angles[2];
//     diff_angles[0] = now_angles[0] - prev_angles[0];
//     diff_angles[1] = now_angles[1] - prev_angles[1];

//     float delta_x = diff_angles[0] * M_PI * 0.0508f / 360.0f; 
//     float delta_y = diff_angles[1] * M_PI * 0.0508f / 360.0f; 

//     current_yaw_rad = -imu_ang[0];

//     _pos_x +=  delta_x * cos(current_yaw_rad) - delta_y * sin(current_yaw_rad);
//     _pos_y +=  delta_x * sin(current_yaw_rad) + delta_y * cos(current_yaw_rad); 

//     prev_angles[0] = now_angles[0];
//     prev_angles[1] = now_angles[1];
// }

// int main() {
//     ps5.setup(20);
//     for(auto& l : leds) l = false;
//     leds[0] = true; 

//     // 物理接続チェック
//     while(!imu.check()){
//         leds[3] = !leds[3]; 
//         ThisThread::sleep_for(100ms);
//     }

//     imu.reset();
//     ThisThread::sleep_for(500ms);

//     imu.setmode(OPERATION_MODE_CONFIG);
//     ThisThread::sleep_for(50ms);
//     imu.setmode(OPERATION_MODE_NDOF);
//     ThisThread::sleep_for(100ms);
//     imu.SetExternalCrystal(true);

//     // キャリブレーション待ち (ジャイロが安定するまで)
//     while(true) {
//         imu.get_calib();
//         if(imu.calib > 0) break; 
//         leds[3] = !leds[3]; 
//         ThisThread::sleep_for(100ms);
//     }
//     leds[3] = true;

//     CANMessage commands, pos_data;
//     pos_data.id = POS_ID;
//     int data[PS5::ALL_BUTTON];
//     bool stop_signal;
//     uint8_t data_for_co446[8];

//     imu.get_quat_async();

//     while(true) {
//         // IMUデータの非同期更新
//         if (!imu_i2c_busy) {
//             imu.update_quat_data();
//             yaw_filter_easy();
//             imu.get_quat_async();
//         }

//         int val = ps5.get_data(data, &stop_signal);
//         leds[2] = (val == 1);
        
//         odometry();

//         if(val == 1) {
//             if(data[PS5::PSBUTTON]){
//                 memset(commands.data, 0xff, 8);
//                 command_can.write(commands);
//                 wait_us(500);
//                 NVIC_SystemReset();
//             }
             
//             memset(data_for_co446, 0, sizeof(data_for_co446));

//             // PS5入力を送信バッファへ格納
//             data_for_co446[0] = (uint8_t)data[PS5::LSTICKX];
//             data_for_co446[1] = (uint8_t)data[PS5::LSTICKY];
//             data_for_co446[2] = (uint8_t)data[PS5::RSTICKX];
//             //deka(砲塔)宛て
//             if(data[PS5::CIRCLE])   data_for_co446[3] = 0x01;
//             //else if(data[PS5::SQUARE]) data_for_co446[1][0] = 0x10;

//             if(data[PS5::R2])       data_for_co446[4] = 0x01;
//             else if(data[PS5::L2])     data_for_co446[4] = 0x10;

//             if(data[PS5::TRIANGLE]) data_for_co446[5] = 0x01;
//             else if(data[PS5::CROSS])  data_for_co446[5] = 0x10;
            
//             if(data[PS5::R1])       data_for_co446[7] = 0x01; //後で消す

//             //mini2(その他)宛て
//             if(data[PS5::LEFT])     data_for_co446[6] = 0x01;
//             else if(data[PS5::RIGHT])  data_for_co446[6] = 0x10;
//             else if(data[PS5::DOWN])  data_for_co446[6] = 0x20;//一応
            
//         }

//         robot_pos.pos_x = _pos_x;
//         robot_pos.pos_y = _pos_y;
//         robot_pos.angle = imu_ang[0];

//         int16_t out_x = (int16_t)(robot_pos.pos_x * 10); // ロボットの前が正方向
//         int16_t out_y = (int16_t)(robot_pos.pos_y * 10);
//         int16_t out_ang = (int16_t)(robot_pos.angle * 100); // 精度確保のため

//         pos_data.data[0] = (out_x >> 8) & 0xFF;
//         pos_data.data[1] = out_x & 0xFF;
//         pos_data.data[2] = (out_y >> 8) & 0xFF;
//         pos_data.data[3] = out_y & 0xFF;
//         pos_data.data[4] = (out_ang >> 8) & 0xFF;
//         pos_data.data[5] = out_ang & 0xFF;
//         pos_data.data[6] = 0;
//         pos_data.data[7] = 0;

//         command_can.write(pos_data);
//         wait_us(500);

//         commands.id = COMMAND_ID;
//         memcpy(commands.data, data_for_co446, 8);
//         command_can.write(commands);

//         leds[1] = !leds[1];
//         ThisThread::sleep_for(10ms);
//     }
// }

// void yaw_filter_easy(){
//     float qw = imu.quat.w;
//     float qx = imu.quat.x;
//     float qy = imu.quat.y;
//     float qz = imu.quat.z;

//     float t3 = +2.0f * (qw * qz + qx * qy);
//     float t4 = +1.0f - 2.0f * (qy * qy + qz * qz);  
//     imu_ang[0] = std::atan2(t3, t4); // CW系
// }















// BACKUP

// 1号機

// #include "mbed.h"
// #include "ps5_can_lib.h"
// #include "IncEnc_board.h"
// #include "BNO055.h"
// #include "gm6020_lib/gm6020.h"
// #include <cstring>
// #include <cmath>

// #ifndef M_PI
// #define M_PI 3.14159265358979323846f
// #endif

// struct Position{
//     float pos_x; 
//     float pos_y;
//     float angle;
// };

// constexpr int co446_id[2] = {0x05, 0x06}; 
// const int gm_motor_num = 4;
// const float gm_gains[3] = {16.0f, 11.f, 0.0f};
// const float invSqrt = 0.707106f;
// const float ROBOT_YAW_KP = 5.f;  
// const float ROBOT_YAW_KD = 0.1f;  
// const float MAX_TURN_SPEED = 5000.0f; 

// DigitalOut leds[4]={ PC_0, PC_1, PC_2, PC_3 };
// EventQueue queue_gm(32 * EVENTS_EVENT_SIZE);
// CAN command_can(PA_11, PA_12, 1e6);
// CAN gm_can(PB_5, PB_6, 1e6);

// gm6020 gm_omni(gm_can, true, gm_motor_num); 
// can920 ps5(command_can, 1);
// Thread ctrl_thread, rcv_thread;
// BNO055 imu(PB_9, PB_8); 
// IncEnc_board odom(command_can, 2);

// int set_spd[gm_motor_num] = {0};
// int motor_output[gm_motor_num] = {0};
// int64_t now_angles[2] = {0}, prev_angles[2] = {0};
// Position robot_pos = {0, 0, 0};
// float target_yaw_rad = 0.0f; 
// float current_yaw_rad = 0.0f; 
// bool is_manual_turning = false; 

// void update_omni_speed(float input_x, float input_y, float input_turn, float current_angle_rad, int *target_spd) {
//     // float cos_val = cos(current_angle_rad);
//     // float sin_val = sin(current_angle_rad);

//     // float rot_x = input_x * cos_val + input_y * sin_val;
//     // float rot_y = -input_x * sin_val + input_y * cos_val;

//     // target_spd[0] = (int)(invSqrt * (-rot_y + rot_x) + input_turn); // 右前
//     // target_spd[1] = (int)(invSqrt * ( rot_y + rot_x) + input_turn); // 右後
//     // target_spd[2] = (int)(invSqrt * ( rot_y - rot_x) + input_turn); // 左後
//     // target_spd[3] = (int)(invSqrt * (-rot_y - rot_x) + input_turn); // 左前

//     target_spd[0] = (int)(invSqrt * (-input_y + input_x) + input_turn); // 右前
//     target_spd[1] = (int)(invSqrt * (-input_y - input_x) + input_turn); // 右後
//     target_spd[2] = (int)(invSqrt * ( input_y - input_x) + input_turn); // 左後
//     target_spd[3] = (int)(invSqrt * ( input_y + input_x) + input_turn); // 左前
// }

// void can_receive_loop_gm() {
//     while(true){        
//         gm_omni.can_read();
//     }
// }

// void onReceive_gm() {
//     queue_gm.call(can_receive_loop_gm);
//     leds[1] = !leds[1];
// }

// void odometry(){ // 自己位置推定ロジック
//     odom.conv_data_all(now_angles);

//     // float diff_x = (now_angles[0] - prev_angles[0]) * M_PI * 0.1 / 360.0;
//     // float diff_y = (now_angles[1] - prev_angles[1]) * M_PI * 0.1 / 360.0; // せっかく累積角返してくれるのに積分するリスク侵さなくてもいいのでは、、？

//     float ang_x = now_angles[0] * M_PI * 0.1 / 360;
//     float ang_y = now_angles[1] * M_PI * 0.1 / 360;

//     imu.get_angles();
//     float theta = imu.euler.yaw; 
//     robot_pos.pos_x += ang_x * cos(theta) - ang_y * sin(theta);
//     robot_pos.pos_y += ang_x * sin(theta) + ang_y * sin(theta);

//     // prev_angles[0] = now_angles[0];
//     // prev_angles[1] = now_angles[1];

// }

// float normalize_angle(float angle) {
//     while (angle > M_PI) angle -= 2.0f * M_PI;
//     while (angle < -M_PI) angle += 2.0f * M_PI;
//     return angle;
// }

// int main() {
//     ps5.setup(10);
//     for(auto& l : leds) l = false;
//     leds[0] = true; 

//     rcv_thread.start(callback(&queue_gm, &EventQueue::dispatch_forever));
//     gm_can.attach(onReceive_gm, CAN::RxIrq);
    
//     ctrl_thread.start([](){
//         while(true) {
//             gm_omni.spd_control(set_spd, motor_output, gm_gains[0], gm_gains[1], gm_gains[2]);
//             ThisThread::sleep_for(1ms);
//         }
//     });

//     CANMessage commands, pos_data;

//     int data[PS5::ALL_BUTTON];
//     bool stop_signal;
//     uint8_t data_for_05[8];
//     uint8_t data_for_06[8];

//     imu.setmode(OPERATION_MODE_IMUPLUS); // NDOFは相対角度だが、地磁気を検知した瞬間それもセンサフュージョンしちゃうから角度がおかしくなるらしい
//     ThisThread::sleep_for(100ms);
//     imu.set_angle_units(RADIANS); 
//     // imu.set_accel_units(MPERSPERS);
//     imu.get_calib();

//     imu.get_angles();
//     // target_yaw_rad = imu.euler.yaw;

//     // float prev_yaw_error = 0.0f;

//     float stick_x = 0.0f;
//     float stick_y = 0.0f;
//     float turn_cmd = 0.0f;


//     while(true) {
//         int val = ps5.get_data(data, &stop_signal);
//         leds[2] = (val == 1);

//         imu.get_angles();
//         current_yaw_rad = imu.euler.yaw; 

//         odometry(); 

//         // float stick_x = 0.0f;
//         // float stick_y = 0.0f;
//         //float turn_cmd = 0.0f;

//         if(val == 1) {
//             stick_x = 0.0f;
//             stick_y = 0.0f;
//             turn_cmd = 0.0f;
//             if(abs(data[PS5::LSTICKX]) > 10) stick_x = (float)data[PS5::LSTICKX];
//             if(abs(data[PS5::LSTICKY]) > 10) stick_y = (float)data[PS5::LSTICKY];
//             float stick_turn = (float)data[PS5::RSTICKX];

//             // if (abs(stick_turn) > 15) { 
//             //     is_manual_turning = true;
//             //     turn_cmd = stick_turn * 2.0f; 
//             //     target_yaw_rad = current_yaw_rad;
//             //     prev_yaw_error = 0.0f;

//             // } else {
//             //     is_manual_turning = false;
//             //     float error = normalize_angle(target_yaw_rad - current_yaw_rad);
//             //     float p_term = error * ROBOT_YAW_KP * 1000.0f; 
//             //     float d_term = (error - prev_yaw_error) * ROBOT_YAW_KD * 1000.0f;
                
//             //     turn_cmd = p_term + d_term;
//             //     prev_yaw_error = error;
//             //     if(turn_cmd > MAX_TURN_SPEED) turn_cmd = MAX_TURN_SPEED;
//             //     if(turn_cmd < -MAX_TURN_SPEED) turn_cmd = -MAX_TURN_SPEED;
//             // }

//             //update_omni_speed(stick_x, stick_y, turn_cmd, current_yaw_rad, set_spd);
//             update_omni_speed(stick_x, stick_y, stick_turn, current_yaw_rad, set_spd);

//             memset(data_for_05, 0, 8);
//             memset(data_for_06, 0, 8);
//             if(data[PS5::PSBUTTON]) {
//                 memset(commands.data, 0xff, 8);
//                 commands.id = co446_id[0]; command_can.write(commands);
//                 wait_us(200);
//                 commands.id = co446_id[1]; command_can.write(commands);
//                 NVIC_SystemReset();
//             }

//             if(data[PS5::CIRCLE])   data_for_05[0] = 0x01;
//             else if(data[PS5::SQUARE]) data_for_05[0] = 0x10;

//             if(data[PS5::LEFT])     data_for_05[1] = 0x01;
//             else if(data[PS5::RIGHT])  data_for_05[1] = 0x10;
            
//             if(data[PS5::R1])       data_for_05[2] = 0x01;

//             if(data[PS5::TRIANGLE]) data_for_06[0] = 0x01;
//             else if(data[PS5::CROSS])  data_for_06[0] = 0x10;

//             if(data[PS5::R2])       data_for_06[1] = 0x01;
//             else if(data[PS5::L2])     data_for_06[1] = 0x10;

//         } else {
//             //for(int i=0; i<gm_motor_num; i++) set_spd[i] = 0;
//         }

//         // 砲塔の制御では極座標のほうが扱いやすいので変換(送信データも圧縮できる)//できてなさそう？>>出来てませんでしたﾃﾍﾍﾟﾛ
//         int send_r = (int)sqrt(pow(robot_pos.pos_x, 2) + pow(robot_pos.pos_y, 2));
//         int16_t send_ang = (int)(robot_pos.angle * 180.0f / M_PI); // 度数法へ

//         pos_data.id = 0x10;

//         pos_data.data[0] = (uint8_t)(send_r >> 24) & 0xFF;
//         pos_data.data[1] = (uint8_t)(send_r >> 16) & 0xFF;
//         pos_data.data[2] = (uint8_t)(send_r >> 8) & 0xFF;
//         pos_data.data[3] = (uint8_t)(send_r >> 0) & 0xFF;


//         pos_data.data[4] = (uint8_t)(send_ang >> 8) & 0xFF;
//         pos_data.data[5] = (uint8_t)(send_ang >> 0) & 0xFF;
//         pos_data.data[6] = (uint8_t)((int16_t)robot_pos.angle >> 8) & 0xFF;
//         pos_data.data[7] = (uint8_t)((int16_t)robot_pos.angle >> 0) & 0xFF; // 自身の向き情報を追加

//         // pos_data.data[4] = (uint8_t)(send_ang >> 24) & 0xFF;
//         // pos_data.data[5] = (uint8_t)(send_ang >> 16) & 0xFF;
//         // pos_data.data[6] = (uint8_t)(send_ang >> 8) & 0xFF;
//         // pos_data.data[7] = (uint8_t)(send_ang >> 0) & 0xFF;



//         command_can.write(pos_data);
//         wait_us(200);

//         commands.id = co446_id[0];
//         memcpy(commands.data, data_for_05, 8);
//         command_can.write(commands);
//         wait_us(200);

//         commands.id = co446_id[1];
//         memcpy(commands.data, data_for_06, 8);
//         command_can.write(commands);

//         leds[3] = (gm_omni.rbms_send(motor_output) == 1);
        //leds[3]=!leds[3];
//         ThisThread::sleep_for(10ms);
//     }
// }


// ２号機(本番機)

// #include "mbed.h"
// #include "ps5_can_lib.h"
// #include "IncEnc_board.h"
// #include "BNO055.h"
// #include <cstring>
// #include <cmath>

// #ifndef M_PI
// #define M_PI 3.14159
// #endif

// #define POS_ID 0x10

// struct Position{
//     float pos_x = 0.0f; 
//     float pos_y = 0.0f;
//     float angle = 0.0f;
// };

// constexpr int co446_id[3] = {0x05, 0x06, 0x07}; 
// DigitalOut leds[4]={ PC_0, PC_1, PC_2, PC_3 };
// EventQueue queue_gm(32 * EVENTS_EVENT_SIZE);
// CAN command_can(PA_11, PA_12, 1e6);
// CAN odom_can(PB_5, PB_6, 1e6);

// can920 ps5(command_can, 1);
// BNO055 imu(PB_9, PB_8); 
// IncEnc_board odom(odom_can, 2);


// bool get_ang();
// void yaw_filter_easy();


// float quat[4]={0.f};
// float imu_ang[3]={0.f};//yaw,roll,pitch
// int64_t now_angles[2] = {0}, prev_angles[2] = {0};
// Position robot_pos = {0, 0, 0};
// float target_yaw_rad = 0.0f; 
// float current_yaw_rad = 0.0f; 
// bool is_manual_turning = false; 
// float _pos_x, _pos_y;



// void odometry(){ // 自己位置推定ロジック
//     odom.conv_data_all(now_angles);

//     // float diff_x = (now_angles[0] - prev_angles[0]) * M_PI * 0.1 / 360.0;
//     // float diff_y = (now_angles[1] - prev_angles[1]) * M_PI * 0.1 / 360.0; // せっかく累積角返してくれるのに積分するリスク侵さなくてもいいのでは、、？

//     float ang_x = now_angles[0] * M_PI * 0.1 / 360; 
//     float ang_y = now_angles[1] * M_PI * 0.1 / 360; 

//     get_ang();
//     current_yaw_rad = imu_ang[0];
//     _pos_x += ang_x * cos(current_yaw_rad) + ang_y * sin(current_yaw_rad);
//     _pos_y +=-ang_x * sin(current_yaw_rad) + ang_y * cos(current_yaw_rad); 

//     // prev_angles[0] = now_angles[0];
//     // prev_angles[1] = now_angles[1];

// }

// float normalize_angle(float angle) {
//     while (angle > M_PI) angle -= 2.0f * M_PI;
//     while (angle < -M_PI) angle += 2.0f * M_PI;
//     return angle;
// }

// int main() {
//     ps5.setup(20); // 10
//     for(auto& l : leds) l = false;
//     leds[0] = true; 

//     CANMessage commands, pos_data;
//     pos_data.id = POS_ID;

//     int data[PS5::ALL_BUTTON];
//     bool stop_signal;
//     uint8_t data_for_co446[3][8]; // 0x05, 0x06, 0x07

// //毎度setmodeしないといけないっぽい？自作関数に移動 
//     // while(!(leds[1] = imu.calib)){
//     //     imu.get_calib();
//     //     ThisThread::sleep_for(1ms);
//     // } // キャリブレーションが終わるまで待機
// // // キャリブレーションが完了したらled点灯<<いったん戻し

//     ThisThread::sleep_for(100ms);
//     imu.set_angle_units(RADIANS); 
//     imu.set_accel_units(MPERSPERS);

//     get_ang();
//     current_yaw_rad = target_yaw_rad = imu_ang[0];

//     float stick_x = 0.0f;
//     float stick_y = 0.0f;
//     float turn_cmd = 0.0f;

//     while(true) {
//         int val = ps5.get_data(data, &stop_signal);
//         leds[2] = (val == 1);

//         odometry();//ローカル変数thetaとcurrent_yaw_rad統合
//         leds[3]=imu_ang[0]>0;

//         if(val == 1) {
//             stick_x = 0;
//             stick_y = 0;
//             turn_cmd = 0;
//             if(abs(data[PS5::LSTICKX]) > 10) stick_x = (float)data[PS5::LSTICKX];
//             if(abs(data[PS5::LSTICKY]) > 10) stick_y = (float)data[PS5::LSTICKY];
//             float stick_turn = (float)data[PS5::RSTICKX];

//             for(auto& i : data_for_co446)
//                 memset(i, 0, 8);

//             if(data[PS5::PSBUTTON]) {
//                 memset(commands.data, 0xff, 8);
//                 for(auto i:co446_id){//通常送信に倣って普通のfor文でもいいかも
//                     commands.id = i;
//                     command_can.write(commands);
//                     wait_us(200);
//                 }
//                 NVIC_SystemReset();
//             }
//             //mini1(足回り)宛て
//             data_for_co446[0][0] = (uint8_t)(stick_x);
//             data_for_co446[0][1] = (uint8_t)(stick_y);
//             data_for_co446[0][2] = (uint8_t)(stick_turn);
//             //deka(砲塔)宛て
//             if(data[PS5::CIRCLE])   data_for_co446[0][3] = 0x01;
//             //else if(data[PS5::SQUARE]) data_for_co446[1][0] = 0x10;

//             if(data[PS5::R2])       data_for_co446[0][4] = 0x01;
//             else if(data[PS5::L2])     data_for_co446[0][4] = 0x10;

//             if(data[PS5::TRIANGLE]) data_for_co446[0][5] = 0x01;
//             else if(data[PS5::CROSS])  data_for_co446[0][5] = 0x10;
            
//             if(data[PS5::R1])       data_for_co446[0][7] = 0x01;//後で消す
//             //mini2(その他)宛て
//             if(data[PS5::LEFT])     data_for_co446[0][6] = 0x01;
//             else if(data[PS5::RIGHT])  data_for_co446[0][6] = 0x10;
            
//             if(data[PS5::R1])       data_for_co446[0][7] = 0x01;

//         } else {
//             //for(int i=0; i<gm_motor_num; i++) set_spd[i] = 0;
//         }

//         //デバッグ用
//         robot_pos.pos_x = _pos_x;
//         robot_pos.pos_y = _pos_y;
//         robot_pos.angle=imu_ang[0]*10;//精度は弧度法のが出る。
//         //pos_data.id = 0x10;//id設定は上部に移動(わずかに処理減らしたいという好み.戻してもいい)

//         pos_data.data[0] = (uint8_t)((int16_t)(robot_pos.pos_x * 10) >> 8) & 0xFF; // 一応10倍して精度mmにしてパッキング
//         pos_data.data[1] = (uint8_t)((int16_t)(robot_pos.pos_x * 10) >> 0) & 0xFF;
//         pos_data.data[2] = (uint8_t)((int16_t)(robot_pos.pos_y * 10) >> 8) & 0xFF;
//         pos_data.data[3] = (uint8_t)((int16_t)(robot_pos.pos_y * 10) >> 0) & 0xFF;


//         pos_data.data[4] = (uint8_t)((int16_t)(robot_pos.angle * 10) >> 8) & 0xFF;
//         pos_data.data[5] = (uint8_t)((int16_t)(robot_pos.angle * 10) >> 0) & 0xFF; 
//         pos_data.data[6] = 0;//デバッグ用
//         pos_data.data[7] = 0;
//         // pos_data.data[6] = (uint8_t)((int16_t)(robot_pos.angle * 10) >> 8) & 0xFF; 
//         // pos_data.data[7] = (uint8_t)((int16_t)(robot_pos.angle * 10) >> 0) & 0xFF; // 自身の向き情報を追加

//         // pos_data.data[4] = (uint8_t)(send_ang >> 24) & 0xFF;
//         // pos_data.data[5] = (uint8_t)(send_ang >> 16) & 0xFF;
//         // pos_data.data[6] = (uint8_t)(send_ang >> 8) & 0xFF;
//         // pos_data.data[7] = (uint8_t)(send_ang >> 0) & 0xFF;

//         command_can.write(pos_data);
//         wait_us(200);

//         for(int i=0;i<1;i++){
//             commands.id = co446_id[i];
//             memcpy(commands.data, data_for_co446[i], 8);
//             command_can.write(commands);
//             wait_us(200);
//         }

//         //leds[3] = (gm_omni.rbms_send(motor_output) == 1);
//         leds[1]=!leds[1];
//         ThisThread::sleep_for(10ms);
//     }
// }

// bool get_ang(){
//     yaw_filter_easy();
//     imu.get_angles();
//     imu_ang[1]=imu.euler.roll;
//     imu_ang[2]=imu.euler.pitch;
//     return abs(imu_ang[1]-imu_ang[2])>=M_PI/180;
// }

// void yaw_filter_easy(){
//         imu.setmode(OPERATION_MODE_IMUPLUS); // NDOFは相対角度だが、地磁気を検知した瞬間それもセンサフュージョンしちゃうから角度がおかしくなるらしい
//         imu.get_calib();
//         imu.get_quat();
//         quat[0]=imu.quat.w;
//         quat[1]=imu.quat.x;
//         quat[2]=imu.quat.y;
//         quat[3]=imu.quat.z;
//         imu_ang[0] = -std::atan2(+2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), +1.0 - 2.0 * (quat[2]*quat[2] + quat[3] * quat[3]));
// }
```
