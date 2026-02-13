#include "Copter.h"

// 移除不必要的全局目标变量，因为 wp_nav 会管理它
// static Vector3p star_guided_pos_target_cm; 
bool star_guided_pos_terrain_alt; 

// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    // start in velaccel control mode
    // 启动位置控制相关参数
    pos_control_start();

    path_num = 0;
    generate_path();
    
    // 初始化航点导航库
    wp_nav->wp_and_spline_init();
    
    // 设置第一个目标点 (path[0]是当前悬停点，path[1]是第一个顶点)
    // 为了让飞机直接动起来，我们可以直接指向 path[1]
    // 或者是先飞到 path[0] (原地)，再由 run() 切换。这里保持你的逻辑，先原地锁定。
    wp_nav->set_wp_destination(path[path_num], false);

    return true;
}

void ModeDrawStar::generate_path()
{
    float radius_cm = g2.star_radius_cm; // 10米半径
    
    // // 获取当前刹车悬停点作为中心参考点 (EKF Origin frame)
    // if (!wp_nav->get_wp_stopping_point(path[0])) {
    //     // 如果获取失败（比如没解锁或没有位置定），使用当前惯导位置
    //     path[0] = copter.inertial_nav.get_position_neu_cm();
    // }
    wp_nav->get_wp_stopping_point(path[0]);

    // 注意：ArduPilot 位置通常是 NEU (North, East, Up)
    // 下面的数学计算生成的五角星将以“正北”为基准方向
    path[1] = path[0] + Vector3f(radius_cm, 0, 0); 
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)) * radius_cm, -sinf(radians(36.0f)) * radius_cm, 0);
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)) * radius_cm, cosf(radians(18.0f)) * radius_cm, 0);
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)) * radius_cm, -cosf(radians(18.0f)) * radius_cm, 0);
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)) * radius_cm, sinf(radians(36.0f)) * radius_cm, 0);
    
    // 回到第一个点闭合
    path[6] = path[1];
}

void ModeDrawStar::run()
{
    // 1. 状态机逻辑：检查是否到达航点
    if (wp_nav->reached_wp_destination()) {
        if (path_num < 6) { // 注意这里是 < 6，因为我们有 path[0] 到 path[6]
            path_num++;
            // set_wp_destination 第二个参数 false 表示"不要飞过"，要在该点减速停顿(Sharp Corner)
            // 如果想要平滑过弯，可以研究 set_wp_destination_next (Spline)
            if (wp_nav->set_wp_destination(path[path_num], false)) {
                // 可以在这里加个日志 GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Star Point %d", path_num);
            }
        }
    }

    // 2. 核心控制逻辑
    pos_control_run();
}

void ModeDrawStar::pos_control_start()
{
    // 初始化水平速度、加速度限制
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // 初始化垂直速度、加速度
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // 初始化控制器
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // 初始化 Yaw (指向下一个航点 或 保持机头朝向)
    auto_yaw.set_mode_to_default(false);

    star_guided_pos_terrain_alt = false;
}

// 核心修改都在这里
void ModeDrawStar::pos_control_run()
{
    // --- 1. 处理飞行员 Yaw 输入 ---
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // --- 2. 安全检查 ---
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // --- 3. 地形跟随处理 (如果需要) ---
    float terr_offset = 0.0f;
    if (star_guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        copter.failsafe_terrain_on_event();
        return;
    }

    // 设置电机状态
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // --- 4. 运行航点导航器 (CRITICAL FIX) ---
    // update_wpnav() 会计算当前的"平滑目标位置"(intermediate target)
    // 并自动将其输入到 pos_control 内部的 input 接口中
    wp_nav->update_wpnav();

    // --- 5. 运行位置控制器 ---
    // 这些函数会根据 update_wpnav 设定的内部目标，计算 roll/pitch/throttle
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

uint32_t ModeDrawStar::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

#include "Copter.h"

// #if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */

// static Vector3p star_guided_pos_target_cm;       // position target (used by posvel controller only)
// bool star_guided_pos_terrain_alt;                // true if star_guided_pos_target_cm.z is an alt above terrain


// // init - initialise guided controller
// bool ModeDrawStar::init(bool ignore_checks)
// {
//     // start in velaccel control mode
//     pos_control_start();

//     path_num = 0;
//     generate_path();
//     if (path_num < 7) {
//         wp_nav->set_wp_destination(path[path_num], false);
//         // 正确赋值：将Vector3f的path[0]转换为Vector3p
//         star_guided_pos_target_cm = Vector3p(path[path_num].x, path[path_num].y, path[path_num].z);
//     }
//     return true;
// }

// void ModeDrawStar::generate_path()
// {
//     float radius_cm = 1000.0f;
//     wp_nav->get_wp_stopping_point(path[0]);
//     path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
//     path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
//     path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
//     path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
//     path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
//     path[6] = path[1];

// }

// // void ModeDrawStar::run()
// // {
// //     if (path_num == 0 && wp_nav->reached_wp_destination()) {
// //         path_num++;
// //         wp_nav->set_wp_destination(path[path_num], false);
// //         star_guided_pos_target_cm = Vector3p(path[path_num].x, path[path_num].y, path[path_num].z);
// //     }

// //     // 常规路径推进
// //     if (path_num > 0 && path_num < 6) {
// //         if (wp_nav->reached_wp_destination()) {
// //             path_num++;
// //             wp_nav->set_wp_destination(path[path_num], false);
// //             star_guided_pos_target_cm = Vector3p(path[path_num].x, path[path_num].y, path[path_num].z);
// //         }
// //     }
// //     pos_control_run();
// // }

// void ModeDrawStar::run()
// {
//     // 只有在 没到达 时才不处理
//     if (!wp_nav->reached_wp_destination()) {
//         pos_control_run();
//         return;
//     }

//     // 已经到达 → 只允许 +1，不允许连续跳
//     if (path_num < 6) {
//         path_num++;
//         wp_nav->set_wp_destination(path[path_num], false);
//         star_guided_pos_target_cm = Vector3p(path[path_num].x, path[path_num].y, path[path_num].z);
//     }

//     pos_control_run();
// }

// // initialise guided mode's position controller
// void ModeDrawStar::pos_control_start()
// {
//     // set to position control mode
//     // guided_mode = SubMode::Pos;

//     // initialise position controller
//     // wp_nav->wp_and_spline_init();
//     // wp_nav->set_wp_destination(path[0], false);
//     pva_control_start();
// }

// void ModeDrawStar::pva_control_start()
// {
//     // initialise horizontal speed, acceleration
//     pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
//     pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

//     // initialize vertical speeds and acceleration
//     pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
//     pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

//     // initialise velocity controller
//     pos_control->init_z_controller();
//     pos_control->init_xy_controller();

//     // initialise yaw
//     auto_yaw.set_mode_to_default(false);

//     // initialise terrain alt
//     star_guided_pos_terrain_alt = false;
// }


// // pos_control_run - runs the guided position controller
// // called from guided_run
// void ModeDrawStar::pos_control_run()
// {
//     // process pilot's yaw input
//     float target_yaw_rate = 0;

//     if (!copter.failsafe.radio && use_pilot_yaw()) {
//         // get pilot's desired yaw rate
//         target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
//         if (!is_zero(target_yaw_rate)) {
//             auto_yaw.set_mode(AUTO_YAW_HOLD);
//         }
//     }

//     // if not armed set throttle to zero and exit immediately
//     if (is_disarmed_or_landed()) {
//         // do not spool down tradheli when on the ground with motor interlock enabled
//         make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
//         return;
//     }

//     // calculate terrain adjustments
//     float terr_offset = 0.0f;
//     if (star_guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
//         // failure to set destination can only be because of missing terrain data
//         copter.failsafe_terrain_on_event();
//         return;
//     }

//     // set motors to full range
//     motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//     // send position and velocity targets to position controller


//     // // stop rotating if no updates received within timeout_ms
//     // if (millis() - star_update_time_ms > get_timeout_ms()) {
//     //     if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
//     //         auto_yaw.set_rate(0.0f);
//     //     }
//     // }

//     float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
//     if (star_guided_pos_terrain_alt) {
//         pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(star_guided_pos_target_cm.z));
//     }
//     pos_control->input_pos_xyz(star_guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

//     // run position controllers
//     pos_control->update_xy_controller();    //水平（XY轴）位置控制
//     pos_control->update_z_controller();     //垂直（Z轴）高度控制

//     // call attitude controller
//     if (auto_yaw.mode() == AUTO_YAW_HOLD) {
//         // roll & pitch from position controller, yaw rate from pilot
//         attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
//     } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
//         // roll & pitch from position controller, yaw rate from mavlink command or mission item
//         attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
//     } else {
//         // roll & pitch from position controller, yaw heading from GCS or auto_heading()
//         attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
//     }
// }

// uint32_t ModeDrawStar::get_timeout_ms() const
// {
//     return MAX(copter.g2.guided_timeout, 0.1) * 1000;
// }


