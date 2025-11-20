#include "swarm_node.h"
#include <cstdint>

// 文件作用域标志：编队时间轴是否已经初始化（每个单机实例一份）
static bool formation_started = false;

// mc_control_instance* mc_control_instance::instance = nullptr;
Swarm_Node::Swarm_Node() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::swarm_node)
{
}

Swarm_Node::~Swarm_Node()
{
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool Swarm_Node::init()
{
    ScheduleOnInterval(20000_us); // 200 Hz
    return true;
}

bool Swarm_Node::takeoff()
{
    control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5);
    return false;
}

bool Swarm_Node::arm_offboard()
{
    return false;
}

bool Swarm_Node::swarm_node_init()
{
    _vehicle_local_position_sub.copy(&_vehicle_local_position);
    _a01_sub.copy(&_target);
    _a02_sub.copy(&_start_flag);

    if ((_vehicle_local_position.xy_valid) && _start_flag.start_swarm) {

        vehicle_status_s _vehicle_status;

        if (_vehicle_status_sub.copy(&_vehicle_status)) {

            vehicle_id = _vehicle_status.system_id;

            if (vehicle_id > 1) {

                _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,
                                                     _vehicle_local_position.ref_lon,
                                                     hrt_absolute_time());

                _global_local_proj_ref.project(_target.lat, _target.lon, target_x, target_y);

                float dist = sqrtf((_vehicle_local_position.x - target_x) *
                                   (_vehicle_local_position.x - target_x) +
                                   (_vehicle_local_position.y - target_y) *
                                   (_vehicle_local_position.y - target_y));

                if (dist < 200) {
                    _vehicle_local_position_sub.copy(&_vehicle_local_position);
                    begin_x = _vehicle_local_position.x;
                    begin_y = _vehicle_local_position.y;
                    begin_z = _vehicle_local_position.z;

                    _global_local_proj_ref.initReference(_vehicle_local_position.ref_lat,
                                                         _vehicle_local_position.ref_lon,
                                                         hrt_absolute_time());
                    // 对从机也需要初始化 time_tick（如果还没初始化）
                    if (!formation_started) {
                        time_tick = hrt_absolute_time();
                        formation_started = true;

                        // 如果所有 time_tick_point 都为 0（未配置），给默认时间点（示例值：每段 5 秒）
                        if (time_tick_point1 == 0 && time_tick_point2 == 0 &&
                            time_tick_point3 == 0 && time_tick_point4 == 0 &&
                            time_tick_point5 == 0) {
                            const uint64_t sec = 1000ULL * 1000ULL;
                            time_tick_point1 = 5ULL * sec;   // 5s
                            time_tick_point2 = 10ULL * sec;  // 10s
                            time_tick_point3 = 15ULL * sec;  // 15s
                            time_tick_point4 = 20ULL * sec;  // 20s
                            time_tick_point5 = 25ULL * sec;  // 25s
                        }
                    }

                    return true;
                } else {
                    return false;
                }

            } else if (vehicle_id >= 1) {
                // leader 或其他 id==1 情况也要初始化 time_tick（如果还没初始化）
                if (!formation_started) {
                    time_tick = hrt_absolute_time();
                    formation_started = true;

                    // 如果 time_tick_point 未被设置，则设默认值（同上）
                    if (time_tick_point1 == 0 && time_tick_point2 == 0 &&
                        time_tick_point3 == 0 && time_tick_point4 == 0 &&
                        time_tick_point5 == 0) {
                        const uint64_t sec = 1000ULL * 1000ULL;
                        time_tick_point1 = 5ULL * sec;   // 5s
                        time_tick_point2 = 10ULL * sec;  // 10s
                        time_tick_point3 = 15ULL * sec;  // 15s
                        time_tick_point4 = 20ULL * sec;  // 20s
                        time_tick_point5 = 25ULL * sec;  // 25s
                    }
                }
                return true;
            }
        }

    } else {
        return false;
    }

    return false;
}

void Swarm_Node::start_swarm_node()
{
    if (vehicle_id > 1) {

        a02_s _a02{};
        _a02_sub.copy(&_a02);

        if (_a02.stop_swarm) {
            control_instance::getInstance()->Change_land();
        } else {

            _vehicle_local_position_sub.copy(&_vehicle_local_position);
            _a01_sub.copy(&_target);

            // 计算目标点（保留原有投影）
            _global_local_proj_ref.project(_target.lat, _target.lon, target_x, target_y);

            // 判断是否达到第 3 个任务点 (time_tick_point2)
            uint64_t elapsed = 0;
            if (formation_started && time_tick > 0) {
                // 安全计算 elapsed
                uint64_t now = hrt_absolute_time();
                if (now > time_tick) {
                    elapsed = now - time_tick;
                } else {
                    elapsed = 0;
                }
            }

            // 切换且保持: 只有在 formation 已初始化且 time_tick_point2>0 时才可能切换
            bool use_line_formation = false;
            if (formation_started && time_tick_point2 > 0) {
                use_line_formation = (elapsed >= time_tick_point2);
            } else {
                use_line_formation = false; // 未初始化或参数未设置 -> 先保持正方形
            }

            //--------------------------------------------
            // 阵型偏移（2~5号），1号为长机（不参与）
            //--------------------------------------------
            float dx = 0.0f;
            float dy = 0.0f;

            if (!use_line_formation) {
                // 到第3点前：正方形队形 (右侧平移避免与 1 号重合)
                if (vehicle_id == 2) { dx = 5.0f;  dy = 0.0f; }
                if (vehicle_id == 3) { dx = 5.0f;  dy = 5.0f; }
                if (vehicle_id == 4) { dx = 10.0f; dy = 5.0f; }
                if (vehicle_id == 5) { dx = 10.0f; dy = 0.0f; }
            } else {
                // 到达第3点后：一字型队形 (保持)
                if (vehicle_id == 2) { dx = 5.0f;   dy = 0.0f; }
                if (vehicle_id == 3) { dx = 10.0f;  dy = 0.0f; }
                if (vehicle_id == 4) { dx = 15.0f;  dy = 0.0f; }
                if (vehicle_id == 5) { dx = 20.0f;  dy = 0.0f; }
            }

            // 发送期望位置（保持高度偏移 begin_z - 5）
            control_instance::getInstance()->Control_posxyz(
                target_x + dx,
                target_y + dy,
                begin_z - 5.0f
            );
        }
    }

    // -------------------- 1号无人机原逻辑保持不变 --------------------
    if (vehicle_id == 1) {

        if ((hrt_absolute_time() - time_tick > 0) &&
            (hrt_absolute_time() - time_tick < time_tick_point1)) {

            control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5);
        }

        if ((hrt_absolute_time() - time_tick > time_tick_point1) &&
            (hrt_absolute_time() - time_tick < time_tick_point2)) {

            control_instance::getInstance()->Control_posxyz(begin_x + 5, begin_y, begin_z - 5);
        }

        if ((hrt_absolute_time() - time_tick > time_tick_point2) &&
            (hrt_absolute_time() - time_tick < time_tick_point3)) {

            control_instance::getInstance()->Control_posxyz(begin_x + 5, begin_y + 5, begin_z - 5);
        }

        if ((hrt_absolute_time() - time_tick > time_tick_point3) &&
            (hrt_absolute_time() - time_tick < time_tick_point4)) {

            control_instance::getInstance()->Control_posxyz(begin_x, begin_y + 5, begin_z - 5);
        }

        if ((hrt_absolute_time() - time_tick > time_tick_point4) &&
            (hrt_absolute_time() - time_tick < time_tick_point5)) {

            control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5);
        }

        if (hrt_absolute_time() - time_tick > time_tick_point5) {

            control_instance::getInstance()->Change_land();
            a02_s _a02;
            _a02.stop_swarm = true;
            _a02_pub.publish(_a02);
        }
    }
}


void Swarm_Node::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    switch (STATE) {

    case state::INIT:
        if (swarm_node_init()) {
            STATE = state::ARM_OFFBOARD;
        }
        break;

    case state::ARM_OFFBOARD:
        if (control_instance::getInstance()->Change_offborad() &&
            control_instance::getInstance()->Arm_vehicle()) {
            STATE = state::TAKEOFF;
        }
        break;

    case state::TAKEOFF:
        if (control_instance::getInstance()->Control_posxyz(begin_x, begin_y, begin_z - 5)) {
            STATE = state::CONTROL;
        }
        break;

    case state::CONTROL:
        start_swarm_node();
        break;

    case state::LAND:
        control_instance::getInstance()->Change_land();
        break;

    case state::EMERGENCY:
        break;

    default:
        break;
    }

    perf_end(_loop_perf);
}

int Swarm_Node::task_spawn(int argc, char *argv[])
{
    Swarm_Node *instance = new Swarm_Node();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int Swarm_Node::print_status()
{
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
    return 0;
}

int Swarm_Node::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int Swarm_Node::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("work_item_example", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int swarm_node_main(int argc, char *argv[])
{
    return Swarm_Node::main(argc, argv);
}

