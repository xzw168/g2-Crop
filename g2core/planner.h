/*
 * planner.h - cartesian trajectory planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2013 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2018 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/*x
 * --- Planner Background ---
 *
 *  The planner is a complicated beast that takes a lot of things into account. 
 *  Planner documentation is scattered about and co-located with the functions
 *  that perform the actions. Key files are:
 *
 *  - planner.h     - This file has defines, structures and prototypes. What you would expect
 *  - planner.cpp   - Core and common functions, queue handling, JSON and command handlers
 *  - plan_line.cpp - Move planning and queuing, backward planning functions
 *  - plan_zoid.cpp - Move forward planning, velocity contour calculations and crazy math
 *  - plan_exec.cpp - Runtime execution functions, calls zoid's forward planning functions
 *  - stepper.cpp/h - Real-time step generation, segment loading, pulls from plan_exec
 *  - plan_arc.cpp/h- Arc calculation and runtime functions - layer above the rest of this
 *
 * --- Planner Overview ---
 *
 * At high level the planner's job is to reconstruct smooth motion from a set of linear 
 * approximations while observing and operating within the physical constraints of the 
 * machine and the physics of motion. Gcode - which consists of a series of into linear 
 * motion segments - is interpreted, queued to the planner, and joined together to produce 
 * continuous, synchronized motion. Non-motion commands such as pauses (dwells) and 
 * peripheral controls such as spindles can also be synchronized in the queue. Arcs are 
 * just a special case consisting of many linear moves. Arcs are not interpreted directly.
 *
 * The planner sits in the middle of three system layers: 
 *  - The Gcode interpreter and canonical machine (the 'model'), which feeds...
 *  - The planner - taking generic commands from the model and queuing them for...
 *  - The runtime layer - pulling from the planner and driving stepper motors or other devices
 * 
 * The planner queue is the heart of the planner. It's a circular list of ~48 complex structures
 * that carry the state of the system needs to execute a linear motion, run a pre-planned command,
 * like turning on a spindle, or executing an arbitrary JSON command such as an active comment.
 *
 * The queue can be viewed as a list of instructions that will execute in exact sequence.
 * Some instructions control motion and need to be joined to their forward and backwards 
 * neighbors so that position, velocity, acceleration, and jerk constraints are not 
 * violated when moving from one motion to the next. 
 *
 * Others are "commands" that are actually just function callbacks that happen to execute
 * at a particular point in time (synchronized with motion commands). Commands can control
 * anything you can reasonably program, such as digital IO, serial communications, or 
 * interpreted commands encoded in JSON.
 *
 * The buffers in the planner queue are treated as a 'closure' - with all state needed for
 * proper execution carried in the planner structure. This is important as it keeps
 * model state coherent in a heavily pipelined system. The local copy of the Gcode
 * model is carried in the gm structure that is part of each planner buffer.
 * See header notes in planner.cpp for more details.
 *
 * The planner is entered by calling one of:
 *  - mp_aline()         - plan and queue a move with acceleration management
 *  - mp_dwell()         - plan and queue a pause (dwell) to the planner queue
 *  - mp_queue_command() - queue a canned command
 *  - mp_json_command()  - queue a JSON command for run-time interpretation and execution (M100)  
 *  - mp_json_wait()     - queue a JSON wait for run-time interpretation and execution (M101)
 *  - 
 * In addition, cm_arc_feed() valaidates and sets up a arc paramewters and calls mp_aline() 
 * repeatedly to spool out the arc segments into the planner queue.
 *
 * All the above queueing commands other than mp_aline() are relatively trivial; they just
 * post callbacks into the next available planner buffer. Command functions are in 2 parts: 
 * the part that posts to the queue, and the callback that is executed when the command is 
 * finally reached in the queue - the _exec().
 *
 * All mp_aline() does is some preliminary math and then posts an initialized buffer to 
 * the planner queue. The rest of the move planning operations takes place in background;
 * via mp_planner_callback() called from the main loop, and as 'pulls' from the runtime 
 * stepper operations.
 *
 * Motion planning is separated into backward planning and forward planning stages. 
 * Backward planning is initiated by mp_planner_callback() which is called repeatedly 
 * from the main loop. Backwards planning is performed by mp_plan_block_list() and 
 * _plan_block(). It starts at the most recently arrived Gcode block. Backward 
 * planning can occur multiple times for a given buffer, as new moves arriving 
 * can make the motion profile more optimal.
 *
 * Backward planning uses velocity and jerk constraints to set maximum entry,
 * travel (cruise) and exit velocities for the moves in the queue. In addition, 
 * it observes the maximum cornering velocities that adjoining moves can sustain 
 * in a corner or a 'kink' to ensure that the jerk limit of any axis participating 
 * in the move is not violated. See mp_planner_callback() header comments for more detail.
 *
 * Forward planning is performed just-in-time and only once, right before the 
 * planner runtime needs the next buffer. Forward planning provides the final
 * contouring of the move. It is invoked by mp_forward_plan() and executed by 
 * mp_calculate_ramps() in plan_zoid.cpp.
 *
 * Planner timing operates at a few different levels:
 *
 *  - New lines of ASCII containing commands and moves arriving from the USB are 
 *    parsed and executed as the lowest priority background task from the main loop.
 *
 *  - Backward planning is invoked by a main loop callback, so it also executes as 
 *    a background task, albeit a higher priority one.
 *
 *  - Forward planning and the ultimate preparation of the move for the runtime runs 
 *    as an interrupt as a 'pull' from the planner queue that uses a series of 
 *    interrupts at progressively lower priorities to ensure that the next planner 
 *    buffer is ready before the runtime runs out of forward-planned moves and starves.
 *
 * Some other functions performed by the planner include:
 *
 *  - Velocity throttling to ensure that very short moves do not execute faster 
 *    than the serial interface can deliver them
 *
 *  - Feed hold and cycle start (resume) operations
 *
 *  - Feed rate override functions and replanning
 *
 * Some terms that are useful that we try to use consistently:
 *
 *  - buffer  - in this context a planner buffer holding a move or a command: mb._ or bf
 *  - block   - a data structure for planning or runtime control. See mp_calculate_ramps() comments
 *  - move    - a linear Gcode move, typically from a G0 or G1 code
 *  - command - a non-move executable in the planner
 *  - group   - a collection of moves or commands that are treated as a unit
 *  - line    - a line of ASCII gcode or arbitrary text
 *  - bootstrap - the startup period where the planner collects moves but does not yet execute them
 */

#ifndef PLANNER_H_ONCE
#define PLANNER_H_ONCE

#include "canonical_machine.h" // used for GCodeState_t

using Motate::Timeout;

/*
 * Enums and other type definitions
 * All the enums that equal some starting value must be that value. Don't change them
 */

typedef void (*cm_exec_t)(float[], bool[]); // callback to canonical_machine execution function

typedef enum
{                         // 计划经营状况
    PLANNER_IDLE = 0,     // 计划者和运动都是空闲的
    PLANNER_STARTUP,      // 运动开始前摄入块
    PLANNER_PRIMING,      // 准备新的计划动作（“拼接”）
    PLANNER_BACK_PLANNING // 主动重新规划所有块，从最新添加到运行块
} plannerState;

typedef enum
{                            // bf->buffer_state values in incresing order so > and < can be used
    MP_BUFFER_EMPTY = 0,     // 缓冲区可供使用（必须为0）
    MP_BUFFER_INITIALIZING,  // 缓冲区已被检出并正在由line（）或命令初始化
    MP_BUFFER_NOT_PLANNED,   // 计划正在进行中 - 至少已设定vmax
    MP_BUFFER_BACK_PLANNED,  // 缓冲准备好进行最终规划; 速度已经确定
    MP_BUFFER_FULLY_PLANNED, // 缓冲完全计划。 可能仍然需要重新计划
    MP_BUFFER_RUNNING,       // 表示移动正在运行时执行。在此阶段，bf被“锁定”
    MP_BUFFER_POLAND,        // 希特勒用波兰作为缓冲状态 Hitler used Poland as a buffer state
    MP_BUFFER_UKRAINE        // Later Stalin did the same to Ukraine
} bufferState;

typedef enum
{                             // bf->block_type values
    BLOCK_TYPE_NULL = 0,      // MUST=0  空着 - 做一个空操作
    BLOCK_TYPE_ALINE = 1,     // MUST=1  加速计划线
    BLOCK_TYPE_COMMAND = 2,   // MUST=2  一般命令
                              // 所有其他非移动命令都是> BLOCK_TYPE_COMMAND
    BLOCK_TYPE_DWELL,         // G代码停留
    BLOCK_TYPE_JSON_WAIT,     // JSON等待命令
    BLOCK_TYPE_TOOL,          // T命令（T，不是M6换刀）
    BLOCK_TYPE_SPINDLE_SPEED, // S命令
    BLOCK_TYPE_STOP,          // 程序停止
    BLOCK_TYPE_END            // 程序结束
} blockType;

typedef enum
{
    BLOCK_INACTIVE = 0,   // 块是非活动的（必须为零）
    BLOCK_INITIAL_ACTION, // 如果需要初始化，则初始化值
    BLOCK_ACTIVE          // 运行状态
} blockState;

typedef enum
{
    SECTION_HEAD = 0, // 加速acceleration
    SECTION_BODY,     // 巡航
    SECTION_TAIL      // 减速 
} moveSection;
#define SECTIONS 3

typedef enum
{
    SECTION_OFF = 0, // 段不活跃section inactive
    SECTION_NEW,     // 未初始化段uninitialized section
    SECTION_RUNNING  // 启动并运行
} sectionState;

typedef enum
{                         // 规划梯形生成代码块
    NO_HINT = 0,          // 块未被提示
    COMMAND_BLOCK,        // 这个块是一个命令
    PERFECT_ACCELERATION, // 头部加速度冲击或无法改进
    PERFECT_DECELERATION, // 急速尾部减速或无法改善
    PERFECT_CRUISE,       // 只有车身巡航: Ve = Vc = Vx != 0
    MIXED_ACCELERATION,   // 扭曲加速到达并保持巡航（HB）
    MIXED_DECELERATION,   // 从巡航区域（BT）开始扭曲减速

    // 其余的是从zoid函数报告选择的内容。
    ZERO_VELOCITY,   // Ve = Vc = Vx = 0
    ZERO_BUMP,       // Ve = Vx = 0, Vc != 0
    SYMMETRIC_BUMP,  // (Ve = Vx) < Vc
    ASYMMETRIC_BUMP, // (Ve != Vx) < Vc
} blockHint;

/*** Most of these factors are the result of a lot of tweaking. Change with caution.***/

#define PLANNER_QUEUE_SIZE ((uint8_t)48)     // 建议12分钟。 限制为255
#define SECONDARY_QUEUE_SIZE ((uint8_t)12)   // 进给保持操作的辅助二次计划程序队列 
#define PLANNER_BUFFER_HEADROOM ((uint8_t)4) // 在处理新输入行之前，在计划程序中保留缓冲区
#define JERK_MULTIPLIER ((float)1000000)     // 请勿改变 - 必须始终为100万

#define JUNCTION_INTEGRATION_MIN (0.05) // JT minimum allowable setting
#define JUNCTION_INTEGRATION_MAX (5.00) // JT maximum allowable setting

#ifndef MIN_SEGMENT_MS               // boards can override this value in hardware.h
#define MIN_SEGMENT_MS ((float)0.75) // minimum segment milliseconds
#endif
#define NOM_SEGMENT_MS ((float)MIN_SEGMENT_MS * 2) // nominal segment ms (at LEAST MIN_SEGMENT_MS * 2)
#define MIN_BLOCK_MS ((float)MIN_SEGMENT_MS * 2)   // minimum block (whole move) milliseconds

#define BLOCK_TIMEOUT_MS ((float)30.0) // MS before deciding there are no new blocks arriving
#define PHAT_CITY_MS ((float)100.0)    // if you have at least this much time in the planner

#define NOM_SEGMENT_TIME ((float)(NOM_SEGMENT_MS / 60000)) // DO NOT CHANGE - time in minutes
#define NOM_SEGMENT_USEC ((float)(NOM_SEGMENT_MS * 1000))  // DO NOT CHANGE - time in microseconds
#define MIN_SEGMENT_TIME ((float)(MIN_SEGMENT_MS / 60000)) // DO NOT CHANGE - time in minutes
#define MIN_BLOCK_TIME ((float)(MIN_BLOCK_MS / 60000))     // DO NOT CHANGE - time in minutes
#define PHAT_CITY_TIME ((float)(PHAT_CITY_MS / 60000))     // DO NOT CHANGE - time in minutes

#define FEED_OVERRIDE_ENABLE false           // initial value
#define FEED_OVERRIDE_MIN (0.05)             // 5% minimum
#define FEED_OVERRIDE_MAX (2.00)             // 200% maximum
#define FEED_OVERRIDE_RAMP_TIME (0.500 / 60) // ramp time for feed overrides
#define FEED_OVERRIDE_FACTOR (1.00)          // initial value

#define TRAVERSE_OVERRIDE_ENABLE false  // initial value
#define TRAVERSE_OVERRIDE_MIN (0.05)    // 5% minimum
#define TRAVERSE_OVERRIDE_MAX (1.00)    // 100% maximum
#define TRAVERSE_OVERRIDE_FACTOR (1.00) // initial value

//// Specialized equalities for comparing velocities with tolerances
//// These determine allowable velocity discontinuities between blocks (among other tests)
//// RG: Simulation shows +-0.001 is about as much as we should allow.
//      VELOCITY_EQ(v0,v1) reads: "True if v0 is within 0.0001 of v1"
//      VELOCITY_LT(v0,v1) reads: "True if v0 is less than v1 by at least 0.0001"
#define VELOCITY_EQ(v0, v1) (fabs(v0 - v1) < 0.0001)
#define VELOCITY_LT(v0, v1) ((v1 - v0) > 0.0001)

#define Vthr2 300.0
#define Veq2_hi 10.0
#define Veq2_lo 1.0
#define VELOCITY_ROUGHLY_EQ(v0, v1) ((v0 > Vthr2) ? fabs(v0 - v1) < Veq2_hi : fabs(v0 - v1) < Veq2_lo)

/* Planner Diagnostics */

//#define __PLANNER_DIAGNOSTICS   // comment this out to drop diagnostics

#ifdef __PLANNER_DIAGNOSTICS
#define ASCII_ART(s) xio_writeline(s)

#define UPDATE_BF_DIAGNOSTICS(bf)                           \
    {                                                       \
        bf->linenum = bf->gm.linenum;                       \
        bf->block_time_ms = bf->block_time * 60000;         \
        bf->plannable_time_ms = bf->plannable_time * 60000; \
    }

#define UPDATE_MP_DIAGNOSTICS                               \
    {                                                       \
        mp->plannable_time_ms = mp->plannable_time * 60000; \
    }
#define SET_PLANNER_ITERATIONS(i) \
    {                             \
        bf->iterations = i;       \
    }
#define INC_PLANNER_ITERATIONS \
    {                          \
        bf->iterations++;      \
    }
#define SET_MEET_ITERATIONS(i)   \
    {                            \
        bf->meet_iterations = i; \
    }
#define INC_MEET_ITERATIONS    \
    {                          \
        bf->meet_iterations++; \
    }

#else
#define ASCII_ART(s)
#define UPDATE_BF_DIAGNOSTICS
#define UPDATE_MP_DIAGNOSTICS
#define SET_PLANNER_ITERATIONS(i)
#define INC_PLANNER_ITERATIONS
#define SET_MEET_ITERATIONS(i)
#define INC_MEET_ITERATIONS
#endif

/*
 *  Planner structures
 *
 *  Be aware of the distinction between 'buffers' and 'blocks'
 *  Please refer to header comments in for important details on buffers and blocks
 *    - plan_zoid.cpp / mp_calculate_ramps()
 *    - plan_exec.cpp / mp_exec_aline()
 */

//**** Planner Queue Structures ****

typedef struct mpBuffer
{

    // *** CAUTION *** These two pointers are not reset by _clear_buffer()
    struct mpBuffer *pv;   // 静态指针指向前一个缓冲区
    struct mpBuffer *nx;   // 静态指向下一个缓冲区
    uint8_t buffer_number; // DIAGNOSTIC，便于调试

    stat_t (*bf_func)(struct mpBuffer *bf); // 回调缓冲exec函数
    cm_exec_t cm_func;                      // 回调规范机器执行功能

#ifdef __PLANNER_DIAGNOSTICS
    uint32_t linenum; // mirror of bf->gm.linenum
    int iterations;
    float block_time_ms;
    float plannable_time_ms; // time in planner
    float plannable_length;  // length in planner
    uint8_t meet_iterations; // iterations needed in _get_meet_velocity
#endif

    bufferState buffer_state; // 用于管理排队/出队
    blockType block_type;     // 用于调度运行程序
    blockState block_state;   // 移动状态机序列
    blockHint hint;           // 暗示了块区域和其他规划操作。 必须准确或NO_HINT

    // block parameters
    float unit[AXES];      // 用于轴缩放和规划的单位矢量
    bool axis_flags[AXES]; // 为参与移动和命令参数的轴设置为true

    bool plannable; // 当此块可用于计划时设置为true

    float length;          // 线或螺旋的总长度，单位mm
    float block_time;      // 计算整个块的移动时间（移动）
    float override_factor; // 此块的进给速率或快速覆盖因子（“覆盖”是保留字）

    // *** SEE NOTES ON THESE VARIABLES, in aline() ***
    // We removed all entry_* values.
    // To get the entry_* values, look at pv->exit_* or mr->exit_*
    float cruise_velocity; // 巡航速度要求和实现
    float exit_velocity;   // 要求移动的出口速度
    // 也是* next * move的进入速度

    float cruise_vset; // 要求移动的巡航速度 - 在覆盖之前
    float cruise_vmax; // 巡航最大速度调整为覆盖
    float exit_vmax;   // 此移动可能的最大出口速度
    // 也是下一步的最大进入速度

    float absolute_vmax; // 最快这个块可以移动超过约束
    float junction_vmax; // 最大出口速度可以通过交汇点。
    // between the NEXT BLOCK AND THIS ONE

    float jerk;             // 此移动的最大线性加加速度项
    float jerk_sq;          // Jm ^ 2用于计划（计算和缓存）
    float recip_jerk;       // 1 / Jm用于计划（计算和缓存）
    float sqrt_j;           // sqrt（jM）用于规划（计算和缓存）
    float q_recip_2_sqrt_j; // (q/(2 sqrt(jM))) where q = (sqrt(10)/(3^(1/4))), used in length computations (computed and cached)

    GCodeState_t gm; // Gcode模型状态 - 从模型传递，由计划程序和运行时使用

    // clears the above structure
    void reset()
    {
        bf_func = nullptr;
        cm_func = nullptr;

#ifdef __PLANNER_DIAGNOSTICS
        linenum = 0;
        iterations = 0;
        block_time_ms = 0;
        plannable_time_ms = 0;
        plannable_length = 0;
        meet_iterations = 0;
#endif
        buffer_state = MP_BUFFER_EMPTY;
        block_type = BLOCK_TYPE_NULL;
        block_state = BLOCK_INACTIVE;
        hint = NO_HINT;

        for (uint8_t i = 0; i < AXES; i++)
        {
            unit[i] = 0;
            axis_flags[i] = 0;
        }
        plannable = false;
        length = 0.0;
        block_time = 0.0;
        override_factor = 0.0;
        cruise_velocity = 0.0;
        exit_velocity = 0.0;
        cruise_vset = 0.0;
        cruise_vmax = 0.0;
        exit_vmax = 0.0;
        absolute_vmax = 0.0;
        junction_vmax = 0.0;
        jerk = 0.0;
        jerk_sq = 0.0;
        recip_jerk = 0.0;
        sqrt_j = 0.0;
        q_recip_2_sqrt_j = 0.0;
        gm.reset();
    }
} mpBuf_t;

typedef struct mpPlannerQueue
{                              // 队列的控制结构
    magic_t magic_start;       // magic number to test memory integrity
    mpBuf_t *r;                // 运行缓冲区指针
    mpBuf_t *w;                // 写缓冲区指针
    uint8_t queue_size;        // 缓冲区总数，一个基础（例如48个不是47个）
    uint8_t buffers_available; // 运行队列中可用缓冲区的计数
    mpBuf_t *bf;               // 指向缓冲池的指针（存储阵列）
    magic_t magic_end;
} mpPlannerQueue_t;

//**** Planner Runtime structures ****

typedef struct mpBlockRuntimeBuf
{                                 // 我们需要计划BLOCK的RunTime部分的数据结构
    struct mpBlockRuntimeBuf *nx; // 单链表结构

    float head_length; // 同名bf变量的副本
    float body_length;
    float tail_length;

    float head_time; // 同名bf变量的副本
    float body_time;
    float tail_time;

    float cruise_velocity; // 头部末端的速度和尾部的开始
    float exit_velocity;   // 移动结束时的速度
} mpBlockRuntimeBuf_t;

typedef struct mpPlannerRuntime
{ // persistent runtime variables
    //  uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
    magic_t magic_start;        // magic number to test memory integrity
    blockState block_state;     // 整体行动的状态
    moveSection section;        // 移动的部分是什么？加速/巡航/减速
    sectionState section_state; // 移动部分内的状态

    bool out_of_band_dwell_flag;     // 设置为有条件地执行带外停顿
    float out_of_band_dwell_seconds; // 带外停留的时间

    float unit[AXES];               // 用于轴缩放和规划的单位矢量
    bool axis_flags[AXES];          // set true for axes participating in the move
    float target[AXES];             // final target for bf (used to correct rounding errors)
    float position[AXES];           // current move position
    float waypoint[SECTIONS][AXES]; // head/body/tail endpoints for correction

    float target_steps[MOTORS];    // current MR target (absolute target as steps)
    float position_steps[MOTORS];  // current MR position (target from previous segment)
    float commanded_steps[MOTORS]; // will align with next encoder sample (target from 2nd previous segment)
    float encoder_steps[MOTORS];   // encoder position in steps - ideally the same as commanded_steps
    float following_error[MOTORS]; // difference between encoder_steps and commanded steps

    mpBlockRuntimeBuf_t *r;       // 正在运行的块
    mpBlockRuntimeBuf_t *p;       // 正在计划的块，p可能== r
    mpBlockRuntimeBuf_t block[2]; // 缓冲器保持所述两个块

    mpBuf_t *plan_bf; // DIAGNOSTIC - 指向下一个计划缓冲区的指针
    mpBuf_t *run_bf;  // DIAGNOSTIC - 指向当前运行的缓冲区

    float entry_velocity; // entry values for the currently running block

    float segments;         // 当前正在运行的块的条目值number of segments in line (also used by arc generation)
    uint32_t segment_count; // 运行段数count of running segments
    float segment_velocity; // 计算线段的速度computed velocity for aline segment
    float segment_time;     // 每个线段的实际时间增量actual time increment per aline segment

    float forward_diff_1; // 前向差异等级1 forward difference level 1
    float forward_diff_2; // forward difference level 2
    float forward_diff_3; // forward difference level 3
    float forward_diff_4; // forward difference level 4
    float forward_diff_5; // forward difference level 5

    GCodeState_t gm; // gcode模型状态当前正在执行 gcode model state currently executing

    magic_t magic_end;

    // resets mpPlannerRuntime structure without actually wiping it
    void reset()
    {
        block_state = BLOCK_INACTIVE;
        section = SECTION_HEAD;
        section_state = SECTION_OFF;
        entry_velocity = 0;   // needed to ensure next block in forward planning starts from 0 velocity
        r->exit_velocity = 0; // ditto
        segment_velocity = 0;
    }

} mpPlannerRuntime_t;

//**** Master Planner Structure ***

typedef struct mpPlanner
{                        // common variables for a planner context
    magic_t magic_start; // magic number to test memory integrity

    // 诊断
    float run_time_remaining_ms;
    float plannable_time_ms;

    // 计划者的位置
    float position[AXES]; // 最终移动位置用于规划目的

    // 时间变量
    float run_time_remaining; // 运行时剩余的时间（包括运行块）
    float plannable_time;     // 计划者中实际可以计划的时间

    // 规划者状态变量
    plannerState planner_state; // 规划师的当前状态
    bool request_planning;      // 设置为true以请求重新计划
    bool backplanning;          // 如果规划师处于反向规划阶段，则为true
    bool mfo_active;            // 如果mfo覆盖有效，则为true
    bool ramp_active;           // 发生斜坡时为true
    bool entry_changed;         // 标记如果exit_velocity变更为下一个块的提示无效

    // 进给覆盖和渐变变量（这些变量在cm-> GMX中扩展）
    float mfo_factor; // 运行时覆盖因子
    float ramp_target;
    float ramp_dvdt;

    // objects
    Timeout block_timeout; // 块规划的超时对象

    // planner pointers
    mpBuf_t *p;               // 规划器缓冲区指针
    mpBuf_t *c;               // 紧跟在关键区域之后的指针缓冲区
    mpBuf_t *planning_return; // 缓冲区返回到一次后退计划完成
    mpPlannerRuntime_t *mr;   // 绑定到mr与此计划者相关联
    mpPlannerQueue_t q;       // 嵌入计划程序缓冲区队列管理器

    magic_t magic_end;

    // clears mpPlanner structure but leaves position alone
    void reset()
    {
        run_time_remaining = 0;
        plannable_time = 0;
        planner_state = PLANNER_IDLE;
        request_planning = false;
        backplanning = false;
        mfo_active = false;
        ramp_active = false;
        entry_changed = false;
        block_timeout.clear();
    }
} mpPlanner_t;

// Reference global scope structures

extern mpPlanner_t *mp; // currently active planner (global variable)
extern mpPlanner_t mp1; // primary planning context
extern mpPlanner_t mp2; // secondary planning context

extern mpPlannerRuntime_t *mr; // context for block runtime
extern mpPlannerRuntime_t mr1; // primary planner runtime context
extern mpPlannerRuntime_t mr2; // secondary planner runtime context

extern mpBuf_t mp1_queue[PLANNER_QUEUE_SIZE];   // storage allocation for primary planner queue buffers
extern mpBuf_t mp2_queue[SECONDARY_QUEUE_SIZE]; // storage allocation for secondary planner queue buffers

/*
 * Global Scope Functions
 */

//**** planner.cpp functions

void planner_init(mpPlanner_t *_mp, mpPlannerRuntime_t *_mr, mpBuf_t *queue, uint8_t queue_size);
void planner_reset(mpPlanner_t *_mp);
stat_t planner_assert(const mpPlanner_t *_mp);

void mp_halt_runtime(void);

void mp_set_planner_position(uint8_t axis, const float position);
void mp_set_runtime_position(uint8_t axis, const float position);
void mp_set_steps_to_runtime_position(void);

void mp_queue_command(void (*cm_exec)(float *, bool *), float *value, bool *flag);
stat_t mp_runtime_command(mpBuf_t *bf);

stat_t mp_json_command(char *json_string);
stat_t mp_json_command_immediate(char *json_string);
stat_t mp_json_wait(char *json_string);

stat_t mp_dwell(const float seconds);
void mp_end_dwell(void);
void mp_request_out_of_band_dwell(float seconds);

//**** planner functions and helpers
uint8_t mp_get_planner_buffers(const mpPlanner_t *_mp);
bool mp_planner_is_full(const mpPlanner_t *_mp);
bool mp_has_runnable_buffer(const mpPlanner_t *_mp);
bool mp_is_phat_city_time(void);

stat_t mp_planner_callback();
void mp_replan_queue(mpBuf_t *bf);
void mp_start_feed_override(const float ramp_time, const float override);
void mp_end_feed_override(const float ramp_time);
void mp_start_traverse_override(const float ramp_time, const float override);
void mp_end_traverse_override(const float ramp_time);
void mp_planner_time_accounting(void);

//**** planner buffer primitives
//void mp_init_planner_buffers(void);
//mpBuf_t * mp_get_w(int8_t q);
//mpBuf_t * mp_get_r(int8_t q);
mpBuf_t *mp_get_w(void);
mpBuf_t *mp_get_r(void);

//mpBuf_t * mp_get_prev_buffer(const mpBuf_t *bf);      // Use the following macro instead
//mpBuf_t * mp_get_next_buffer(const mpBuf_t *bf);      // Use the following macro instead
#define mp_get_prev_buffer(b) ((mpBuf_t *)(b->pv))
#define mp_get_next_buffer(b) ((mpBuf_t *)(b->nx))

mpBuf_t *mp_get_write_buffer(void);
void mp_commit_write_buffer(const blockType block_type);
mpBuf_t *mp_get_run_buffer(void);
bool mp_free_run_buffer(void);

//**** plan_line.c functions
void mp_zero_segment_velocity(void); // getters and setters...
float mp_get_runtime_velocity(void);
float mp_get_runtime_absolute_position(mpPlannerRuntime_t *_mr, uint8_t axis);
float mp_get_runtime_display_position(uint8_t axis);
void mp_set_runtime_display_offset(float offset[]);
bool mp_get_runtime_busy(void);
bool mp_runtime_is_idle(void);

stat_t mp_aline(GCodeState_t *_gm); // line planning...
void mp_plan_block_list(void);
void mp_plan_block_forward(mpBuf_t *bf);

//**** plan_zoid.c functions
stat_t mp_calculate_ramps(mpBlockRuntimeBuf_t *block, mpBuf_t *bf, const float entry_velocity);
float mp_get_target_length(const float v_0, const float v_1, const mpBuf_t *bf);
float mp_get_target_velocity(const float v_0, const float L, const mpBuf_t *bf); // acceleration ONLY
float mp_get_decel_velocity(const float v_0, const float L, const mpBuf_t *bf);  // deceleration ONLY
float mp_find_t(const float v_0, const float v_1, const float L, const float totalL, const float initial_t, const float T);

float mp_calc_v(const float t, const float v_0, const float v_1);                // compute the velocity along the curve accelerating from v_0 to v_1, at position t=[0,1]
float mp_calc_a(const float t, const float v_0, const float v_1, const float T); // compute acceleration over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
float mp_calc_j(const float t, const float v_0, const float v_1, const float T); // compute jerk over curve accelerating from v_0 to v_1, at position t=[0,1], total time T
//float mp_calc_l(const float t, const float v_0, const float v_1, const float T); // compute length over curve accelerating from v_0 to v_1, at position t=[0,1], total time T

//**** plan_exec.c functions
stat_t mp_forward_plan(void);
stat_t mp_exec_move(void);
stat_t mp_exec_aline(mpBuf_t *bf);
void mp_exit_hold_state(void);

void mp_dump_planner(mpBuf_t *bf_start);

#endif // End of include Guard: PLANNER_H_ONCE
