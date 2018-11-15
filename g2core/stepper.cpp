/*
 * stepper.cpp - 步进电机控制
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
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
/*  This module provides the low-level stepper drivers and some related functions.
 *  See stepper.h for a detailed explanation of this module.
 */

#include "g2core.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "planner.h"
#include "hardware.h"
#include "text_parser.h"
#include "util.h"
#include "controller.h"
#include "xio.h"

/**** Debugging output with semihosting ****/

#include "MotateDebug.h"

/* Note: stepper_debug statements removed 1/16/17 in SHA eb0905ccae03c04f99e6f471cbe029002f0324c6. 
 * See earlier commits to recover
 */

/**** Allocate structures ****/

stConfig_t st_cfg;
stPrepSingleton_t st_pre;
static stRunSingleton_t st_run;

/**** Static functions ****/

static void _load_move(void);

/**** Setup motate ****/

using namespace Motate;
extern OutputPin<kDebug1_PinNumber> debug_pin1;
extern OutputPin<kDebug2_PinNumber> debug_pin2;
extern OutputPin<kDebug3_PinNumber> debug_pin3;
//extern OutputPin<kDebug4_PinNumber> debug_pin4;

dda_timer_type dda_timer{kTimerUpToMatch, FREQUENCY_DDA}; // 步进脉冲生成
exec_timer_type exec_timer;                               // 触发下一个+ 1步进段的计算
fwd_plan_timer_type fwd_plan_timer;                       // 触发下一个块的计划

// SystickEvent用于处理停顿（必须在活动之前注册）
Motate::SysTickEvent dwell_systick_event{[&] {
                                             if (--st_run.dwell_ticks_downcount == 0)
                                             {
                                                 SysTickTimer.unregisterEvent(&dwell_systick_event);
                                                 _load_move(); // 在当前中断级别加载下一步移动
                                             }
                                         },
                                         nullptr};

/* Note on the above:
It's a lambda function creating a closure function. 
The full implementation that uses it is small and may help: 
https://github.com/synthetos/Motate/blob/41e5b92a98de4b268d1804bf6eadf3333298fc75/MotateProject/motate/Atmel_sam_common/SamTimers.h#L1147-L1218
It's just like a function, and is used as a function pointer.

But the closure part means that whatever variables that were in scope where the 
[&](parameters){code} is will be captured by the compiler as references in the generated 
function and used wherever the function gets called. In this particular use, there isn't 
anything that wouldn't be available anywhere in that file, but they're not being called 
from that file. They're being called by the systick interrupt which is over in SamTmers.cpp
So this saves a bunch of work exposing bits that the systick would need to call and encapsulates it.
And there's almost no runtime overhead. Just a check for a valid function pointer and then a call of it.
I'd like to get rid of that check but it's more work than its worth.

See here for some good info on lambda functions in C++
http://www.cprogramming.com/c++11/c++11-lambda-closures.html
http://en.cppreference.com/w/cpp/language/lambda
*/

/************************************************************************************
 **** CODE **************************************************************************
 ************************************************************************************/
/*
 * stepper_init() - 初始化步进电机子系统
 * stepper_reset() - 复位步进电机子系统
 *
 *备注：
 *  - 此init需要事先运行sys_init（）
 *  - 在config_init（）期间设置微步
 *  - 在config_init（）期间设置电机极性
 *  - 一旦完成所有内容，必须在main（）中启用高级中断
 */
/*  注意：这是Motate计时器调用的裸代码。
 *  NB: 要求：#include <component_tc.h>
 *
 *  REG_TC1_WPMR = 0x54494D00;              // 启用写入寄存器
 *  TC_Configure(TC_BLOCK_DDA, TC_CHANNEL_DDA, TC_CMR_DDA);
 *  REG_RC_DDA = TC_RC_DDA;                 //设定频率
 *  REG_IER_DDA = TC_IER_DDA;               //启用中断
 *  NVIC_EnableIRQ(TC_IRQn_DDA);
 *  pmc_enable_periph_clk(TC_ID_DDA);
 *  TC_Start(TC_BLOCK_DDA, TC_CHANNEL_DDA);
 */
void stepper_init()
{
    memset(&st_run, 0, sizeof(st_run)); // 清除所有值，指针和状态
    memset(&st_pre, 0, sizeof(st_pre)); // 清除所有值，指针和状态
    stepper_init_assertions();

    // setup DDA timer
    // Longer duty cycles stretch ON pulses but 75% is about the upper limit and about
    // optimal for 200 KHz DDA clock before the time in the OFF cycle is too short.
    // If you need more pulse width you need to drop the DDA clock rate
    dda_timer.setInterrupts(kInterruptOnOverflow | kInterruptPriorityHighest);

    // setup software interrupt exec timer & initial condition
    exec_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityHigh);
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;

    // setup software interrupt forward plan timer & initial condition
    fwd_plan_timer.setInterrupts(kInterruptOnSoftwareTrigger | kInterruptPriorityMedium);

    // 设置电机电流
    for (uint8_t motor = 0; motor < MOTORS; motor++)
    {
        Motors[motor]->setPowerLevel(st_cfg.mot[motor].power_level_scaled);
        st_run.mot[motor].power_level_dynamic = st_cfg.mot[motor].power_level_scaled;
    }
    board_stepper_init();
    stepper_reset(); // reset steppers to known state
}

/*
 * stepper_reset() - reset stepper internals
 *
 * 用于初始化步进器以及停止移动
 */

void stepper_reset()
{
    dda_timer.stop();               // stop all movement
    st_run.dda_ticks_downcount = 0; // signal the runtime is not busy
    st_run.dwell_ticks_downcount = 0;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC; // set to EXEC or it won't restart

    for (uint8_t motor = 0; motor < MOTORS; motor++)
    {
        st_pre.mot[motor].prev_direction = STEP_INITIAL_DIRECTION;
        st_pre.mot[motor].direction = STEP_INITIAL_DIRECTION;
        st_run.mot[motor].substep_accumulator = 0; // will become max negative during per-motor setup;
        st_pre.mot[motor].corrected_steps = 0;     // diagnostic only - no action effect
    }
    mp_set_steps_to_runtime_position(); // reset encoder to agree with the above
}

/*
 * stepper_init_assertions() - test assertions, return error code if violation exists
 * stepper_test_assertions() - test assertions, return error code if violation exists
 */

void stepper_init_assertions()
{
    st_run.magic_end = MAGICNUM;
    st_run.magic_start = MAGICNUM;
    st_pre.magic_end = MAGICNUM;
    st_pre.magic_start = MAGICNUM;
}

stat_t stepper_test_assertions()
{
    if ((BAD_MAGIC(st_run.magic_start)) || (BAD_MAGIC(st_run.magic_end)) ||
        (BAD_MAGIC(st_pre.magic_start)) || (BAD_MAGIC(st_pre.magic_end)))
    {
        return (cm_panic(STAT_STEPPER_ASSERTION_FAILURE, "stepper_test_assertions()"));
    }
    return (STAT_OK);
}

/*
 * st_runtime_isbusy() - return TRUE if runtime is busy:
 *
 *  Busy conditions:
 *  - motors are running
 *  - dwell is running
 */

bool st_runtime_isbusy()
{
    return (st_run.dda_ticks_downcount || st_run.dwell_ticks_downcount); // returns false if down count is zero
}

/*
 * st_clc() - clear counters
 */

stat_t st_clc(nvObj_t *nv) // clear diagnostic counters, reset stepper prep
{
    stepper_reset();
    return (STAT_OK);
}

/*
 * st_motor_power_callback() - 回调以管理电机电源排序
 *
 *  处理电机掉电定时，低功耗空闲和自适应电机功率
 */

stat_t st_motor_power_callback() // 由控制器调用
{
    if (!mp_is_phat_city_time())
    { // don't process this if you are time constrained in the planner
        return (STAT_NOOP);
    }

    bool have_actually_stopped = false;
    if ((!st_runtime_isbusy()) &&
        (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) &&
        (cm_get_machine_state() != MACHINE_CYCLE))
    { // if there are no moves to load...
        have_actually_stopped = true;
    }

    // 分别管理每台电机的电源
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++)
    {
        Motors[motor]->periodicCheck(have_actually_stopped);
    }
    return (STAT_OK);
}

/******************************
 * 中断服务例程 *
 ******************************/

/***** 步进中断服务常规 ************************************************
 * ISR -  DDA定时器中断程序 - 来自DDA定时器的服务定时器
 */

/*
 *  DDA定时器中断执行此操作:
 *    - 溢出时开火
 *    - 清除中断条件
 *    - 清除所有步进引脚 - 清除在前一个中断期间设置的引脚
 *    - 如果downcount == 0并停止计时器并退出
 *    - 为每个通道运行DDA
 *    - 递减计数 - 如果达到零，则加载下一个段
 *
 *  请注意，motor_N.step.isNull（）测试是编译时测试，而不是运行时测试。
 *  如果未定义motor_N，则{}子句（即该电机）从符合的代码中删除。
 */

namespace Motate
{ // 必须在Motate命名空间内定义计时器中断
template <>
void dda_timer_type::interrupt()
{
    dda_timer.getInterruptCause(); //清除中断条件

    // 清除上一次中断的所有步骤
    motor_1.stepEnd();
    motor_2.stepEnd();
#if MOTORS > 2
    motor_3.stepEnd();
#endif
#if MOTORS > 3
    motor_4.stepEnd();
#endif
#if MOTORS > 4
    motor_5.stepEnd();
#endif
#if MOTORS > 5
    motor_6.stepEnd();
#endif

    // 在段结束后处理最后一个DDA
    if (st_run.dda_ticks_downcount == 0)
    {
        dda_timer.stop(); // 把它关掉，否则它会继续走出最后一段
        return;
    }

    //  以下代码可以使用，但是在M3上循环展开它会更快。也许不是在M7上
    //    for (uint8_t motor=0; motor<MOTORS; motor++) {
    //        if  ((st_run.mot[motor].substep_accumulator += st_run.mot[motor].substep_increment) > 0) {
    //            Motors[motor]->stepStart();        // turn step bit on
    //            st_run.mot[motor].substep_accumulator -= st_run.dda_ticks_X_substeps;
    //            INCREMENT_ENCODER(motor);
    //        }
    //    }

    // process DDAs for each motor
    if ((st_run.mot[MOTOR_1].substep_accumulator += st_run.mot[MOTOR_1].substep_increment) > 0)
    {
        motor_1.stepStart(); // turn step bit on
        st_run.mot[MOTOR_1].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_1);
		//printf("x1,%d\n", st_run.dda_ticks_downcount);
	}
	else {
		//printf("x0,%d\n", st_run.dda_ticks_downcount);
	}
    if ((st_run.mot[MOTOR_2].substep_accumulator += st_run.mot[MOTOR_2].substep_increment) > 0)
    {
        motor_2.stepStart(); // turn step bit on
        st_run.mot[MOTOR_2].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_2);
    }
#if MOTORS > 2
    if ((st_run.mot[MOTOR_3].substep_accumulator += st_run.mot[MOTOR_3].substep_increment) > 0)
    {
        motor_3.stepStart(); // turn step bit on
        st_run.mot[MOTOR_3].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_3);
    }
#endif
#if MOTORS > 3
    if ((st_run.mot[MOTOR_4].substep_accumulator += st_run.mot[MOTOR_4].substep_increment) > 0)
    {
        motor_4.stepStart(); // turn step bit on
        st_run.mot[MOTOR_4].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_4);
    }
#endif
#if MOTORS > 4
    if ((st_run.mot[MOTOR_5].substep_accumulator += st_run.mot[MOTOR_5].substep_increment) > 0)
    {
        motor_5.stepStart(); // turn step bit on
        st_run.mot[MOTOR_5].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_5);
    }
#endif
#if MOTORS > 5
    if ((st_run.mot[MOTOR_6].substep_accumulator += st_run.mot[MOTOR_6].substep_increment) > 0)
    {
        motor_6.stepStart(); // turn step bit on
        st_run.mot[MOTOR_6].substep_accumulator -= st_run.dda_ticks_X_substeps;
        INCREMENT_ENCODER(MOTOR_6);
    }
#endif

    // 处理段的结束。
    //在此过程中设置的任何脉冲都会发生一次中断。
    if (--st_run.dda_ticks_downcount == 0)
    {
        _load_move(); // 在当前中断级别加载下一步移动
    }
} // MOTATE_TIMER_INTERRUPT
} // namespace Motate

/****************************************************************************************
 * Exec 测序代码   - 计算并准备下一个负载段
 * st_request_exec_move() - 请求执行移动的SW中断
 * exec_timer interrupt   - 用于调用exec函数的中断处理程序
 */

void st_request_exec_move()
{
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC)
    { //打扰中断
        exec_timer.setInterruptPending();
        return;
    }
}

namespace Motate
{ // 在Motate命名空间内定义计时器
template <>
void exec_timer_type::interrupt()
{
    exec_timer.getInterruptCause();                       // 清除中断条件
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_EXEC) // 正在加载临时缓冲区
    {
        if (mp_exec_move() != STAT_NOOP)
        {
            st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // 临时缓冲区已准备好加载
            st_request_load_move();
            return;
        }
    }
}
} // namespace Motate

/****************************************************************************************
 * st_request_forward_plan  - 对倒数第二个块执行前瞻规划
 * fwd_plan interrupt       - 用于调用前向规划功能的中断处理程序
 */

void st_request_forward_plan()
{
    fwd_plan_timer.setInterruptPending();
}

namespace Motate
{ // Define timer inside Motate namespace
template <>
void fwd_plan_timer_type::interrupt() //前向规划
{
    fwd_plan_timer.getInterruptCause(); // 清除中断条件
    if (mp_forward_plan() != STAT_NOOP)
    { // 我们现在转向执行。
        st_request_exec_move();
        return;
    }
}
} // namespace Motate

/****************************************************************************************
 * 装载机排序代码
 * st_request_load_move() - 触发软件中断（定时器）以请求加载移动
 * load_move interrupt    - 用于运行加载程序的中断处理程序
 *
 *  _load_move() 只能从与ISR相同或更高级别的ISR调用
 *  DDA或驻留ISR。已提供软件中断以允许非ISR
 *  请求加载（请参阅st_request_load_move（））
 */

void st_request_load_move()
{
    if (st_runtime_isbusy())
    { // 如果运行时忙，则不请求加载
        return;
    }
    if (st_pre.buffer_state == PREP_BUFFER_OWNED_BY_LOADER)
    { // 打扰中断
        _load_move();
    }
}

/****************************************************************************************
 * _load_move() - 将移动和加载到步进器运行时结构中
 *
 *此例程只能从ISR调用同一个或
 *作为DDA或驻留ISR的更高级别。软件中断已经发生
 *提供允许非ISR请求加载（st_request_load_move（））
 *
 *在aline（）代码中：
 *  - 所有轴必须设置步数并补偿超出范围的脉冲相位。
 *  - 如果轴有0步，则可以省略方向设置
 *  - 如果轴有0步，则必须根据功率模式设置电机功率
 */

static void _load_move()
{
    //请注意，dda_ticks_downcount必须等于零才能运行加载程序。
    //因此初始加载也必须将此设置为零作为初始化的一部分
    if (st_runtime_isbusy()) //st_run.dda_ticks_downcount || st_run.dwell_ticks_downcount
    {
        return; // exit if the runtime is busy
    }

    // 如果没有动作加载启动电机电源超时
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) //!=临时缓冲区已准备好加载
    {
        motor_1.motionStopped(); // ...启动电机功率超时
        motor_2.motionStopped();
#if (MOTORS > 2)
        motor_3.motionStopped();
#endif
#if (MOTORS > 3)
        motor_4.motionStopped();
#endif
#if (MOTORS > 4)
        motor_5.motionStopped();
#endif
#if (MOTORS > 5)
        motor_6.motionStopped();
#endif
        return;
    } // if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER)

    // 首先处理aline负载（最常见的情况）
    if (st_pre.block_type == BLOCK_TYPE_ALINE)
    {

        //**** 建立新的段 ****

        //debug_trap_if_true((st_run.dda_ticks_downcount != 0), "_load_move() 向下计数不为零");
        st_run.dda_ticks_downcount = st_pre.dda_ticks;
        st_run.dda_ticks_X_substeps = st_pre.dda_ticks_X_substeps;
		
        // INLINED VERSION: 4.3us
        //**** MOTOR_1 LOAD ****

        //这些部分在某种程度上针对执行速度进 整个加载操作
        //应该采用<5 uSec（Arm M3核心）。 如果你搞砸这个，要小心。

        // 以下if（）语句设置运行时子步增量值或将其归零
        if ((st_run.mot[MOTOR_1].substep_increment = st_pre.mot[MOTOR_1].substep_increment) != 0)
        {

            //注意：如果电机有0步，则全部跳过。 这确保了状态比较
            //始终在该电机实际运行的最后一段上运行，无论多少
            //段可能在两者之间处于非活动状态。

            // 如果自上一段以来时基已更改，则应用累加器校正
            if (st_pre.mot[MOTOR_1].accumulator_correction_flag == true)//信号累加器需要校正
            {
                st_pre.mot[MOTOR_1].accumulator_correction_flag = false;
                st_run.mot[MOTOR_1].substep_accumulator *= st_pre.mot[MOTOR_1].accumulator_correction;
            }

            //检测方向变化，如果是，
            //在硬件中设置方向位。
            //通过翻转子步骤累加器值关于其中点来补偿方向变化。

            if (st_pre.mot[MOTOR_1].direction != st_pre.mot[MOTOR_1].prev_direction)
            {
                st_pre.mot[MOTOR_1].prev_direction = st_pre.mot[MOTOR_1].direction;
                st_run.mot[MOTOR_1].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_1].substep_accumulator);
                motor_1.setDirection(st_pre.mot[MOTOR_1].direction);
            }

            // Enable the stepper and start/update motor power management
            motor_1.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_1, st_pre.mot[MOTOR_1].step_sign);
        }
        else
        { // 电机有0步; 可能需要激励电机进行电源模式处理
            motor_1.motionStopped();
        }
        // 累计计数步骤到步骤位置，并将当前正在加载的段的计数步骤归零
        ACCUMULATE_ENCODER(MOTOR_1);

#if (MOTORS >= 2)
        if ((st_run.mot[MOTOR_2].substep_increment = st_pre.mot[MOTOR_2].substep_increment) != 0)
        {
            if (st_pre.mot[MOTOR_2].accumulator_correction_flag == true)
            {
                st_pre.mot[MOTOR_2].accumulator_correction_flag = false;
                st_run.mot[MOTOR_2].substep_accumulator *= st_pre.mot[MOTOR_2].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_2].direction != st_pre.mot[MOTOR_2].prev_direction)
            {
                st_pre.mot[MOTOR_2].prev_direction = st_pre.mot[MOTOR_2].direction;
                st_run.mot[MOTOR_2].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_2].substep_accumulator);
                motor_2.setDirection(st_pre.mot[MOTOR_2].direction);
            }
            motor_2.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_2, st_pre.mot[MOTOR_2].step_sign);
        }
        else
        {
            motor_2.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_2);
#endif
#if (MOTORS >= 3)
        if ((st_run.mot[MOTOR_3].substep_increment = st_pre.mot[MOTOR_3].substep_increment) != 0)
        {
            if (st_pre.mot[MOTOR_3].accumulator_correction_flag == true)
            {
                st_pre.mot[MOTOR_3].accumulator_correction_flag = false;
                st_run.mot[MOTOR_3].substep_accumulator *= st_pre.mot[MOTOR_3].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_3].direction != st_pre.mot[MOTOR_3].prev_direction)
            {
                st_pre.mot[MOTOR_3].prev_direction = st_pre.mot[MOTOR_3].direction;
                st_run.mot[MOTOR_3].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_3].substep_accumulator);
                motor_3.setDirection(st_pre.mot[MOTOR_3].direction);
            }
            motor_3.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_3, st_pre.mot[MOTOR_3].step_sign);
        }
        else
        {
            motor_3.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_3);
#endif
#if (MOTORS >= 4)
        if ((st_run.mot[MOTOR_4].substep_increment = st_pre.mot[MOTOR_4].substep_increment) != 0)
        {
            if (st_pre.mot[MOTOR_4].accumulator_correction_flag == true)
            {
                st_pre.mot[MOTOR_4].accumulator_correction_flag = false;
                st_run.mot[MOTOR_4].substep_accumulator *= st_pre.mot[MOTOR_4].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_4].direction != st_pre.mot[MOTOR_4].prev_direction)
            {
                st_pre.mot[MOTOR_4].prev_direction = st_pre.mot[MOTOR_4].direction;
                st_run.mot[MOTOR_4].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_4].substep_accumulator);
                motor_4.setDirection(st_pre.mot[MOTOR_4].direction);
            }
            motor_4.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_4, st_pre.mot[MOTOR_4].step_sign);
        }
        else
        {
            motor_4.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_4);
#endif
#if (MOTORS >= 5)
        if ((st_run.mot[MOTOR_5].substep_increment = st_pre.mot[MOTOR_5].substep_increment) != 0)
        {
            if (st_pre.mot[MOTOR_5].accumulator_correction_flag == true)
            {
                st_pre.mot[MOTOR_5].accumulator_correction_flag = false;
                st_run.mot[MOTOR_5].substep_accumulator *= st_pre.mot[MOTOR_5].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_5].direction != st_pre.mot[MOTOR_5].prev_direction)
            {
                st_pre.mot[MOTOR_5].prev_direction = st_pre.mot[MOTOR_5].direction;
                st_run.mot[MOTOR_5].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_5].substep_accumulator);
                motor_5.setDirection(st_pre.mot[MOTOR_5].direction);
            }
            motor_5.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_5, st_pre.mot[MOTOR_5].step_sign);
        }
        else
        {
            motor_5.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_5);
#endif
#if (MOTORS >= 6)
        if ((st_run.mot[MOTOR_6].substep_increment = st_pre.mot[MOTOR_6].substep_increment) != 0)
        {
            if (st_pre.mot[MOTOR_6].accumulator_correction_flag == true)
            {
                st_pre.mot[MOTOR_6].accumulator_correction_flag = false;
                st_run.mot[MOTOR_6].substep_accumulator *= st_pre.mot[MOTOR_6].accumulator_correction;
            }
            if (st_pre.mot[MOTOR_6].direction != st_pre.mot[MOTOR_6].prev_direction)
            {
                st_pre.mot[MOTOR_6].prev_direction = st_pre.mot[MOTOR_6].direction;
                st_run.mot[MOTOR_6].substep_accumulator = -(st_run.dda_ticks_X_substeps + st_run.mot[MOTOR_6].substep_accumulator);
                motor_6.setDirection(st_pre.mot[MOTOR_6].direction);
            }
            motor_6.enable();
            SET_ENCODER_STEP_SIGN(MOTOR_6, st_pre.mot[MOTOR_6].step_sign);
        }
        else
        {
            motor_6.motionStopped();
        }
        ACCUMULATE_ENCODER(MOTOR_6);
#endif

        // ****最后这个****
		printf("downcount=%d,X_substeps=%d,accumulator=%d,increment=%d\n", 
			st_run.dda_ticks_downcount,
			st_run.dda_ticks_X_substeps, 
			st_run.mot[MOTOR_1].substep_accumulator, 
			st_run.mot[MOTOR_1].substep_increment);
        dda_timer.start(); //如果尚未运行，则启动DDA计时器

        // 处理暂停和命令
    }
    else if (st_pre.block_type == BLOCK_TYPE_DWELL)
    {
        st_run.dwell_ticks_downcount = st_pre.dwell_ticks;
        SysTickTimer.registerEvent(&dwell_systick_event); // We now use SysTick events to handle dwells

        // 处理同步命令
    }
    else if (st_pre.block_type == BLOCK_TYPE_COMMAND)
    {
        mp_runtime_command(st_pre.bf);

    } // else null - 在许多情况下这没关系

    // 所有其他情况下降到此处（例如，在M代码跳到此处后，Null移动）
    st_pre.block_type = BLOCK_TYPE_NULL;             //空着 - 做一个空操作
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC; // 正在加载临时缓冲区
    st_request_exec_move();                          // 执行并准备下一步行动
}

/***********************************************************************************
 * st_prep_line（） - 为装载程序准备下一步操作
 *
 *此函数对下一个脉冲段进行数学运算并准备好
 *装载机。它处理所有DDA优化和计时器设置，以便
 *加载可以尽快进行。它在联合空间中起作用
 *（电机）它按步骤工作，而不是长度单位。所有的args都是以
 *浮动并转换为适当的加载器整数类型。
 *
 * Args：
 *  -  travel_steps []是每个电机的步进相关运动。步骤是
 *浮点数通常具有小数值（小数步长）。标志
 *表示方向。不在移动中的电动机应该是0步输入。
 *
 *  -  following_error []是步数的测量误差向量。用于纠正。
 *
 *  -  segment_time  - 段应运行多少分钟。如果时机不是
 * 100％准确，这将影响移动速度，但不会影响行进距离。
 *
 *注意：许多表达式对于转换和执行顺序都很敏感，以避免长期
 *由于浮点舍入导致的精度误差。之前的失败尝试是：
 * dda_ticks_X_substeps =（int32_t）（（微秒/ 1000000）* f_dda * dda_substeps）;
 */

stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time)
{
    // trap assertion failures and other conditions that would prevent queuing the line
    if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_EXEC)
    { // never supposed to happen
        return (cm_panic(STAT_INTERNAL_ERROR, "st_prep_line() prep sync error"));
    }
    else if (isinf(segment_time))
    { // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_INFINITE, "st_prep_line()"));
    }
    else if (isnan(segment_time))
    { // never supposed to happen
        return (cm_panic(STAT_PREP_LINE_MOVE_TIME_IS_NAN, "st_prep_line()"));
    }
    // setup segment parameters
    // - dda_ticks is the integer number of DDA clock ticks needed to play out the segment
    // - ticks_X_substeps is the maximum depth of the DDA accumulator (as a negative number)

    st_pre.dda_ticks = (int32_t)(segment_time * 60 * FREQUENCY_DDA); // NB: converts minutes to seconds
    st_pre.dda_ticks_X_substeps = st_pre.dda_ticks * DDA_SUBSTEPS;

    // setup motor parameters

    float correction_steps;
    for (uint8_t motor = 0; motor < MOTORS; motor++)
    { // remind us that this is motors, not axes

        // Skip this motor if there are no new steps. Leave all other values intact.
        if (fp_ZERO(travel_steps[motor]))
        {
            st_pre.mot[motor].substep_increment = 0; // substep increment also acts as a motor flag
            continue;
        }

        // Setup the direction, compensating for polarity.
        // Set the step_sign which is used by the stepper ISR to accumulate step position

        if (travel_steps[motor] >= 0)
        { // positive direction
            st_pre.mot[motor].direction = DIRECTION_CW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = 1;
        }
        else
        {
            st_pre.mot[motor].direction = DIRECTION_CCW ^ st_cfg.mot[motor].polarity;
            st_pre.mot[motor].step_sign = -1;
        }

        // Detect segment time changes and setup the accumulator correction factor and flag.
        // Putting this here computes the correct factor even if the motor was dormant for some number
        // of previous moves. Correction is computed based on the last segment time actually used.

        if (fabs(segment_time - st_pre.mot[motor].prev_segment_time) > 0.0000001)
        { // highly tuned FP != compare
            if (fp_NOT_ZERO(st_pre.mot[motor].prev_segment_time))
            { // special case to skip first move
                st_pre.mot[motor].accumulator_correction_flag = true;
                st_pre.mot[motor].accumulator_correction = segment_time / st_pre.mot[motor].prev_segment_time;
            }
            st_pre.mot[motor].prev_segment_time = segment_time;
        }

        // 'Nudge' correction strategy. Inject a single, scaled correction value then hold off
        // NOTE: This clause can be commented out to test for numerical accuracy and accumulating errors

        if ((--st_pre.mot[motor].correction_holdoff < 0) &&
            (fabs(following_error[motor]) > STEP_CORRECTION_THRESHOLD))
        {

            st_pre.mot[motor].correction_holdoff = STEP_CORRECTION_HOLDOFF;
            correction_steps = following_error[motor] * STEP_CORRECTION_FACTOR;

            if (correction_steps > 0)
            {
                correction_steps = min3(correction_steps, fabs(travel_steps[motor]), STEP_CORRECTION_MAX);
            }
            else
            {
                correction_steps = max3(correction_steps, -fabs(travel_steps[motor]), -STEP_CORRECTION_MAX);
            }
            st_pre.mot[motor].corrected_steps += correction_steps;
            travel_steps[motor] -= correction_steps;
        }

        // Compute substeb increment. The accumulator must be *exactly* the incoming
        // fractional steps times the substep multiplier or positional drift will occur.
        // Rounding is performed to eliminate a negative bias in the uint32 conversion
        // that results in long-term negative drift. (fabs/round order doesn't matter)

        st_pre.mot[motor].substep_increment = round(fabs(travel_steps[motor] * DDA_SUBSTEPS));
    }
    st_pre.block_type = BLOCK_TYPE_ALINE;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // signal that prep buffer is ready
    return (STAT_OK);
}

/*
 * st_prep_null() - 保持装载机的快乐。 否则不执行任何操作
 */

void st_prep_null()
{
    st_pre.block_type = BLOCK_TYPE_NULL;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC; // 正在加载临时缓冲区
}

/*
 * st_prep_command() - Stage command to execution
 */

void st_prep_command(void *bf)
{
    st_pre.block_type = BLOCK_TYPE_COMMAND;
    st_pre.bf = (mpBuf_t *)bf;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // signal that prep buffer is ready
}

/*
 * st_prep_dwell()      - Add a dwell to the move buffer
 */

void st_prep_dwell(float microseconds)
{
    st_pre.block_type = BLOCK_TYPE_DWELL;
    // we need dwell_ticks to be at least 1
    st_pre.dwell_ticks = std::max((uint32_t)((microseconds / 1000000) * FREQUENCY_DWELL), 1u); //xzw168
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;                                         // signal that prep buffer is ready
}

/*
 * st_prep_out_of_band_dwell()
 *
 * Add a dwell to the loader without going through the planner buffers.
 * Only usable while exec isn't running, e.g. in feedhold or stopped states.
 * Otherwise it is skipped.
 */

void st_prep_out_of_band_dwell(float microseconds)
{
    if (!st_runtime_isbusy())
    {
        st_prep_dwell(microseconds);
        st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; // signal that prep buffer is ready
        st_request_load_move();
    }
}

/*
 * _set_hw_microsteps() - set microsteps in hardware
 */

static void _set_hw_microsteps(const uint8_t motor, const uint8_t microsteps)
{
    if (motor >= MOTORS)
    {
        return;
    }

    Motors[motor]->setMicrosteps(microsteps);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/* HELPERS
 * _motor() - motor number as an index or -1 if na
 */

static int8_t _motor(const index_t index)
{
    char c = cfgArray[index].token[0];
    return (isdigit(c) ? c - 0x31 : -1); // 0x30 + 1 offsets motor 1 to == 0
}

/*
 * _set_motor_steps_per_unit() - what it says
 * This function will need to be rethought if microstep morphing is implemented
 */

static float _set_motor_steps_per_unit(nvObj_t *nv)
{
    uint8_t m = _motor(nv->index);
    st_cfg.mot[m].units_per_step = (st_cfg.mot[m].travel_rev * st_cfg.mot[m].step_angle) /
                                   (360 * st_cfg.mot[m].microsteps);

    st_cfg.mot[m].steps_per_unit = 1 / st_cfg.mot[m].units_per_step;
    return (st_cfg.mot[m].steps_per_unit);
}

/* PER-MOTOR FUNCTIONS
 *
 * st_get_ma() - get motor axis mapping
 * st_set_ma() - set motor axis mapping
 * st_get_sa() - get motor step angle
 * st_set_sa() - set motor step angle
 * st_get_tr() - get travel per motor revolution
 * st_set_tr() - set travel per motor revolution
 * st_get_mi() - get motor microsteps
 * st_set_mi() - set motor microsteps
 * 
 * st_set_pm() - set motor power mode
 * st_get_pm() - get motor power mode
 * st_set_pl() - set motor power level
 */

/*
 * st_get_ma() - get motor axis mapping
 *
 *  Legacy axis numbers are     XYZABC    for axis 0-5
 *  External axis numbers are   XYZABCUVW for axis 0-8
 *  Internal axis numbers are   XYZUVWABC for axis 0-8 (for various code reasons)
 *
 *  This function retrieves an internal axis number and remaps it to an external axis number
 */
stat_t st_get_ma(nvObj_t *nv)
{
    uint8_t remap_axis[9] = {0, 1, 2, 6, 7, 8, 3, 4, 5};
    ritorno(get_integer(nv, st_cfg.mot[_motor(nv->index)].motor_map));
    nv->value_int = remap_axis[nv->value_int];
    return (STAT_OK);
}

/*
 * st_set_ma() - set motor axis mapping
 *
 *  Legacy axis numbers are     XYZABC    for axis 0-5
 *  External axis numbers are   XYZABCUVW for axis 0-8
 *  Internal axis numbers are   XYZUVWABC for axis 0-8 (for various code reasons)
 *
 *  This function accepts an external axis number and remaps it to an external axis number,
 *  writes the internal axis number and returns the external number in the JSON response.
 */
stat_t st_set_ma(nvObj_t *nv)
{
    if (nv->value_int < 0)
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > AXES)
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    uint8_t external_axis = nv->value_int;
    uint8_t remap_axis[9] = {0, 1, 2, 6, 7, 8, 3, 4, 5};
    nv->value_int = remap_axis[nv->value_int];
    ritorno(set_integer(nv, st_cfg.mot[_motor(nv->index)].motor_map, 0, AXES));
    nv->value_int = external_axis;
    return (STAT_OK);
}

// step angle
stat_t st_get_sa(nvObj_t *nv) { return (get_float(nv, st_cfg.mot[_motor(nv->index)].step_angle)); }
stat_t st_set_sa(nvObj_t *nv)
{
    ritorno(set_float_range(nv, st_cfg.mot[_motor(nv->index)].step_angle, 0.001, 360));
    _set_motor_steps_per_unit(nv);
    return (STAT_OK);
}

// travel per revolution
stat_t st_get_tr(nvObj_t *nv) { return (get_float(nv, st_cfg.mot[_motor(nv->index)].travel_rev)); }
stat_t st_set_tr(nvObj_t *nv)
{
    ritorno(set_float_range(nv, st_cfg.mot[_motor(nv->index)].travel_rev, 0.0001, 1000000));
    _set_motor_steps_per_unit(nv);
    return (STAT_OK);
}

// microsteps
stat_t st_get_mi(nvObj_t *nv) { return (get_integer(nv, st_cfg.mot[_motor(nv->index)].microsteps)); }
stat_t st_set_mi(nvObj_t *nv)
{
    if (nv->value_int <= 0)
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }

    uint8_t mi = (uint8_t)nv->value_int;
    if ((mi != 1) && (mi != 2) && (mi != 4) && (mi != 8) && (mi != 16) && (mi != 32))
    {
        nv_add_conditional_message((const char *)"*** WARNING *** Setting non-standard microstep value");
    }
    // set it anyway, even if it's unsupported
    ritorno(set_integer(nv, st_cfg.mot[_motor(nv->index)].microsteps, 1, 255));
    _set_motor_steps_per_unit(nv);
    _set_hw_microsteps(_motor(nv->index), nv->value_int);
    return (STAT_OK);
}

// motor steps per unit (direct)
stat_t st_get_su(nvObj_t *nv)
{
    return (get_float(nv, st_cfg.mot[_motor(nv->index)].steps_per_unit));
}

stat_t st_set_su(nvObj_t *nv)
{
    // Don't set a zero or negative value - just calculate based on sa, tr, and mi
    // This way, if STEPS_PER_UNIT is set to 0 it is unused and we get the computed value
    if (nv->value_flt <= 0)
    {
        nv->value_flt = _set_motor_steps_per_unit(nv);
        return (STAT_OK);
    }

    // Do unit conversion here because it's a reciprocal value (rather than process_incoming_float())
    if (cm_get_units_mode(MODEL) == INCHES)
    {
        if (cm_get_axis_type(nv) == AXIS_TYPE_LINEAR)
        {
            nv->value_flt *= INCHES_PER_MM;
        }
    }
    uint8_t m = _motor(nv->index);
    st_cfg.mot[m].steps_per_unit = nv->value_flt;
    st_cfg.mot[m].units_per_step = 1.0 / st_cfg.mot[m].steps_per_unit;

    // Scale TR so all the other values make sense
    // You could scale any one of the other values, but TR makes the most sense
    st_cfg.mot[m].travel_rev = (360.0 * st_cfg.mot[m].microsteps) /
                               (st_cfg.mot[m].steps_per_unit * st_cfg.mot[m].step_angle);
    return (STAT_OK);
}

// polarity
stat_t st_get_po(nvObj_t *nv) { return (get_integer(nv, st_cfg.mot[_motor(nv->index)].polarity)); }
stat_t st_set_po(nvObj_t *nv) { return (set_integer(nv, st_cfg.mot[_motor(nv->index)].polarity, 0, 1)); }

// power management mode
stat_t st_get_pm(nvObj_t *nv)
{
    nv->value_int = (float)Motors[_motor(nv->index)]->getPowerMode();
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t st_set_pm(nvObj_t *nv)
{
    // Test the value without setting it, then setPowerMode() now
    // to both set and take effect immediately.
    ritorno(set_integer(nv, (uint8_t &)cs.null, 0, MOTOR_POWER_MODE_MAX_VALUE));
    Motors[_motor(nv->index)]->setPowerMode((stPowerMode)nv->value_int);
    return (STAT_OK);
}

/*
 * st_get_pl() - get motor power level
 * st_set_pl() - set motor power level
 *
 *  Input value may vary from 0.000 to 1.000 The setting is scaled to allowable PWM range.
 *  This function sets both the scaled and dynamic power levels, and applies the
 *  scaled value to the vref.
 */
stat_t st_get_pl(nvObj_t *nv) { return (get_float(nv, st_cfg.mot[_motor(nv->index)].power_level)); }
stat_t st_set_pl(nvObj_t *nv)
{
    uint8_t m = _motor(nv->index);
    ritorno(set_float_range(nv, st_cfg.mot[m].power_level, 0.0, 1.0));
    st_cfg.mot[m].power_level_scaled = (nv->value_flt * POWER_LEVEL_SCALE_FACTOR);
    st_run.mot[m].power_level_dynamic = (st_cfg.mot[m].power_level_scaled);
    Motors[m]->setPowerLevel(st_cfg.mot[m].power_level_scaled);
    return (STAT_OK);
}

/*
 * st_get_pwr()	- get current motor power
 *
 *  Returns the current power level of the motor given it's enable/disable state
 *  Returns 0.0 if motor is de-energized or disabled
 *  Can be extended to report idle setback by changing getCurrentPowerLevel()
 */
stat_t st_get_pwr(nvObj_t *nv)
{
    // this is kind of a hack to extract the motor number from the table
    uint8_t motor = (cfgArray[nv->index].token[3] & 0x0F) - 1;
    if (motor > MOTORS)
    {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    };

    nv->value_flt = Motors[motor]->getCurrentPowerLevel(motor);
    nv->valuetype = TYPE_FLOAT;
    nv->precision = cfgArray[nv->index].precision;
    return (STAT_OK);
}

stat_t st_set_ep(nvObj_t *nv) // set motor enable polarity
{
    if (nv->value_int < IO_ACTIVE_LOW)
    {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > IO_ACTIVE_HIGH)
    {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }

    uint8_t motor = _motor(nv->index);
    if (motor > MOTORS)
    {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    };

    Motors[motor]->setEnablePolarity((ioMode)nv->value_int);
    return (STAT_OK);
}

stat_t st_get_ep(nvObj_t *nv) // get motor enable polarity
{
    if (nv->value_int < IO_ACTIVE_LOW)
    {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > IO_ACTIVE_HIGH)
    {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }

    uint8_t motor = _motor(nv->index);
    if (motor > MOTORS)
    {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    };

    nv->value_int = (float)Motors[motor]->getEnablePolarity();
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

stat_t st_set_sp(nvObj_t *nv) // set motor step polarity
{
    if (nv->value_int < IO_ACTIVE_LOW)
    {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > IO_ACTIVE_HIGH)
    {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }

    uint8_t motor = _motor(nv->index);
    if (motor > MOTORS)
    {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    };

    Motors[motor]->setStepPolarity((ioMode)nv->value_int);
    return (STAT_OK);
}

stat_t st_get_sp(nvObj_t *nv) // get motor step polarity
{
    if (nv->value_int < IO_ACTIVE_LOW)
    {
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > IO_ACTIVE_HIGH)
    {
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }

    uint8_t motor = _motor(nv->index);
    if (motor > MOTORS)
    {
        return STAT_INPUT_VALUE_RANGE_ERROR;
    };

    nv->value_int = (float)Motors[motor]->getStepPolarity();
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/* GLOBAL FUNCTIONS (SYSTEM LEVEL)
 *
 * st_get_mt() - get motor timeout in seconds
 * st_set_mt() - set motor timeout in seconds
 * st_set_md() - disable motor power
 * st_set_me() - enable motor power
 * st_set_md() - disable motor power
 * st_get_dw() - get remaining dwell time
 *
 * Calling me or md with NULL will enable or disable all motors
 * Setting a value of 0 will enable or disable all motors
 * Setting a value from 1 to MOTORS will enable or disable that motor only
 */

stat_t st_get_mt(nvObj_t *nv) { return (get_float(nv, st_cfg.motor_power_timeout)); }
stat_t st_set_mt(nvObj_t *nv) { return (set_float_range(nv, st_cfg.motor_power_timeout,
                                                        MOTOR_TIMEOUT_SECONDS_MIN,
                                                        MOTOR_TIMEOUT_SECONDS_MAX)); }

// Make sure this function is not part of initialization --> f00
// nv->value is seconds of timeout
stat_t st_set_me(nvObj_t *nv)
{
    for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++)
    {
        Motors[motor]->enable(nv->value_int); // nv->value is the timeout or 0 for default
    }
    return (STAT_OK);
}

// Make sure this function is not part of initialization --> f00
// nv-value is motor to disable, or 0 for all motors
stat_t st_set_md(nvObj_t *nv)
{
    if (nv->value_int < 0)
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_LESS_THAN_MIN_VALUE);
    }
    if (nv->value_int > MOTORS)
    {
        nv->valuetype = TYPE_NULL;
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
    }
    // de-energize all motors
    if ((uint8_t)nv->value_int == 0)
    { // 0 means all motors
        for (uint8_t motor = MOTOR_1; motor < MOTORS; motor++)
        {
            Motors[motor]->disable();
        }
    }
    else
    { // otherwise it's just one motor
        Motors[(uint8_t)nv->value_int - 1]->disable();
    }
    return (STAT_OK);
}

stat_t st_get_dw(nvObj_t *nv)
{
    nv->value_int = st_run.dwell_ticks_downcount;
    nv->valuetype = TYPE_INTEGER;
    return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char msg_units0[] = " in"; // used by generic print functions
static const char msg_units1[] = " mm";
static const char msg_units2[] = " deg";
static const char *const msg_units[] = {msg_units0, msg_units1, msg_units2};
#define DEGREE_INDEX 2

static const char fmt_me[] = "motors energized\n";
static const char fmt_md[] = "motors de-energized\n";
static const char fmt_mt[] = "[mt]  motor idle timeout%14.2f seconds\n";
static const char fmt_0ma[] = "[%s%s] m%s map to axis%15d [0=X,1=Y,2=Z...]\n";
static const char fmt_0sa[] = "[%s%s] m%s step angle%20.3f%s\n";
static const char fmt_0tr[] = "[%s%s] m%s travel per revolution%10.4f%s\n";
static const char fmt_0mi[] = "[%s%s] m%s microsteps%16d [1,2,4,8,16,32]\n";
static const char fmt_0su[] = "[%s%s] m%s steps per unit %17.5f steps per%s\n";
static const char fmt_0po[] = "[%s%s] m%s polarity%18d [0=normal,1=reverse]\n";
static const char fmt_0ep[] = "[%s%s] m%s enable polarity%11d [0=active HIGH,1=active LOW]\n";
static const char fmt_0sp[] = "[%s%s] m%s step polarity%13d [0=active HIGH,1=active LOW]\n";
static const char fmt_0pm[] = "[%s%s] m%s power management%10d [0=disabled,1=always on,2=in cycle,3=when moving]\n";
static const char fmt_0pl[] = "[%s%s] m%s motor power level%13.3f [0.000=minimum, 1.000=maximum]\n";
static const char fmt_pwr[] = "[%s%s] Motor %c power level:%12.3f\n";

void st_print_me(nvObj_t *nv) { text_print(nv, fmt_me); } // TYPE_NULL - message only
void st_print_md(nvObj_t *nv) { text_print(nv, fmt_md); } // TYPE_NULL - message only
void st_print_mt(nvObj_t *nv) { text_print(nv, fmt_mt); } // TYPE_FLOAT

static void _print_motor_int(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, (int)nv->value_int);
    xio_writeline(cs.out_buf);
}

static void _print_motor_flt(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_flt);
    xio_writeline(cs.out_buf);
}

static void _print_motor_flt_units(nvObj_t *nv, const char *format, uint8_t units)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->group, nv->value_flt, GET_TEXT_ITEM(msg_units, units));
    xio_writeline(cs.out_buf);
}

static void _print_motor_pwr(nvObj_t *nv, const char *format)
{
    sprintf(cs.out_buf, format, nv->group, nv->token, nv->token[0], nv->value_flt);
    xio_writeline(cs.out_buf);
}

void st_print_ma(nvObj_t *nv) { _print_motor_int(nv, fmt_0ma); }
void st_print_sa(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0sa, DEGREE_INDEX); }
void st_print_tr(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0tr, cm_get_units_mode(MODEL)); }
void st_print_mi(nvObj_t *nv) { _print_motor_int(nv, fmt_0mi); }
void st_print_su(nvObj_t *nv) { _print_motor_flt_units(nv, fmt_0su, cm_get_units_mode(MODEL)); }
void st_print_po(nvObj_t *nv) { _print_motor_int(nv, fmt_0po); }
void st_print_ep(nvObj_t *nv) { _print_motor_int(nv, fmt_0ep); }
void st_print_sp(nvObj_t *nv) { _print_motor_int(nv, fmt_0sp); }
void st_print_pm(nvObj_t *nv) { _print_motor_int(nv, fmt_0pm); }
void st_print_pl(nvObj_t *nv) { _print_motor_flt(nv, fmt_0pl); }
void st_print_pwr(nvObj_t *nv) { _print_motor_pwr(nv, fmt_pwr); }

#endif // __TEXT_MODE
