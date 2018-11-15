/*
 * plan_exec.cpp - execution function for acceleration managed lines
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2018 Rob Giseburt
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

#include "g2core.h"
#include "config.h"
#include "controller.h"
#include "planner.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "util.h"
#include "spindle.h"
#include "xio.h" // DIAGNOSTIC

// 执行例程（注意：这些都是从LO中断调用的）
static stat_t _exec_aline_head(mpBuf_t *bf); //传递bf因为body可能需要它，它可能会调用body
static stat_t _exec_aline_body(mpBuf_t *bf); //传递bf，以便在出口速度上升时身体可以自我伸展。
static stat_t _exec_aline_tail(mpBuf_t *bf);
static stat_t _exec_aline_segment(void);
static void _exec_aline_normalize_block(mpBlockRuntimeBuf_t *b);
static stat_t _exec_aline_feedhold(mpBuf_t *bf);

static void _init_forward_diffs(float v_0, float v_1);

/****************************************************************************************
* mp_forward_plan（） - 计划命令并在exec之前移动; 呼吁缓慢移动
*
**** 警告 ****
* mp_forward_plan（）不应该直接调用！
*而是调用st_request_forward_plan（），它调解访问。
*
* mp_forward_plan（）在此之前执行即时前向计划
*移动和命令排队到移动执行运行时（exec）。
*与后台规划不同，缓冲区只是前瞻性规划一次。
*
* mp_forward_plan（）通过st_request_forward_plan（）积极调用。
*它具有相对较低的中断级别来调用它自己的。
*另请参阅：planner.h中的计划程序背景和概述说明
*
*它检查当前运行的缓冲区及其相邻的缓冲区：
*  - 停止系统重新规划或计划未准备的东西
*  - 通过COMMAND块计划下一个可用的ALINE（移动）块
*  - 跳过/或预先计划COMMAND块，同时将它们标记为FULLY_PLANNED
*
*返回：
*  -  STAT_OK如果应该调用exec来kickstart（或继续）移动
*  -  STAT_NOOP退出而不执行任何操作（不要调用exec）
* 
*
* ---前瞻性规划处理和案例---
*
*这些情况描述了计划程序队列中所有可能的缓冲区序列
*使用当前正在执行（或即将执行）的运行缓冲区，展望未来
*到最近到达的缓冲区。在大多数情况下，只需要一个或两个缓冲区
*检查，但可能需要处理连续的命令组。
*
*'Running'情况是运行缓冲区状态为RUNNING的位置。Bootstrap处理所有其他情况。
*'Bootstrap'在启动阶段发生，在开始移动之前收集移动。
*基于此定义不可能的条件未在下表中列出。
*
*有关描述中使用的简写，请参阅planner.h / bufferState枚举。
*如速记中所述，所有案例都假设混合了移动和命令。
*所有情况假设2'块' - 运行块（r）和计划块（p）。这些案件
*如果将来使用更多的块，将需要重新审视和推广
*（即更深入的前瞻性规划）。
*
*'NOT_PLANNED'表示该块未被重新规划或正向计划
*这指的是BACK_PLANNED以下的任何状态，即<MP_BUFFER_BACK_PLANNED
*'NOT_PLANNED'既可以是移动也可以是命令，我们不关心，因此未指定。
*
*'BACK_PLANNED'表示该块已经重新规划，但未进行前瞻性规划
*'FULLY_PLANNED'表示该块已重新规划并已进行前瞻计划，已准备好执行
*'RUNNING'表示移动正在运行时执行。在此阶段，bf被“锁定”
*
*'COMMAND'或'COMMAND（s）'是指一个命令或一组连续的命令缓冲区
*可能处于BACK_PLANNED或FULLY_PLANNED状态。处理总是一样的;
*计划所有BACK_PLANNED命令并跳过所有FULLY_PLANNED命令。
*
*注1：对于MOVE，使用Run块的退出速度（mr-> r-> exit_velocity）
*作为下一个相邻移动的进入速度。
*
*注意1a：在这个特殊的COMMAND案例中，我们信任mr-> r-> exit_velocity，因为
* back planner已经为我们处理过这个案子。
*
*注2：对于COMMAND，使用当前运行时的输入速度（mr-> entry_velocity）
*作为下一个相邻移动的进入速度。mr-> entry_velocity几乎总是0，
*但在比赛条件下可能不是0。
* FYI：mr-> entry_velocity设置在mp_exec_aline（）中最后一个运行块的末尾。
 *
 *  CASE:
 *   0. Nothing to do
 *
 *         run_buffer
 *         ----------
 *       a. <no buffer>          Run buffer has not yet been initialized (prep null buffer and return NOOP)
 *       b. NOT_BACK_PLANNED     No moves or commands in run buffer. Exit with no action
 *
 *   1.引导案例（缓冲状态<RUNNING）
 *
 *         run_buffer               next N bufs         terminal buf        Actions
 *         ----------               -----------         ------------        ----------------------------------
 *       a. BACK_PLANNED/MOVE       <don't care>        <don't care>        计划移动，退出确定
 *       b. FULLY_PLANNED/MOVE      NOT_PLANNED         <don't care>        退出NOOP
 *       c. FULLY_PLANNED/MOVE      BACK_PLANNED/MOVE   <don't care>        退出NOOP（不要计划通过PLANNED缓冲区）
 *       d. FULLY_PLANNED/MOVE      FULLY_PLANNED/MOVE  <don't care>        陷阱非法状态，退出NOOP
 *       e. FULLY_PLANNED/MOVE      COMMAND(s)          <don't care>        退出NOOP
 *       f. BACK_PLANNED/COMMAND    NOT_PREPPED         <don't care>        计划命令，退出确定
 *       g. BACK_PLANNED/COMMAND    BACK_PLANNED/MOVE   <don't care>        计划命令，计划移动（注2），退出OK
 *       h. BACK_PLANNED/COMMAND    FULLY_PLANNED/MOVE  <don't care>        陷阱非法状态，退出NOOP
 *       i. BACK_PLANNED/COMMAND    NOT_PLANNED         <don't care>        跳过命令，退出确定
 *       j. BACK_PLANNED/COMMAND    BACK_PLANNED/MOVE   <don't care>        跳过命令，计划移动（注2），退出OK
 *       k. BACK_PLANNED/COMMAND    FULLY_PLANNED/MOVE  <don't care>        退出NOOP
 *
 *   2. 运行案例（缓冲状态==运行）
 *
 *          run_buffer              next N bufs         terminal buf        Actions
 *          ----------              -----------         ------------        ----------------------------------
 *       a. RUNNING/MOVE            BACK_PLANNED/MOVE   <don't care>        plan move, exit OK
 *       b. RUNNING/MOVE            FULLY_PLANNED/MOVE  <don't care>        退出NOOP
 *       c. RUNNING/MOVE            COMMAND(s)          NOT_PLANNED         skip/plan command(s), exit OK
 *       d. RUNNING/MOVE            COMMAND(s)          BACK_PLANNED/MOVE   skip/plan command(s), plan move, exit OK
 *       e. RUNNING/MOVE            BACK_PLANNED(s)     FULLY_PLANNED-MOVE  退出NOOP
 *       f. RUNNING/COMMAND         BACK_PLANNED/MOVE   <don't care>        plan move, exit OK
 *       g. RUNNING/COMMAND         FULLY_PLANNED/MOVE  <don't care>        退出NOOP
 *       h. RUNNING/COMMAND         COMMAND(s)          NOT_PLANNED         skip/plan command(s), exit OK
 *       i. RUNNING/COMMAND         COMMAND(s)          BACK_PLANNED/MOVE   skip/plan command(s), plan move (Note 1a), exit OK
 *       j. RUNNING/COMMAND         COMMAND(s)          FULLY_PLANNED/MOVE  skip command(s), exit NOOP
 *
 *       (注意：2j中的所有COMMAND应处于PLANNED状态)
 *
 *
 * _plan_aline() - mp_forward_plan() helper
 *
 *计算当前计划块和下一个PREPPED缓冲区的斜坡
 * PREPPED缓冲区将在稍后设置为PLANNED ...
 *
 *传入将与计划块“链接”的bf缓冲区
 * exec_aline（）隐式链接块和缓冲区
 *
 *请注意，一次只能进行一次PLANNED移动。
 *这是为了帮助同步mr-> p指向下一个计划的mr-> bf
 * mr-> p仅在mp.r = mr-> p之后在mp_exec_aline（）中前进。
 *此代码对齐exec_aline（）的缓冲区和块。
 */
static stat_t _plan_aline(mpBuf_t *bf, float entry_velocity)
{
    mpBlockRuntimeBuf_t *block = mr->p;            // 设置一个本地计划块，这样指针就不会改变
    mp_calculate_ramps(block, bf, entry_velocity); // 计算梯形状斜坡参数用于块

    debug_trap_if_true((block->exit_velocity > block->cruise_velocity),
                       "_plan_line() exit velocity > cruise velocity after calculate_ramps()");

    debug_trap_if_true((block->head_length < 0.00001 && block->body_length < 0.00001 && block->tail_length < 0.00001),
                       "_plan_line() zero or negative length block after calculate_ramps()");

    bf->buffer_state = MP_BUFFER_FULLY_PLANNED; //...here
    bf->plannable = false;
    return (STAT_OK); // report that we planned something...
}

stat_t mp_forward_plan()
{
    mpBuf_t *bf = mp_get_run_buffer();
    float entry_velocity;

    // 案例0：检查提前退出条件当前正在运行的缓冲区
    if (bf == NULL)
    { // case 0a：NULL表示没有运行 - 这没关系
        st_prep_null();//st_pre.block_type = BLOCK_TYPE_NULL;st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC; // 正在加载临时缓冲区
        return (STAT_NOOP);
    }
    if (bf->buffer_state < MP_BUFFER_BACK_PLANNED)//<计划正在进行中 - 至少已设定vmax
    { // 案例0b：无事可做。 你出去。
        return (STAT_NOOP);
    }

    // Case 2: Running cases - move bf past run buffer so it acts like case 1
    if (bf->buffer_state == MP_BUFFER_RUNNING)//计划正在进行中 - 至少已设定vmax
    {
        bf = bf->nx;
        entry_velocity = mr->r->exit_velocity; // set Note 1 entry_velocity (move cases)
    }
    else
    {
        entry_velocity = mr->entry_velocity; // set Note 2 entry velocity (command cases)
    }

    // bf points to a command block; start cases 1f, 1g, 1h, 1i, 1j, 1k, 2c, 2d, 2e, 2h, 2i, 2j
    bool planned_something = false;

    if (bf->block_type != BLOCK_TYPE_ALINE)
    { // meaning it's a COMMAND
        while (bf->block_type >= BLOCK_TYPE_COMMAND)
        {
            if (bf->buffer_state == MP_BUFFER_BACK_PLANNED)
            {
                bf->buffer_state = MP_BUFFER_FULLY_PLANNED; // "planning" is just setting the state (for now)
                planned_something = true;
            }
            bf = bf->nx;
        }
        // Note: bf now points to the first non-command buffer past the command(s)
        if ((bf->block_type == BLOCK_TYPE_ALINE) && (bf->buffer_state > MP_BUFFER_BACK_PLANNED))
        {                                          // case 1i
            entry_velocity = mr->r->exit_velocity; // set entry_velocity for Note 1a
        }
    }
    // bf will always be on a non-command at this point - either a move or empty buffer

    // process move
    if (bf->block_type == BLOCK_TYPE_ALINE)
    { // do cases 1a - 1e; finish cases 1f - 1k
        if (bf->buffer_state == MP_BUFFER_BACK_PLANNED)
        { // do 1a; finish 1f, 1j, 2d, 2i
            _plan_aline(bf, entry_velocity);
            planned_something = true;
        }
    }
    return (planned_something ? STAT_OK : STAT_NOOP);
}

/*************************************************************************
 * mp_exec_move() - 执行运行时函数以准备步进器的移动
 *
 *  使缓冲区队列出列并执行移动连续。
 *  管理运行缓冲区和其他详细信息
 */

stat_t mp_exec_move()
{
    mpBuf_t *bf;

    // It is possible to try to try to exec from a priming planner if coming off a hold
    // This occurs if new p1 commands (and were held back) arrived while in a hold
    //    if (mp->planner_state <= MP_BUFFER_BACK_PLANNED) {
    //        st_prep_null();
    //        return (STAT_NOOP);
    //    }

    // 运行带外停留的。 它可能是在之前的st_load_move（）中设置的
    if (mr->out_of_band_dwell_flag) //有条件地执行带外停顿
    {
        mr->out_of_band_dwell_flag = false;
        st_prep_out_of_band_dwell(mr->out_of_band_dwell_seconds * 1000000);
        return (STAT_OK);
    }

    // 获取NULL缓冲区意味着队列中没有运行任何东西 - 这没关系
    if ((bf = mp_get_run_buffer()) == NULL) //bf=mp->q.r
    {
        st_prep_null();
        return (STAT_NOOP);
    }

    if (bf->block_type == BLOCK_TYPE_ALINE) //加速计划线
    {                                       // 仅为行循环自动启动cycle auto-start for lines only
        // 第一次操作

        if (bf->buffer_state != MP_BUFFER_RUNNING) //!=当前运行缓冲区
        {
            if ((bf->buffer_state < MP_BUFFER_BACK_PLANNED) && (cm->motion_state == MOTION_RUN))
            {
                //debug_trap("mp_exec_move() 缓冲区未准备好。 饥饿"); // 重要提示：不能从这里rpt_exception！
                st_prep_null();
                return (STAT_NOOP);
            }
            if ((bf->nx->buffer_state < MP_BUFFER_BACK_PLANNED) && (bf->nx->buffer_state > MP_BUFFER_EMPTY))
            {
                //这可以检测缓冲区饥饿，但也可以是单行“慢跑”或命令
                // rpt_exception（42，“mp_exec_move（）下一个缓冲区为空”）;
                // ^^^导致崩溃。 我们不能从这里rpt_exception！
                debug_trap("mp_exec_move() 没有缓冲准备 - 饥饿");
            }

            if (bf->buffer_state == MP_BUFFER_BACK_PLANNED) // 缓冲准备好进行最终规划; 速度已经确定
            {
                debug_trap_if_true((cm->motion_state == MOTION_RUN), "mp_exec_move() 缓冲准备但没有计划");
                //重要提示：不能从这里获取rpt_exception！
                //我们需要有计划。 我们不想在这里这样做，
                //因为它可能已经发生在较低的中断中。
                st_request_forward_plan(); //请求前进计划 fwd_plan_timer.setInterruptPending();
                return (STAT_NOOP);
            }

            if (bf->buffer_state == MP_BUFFER_FULLY_PLANNED) //== 缓冲完全计划。 可能仍然需要重新计划
            {
                bf->buffer_state = MP_BUFFER_RUNNING; // 当前运行缓冲区 // 必须先于mp_planner_time_acccounting（）
            }
            else
            {
                return (STAT_NOOP);
            }
            mp_planner_time_accounting(); //计划者时间累计
        }

        //继续*问*以便进行下一步行动的前瞻性计划。
        //在我们离开这个函数之前，这不会调用mp_plan_move
        //（并通过bf-> bf_func调用了mp_exec_aline）。
        //这也允许mp_exec_aline首先提前mr-> p。
        if (bf->nx->buffer_state >= MP_BUFFER_BACK_PLANNED) //>=缓冲准备好进行最终规划; 速度已经确定
        {
            st_request_forward_plan(); //请求前进计划 fwd_plan_timer.setInterruptPending();
        }
    }
    if (bf->bf_func == NULL)
    {
        return (cm_panic(STAT_INTERNAL_ERROR, "mp_exec_move()")); // 永远不应该到这里来
    }
    return (bf->bf_func(bf)); // 在planner缓冲区中运行move回调
}

/*************************************************************************/
/**** ALINE EXECUTION ROUTINES *******************************************/
/*************************************************************************
 * ---> Everything here fires from interrupts and must be interrupt safe
 *
 *  _exec_aline()           - acceleration line main routine
 *    _exec_aline_head()    - helper for acceleration section
 *    _exec_aline_body()    - helper for cruise section
 *    _exec_aline_tail()    - helper for deceleration section
 *    _exec_aline_segment() - helper for running a segment
 *
 *  Returns:
 *     STAT_OK        move is done
 *     STAT_EAGAIN    move is not finished - has more segments to run
 *     STAT_NOOP      would cause no operation to the steppers - do not load the move
 *     STAT_xxxxx     fatal error. Ends the move and frees the bf buffer
 *
 *  This routine is called from the (LO) interrupt level. The interrupt sequencing 
 *  relies on the behaviors of the routines being exactly correct. Each call to 
 *  _exec_aline() must execute and prep **one and only one** segment. If the segment 
 *  is the not the last segment in the bf buffer the _aline() must return STAT_EAGAIN. 
 *  If it's the last segment it must return STAT_OK. If it encounters a fatal error 
 *  that would terminate the move it should return a valid error code. Failure to 
 *  obey this will introduce subtle and very difficult to diagnose bugs (trust us on this).
 *
 *   Note 1: Returning STAT_OK ends the move and frees the bf buffer.
 *           Returning STAT_OK at this point does NOT advance the position vector, 
 *              meaning any position error will be compensated by the next move.
 *
 *   Note 2: BF/MR sequencing solves a potential race condition where the current move 
 *           ends but the new move has not started because the previous move is still 
 *           being run by the steppers. Planning can overwrite the new move.
 */
/* --- State transitions - hierarchical state machine ---
 *
 *  bf->block_state transitions:
 *     from _NEW to _RUN on first call (sub_state set to _OFF)
 *     from _RUN to _OFF on final call
 *      or just remains _OFF
 *
 *  mr->block_state transitions on first call from _OFF to one of _HEAD, _BODY, _TAIL
 *  Within each section state may be
 *     _NEW - trigger initialization
 *     _RUN1 - run the first part
 *     _RUN2 - run the second part
 *
 *  Important distinction to note:
 *    - mp_plan move() is called for every type of move (bf block)
 *    - mp_exec_move() is called for every type of move
 *    - mp_exec_aline() is only called for alines
 */
/* Synchronization of run BUFFER and run BLOCK
 *
 * Note first: mp_exec_aline() makes a huge assumption: When it comes time to get a 
 *  new run block (mr->r) it assumes the planner block (mr->p) has been fully planned 
 *  via the JIT forward planning and is ready for use as the new run block.
 *
 * The runtime uses 2 structures for the current move or commend, the run BUFFER 
 *  from the planner queue (mb.r, aka bf), and the run BLOCK from the runtime 
 *  singleton (mr->r). These structures are synchronized implicitly, but not 
 *  explicitly referenced, as pointers can lead to race conditions. 
 *  See plan_zoid.cpp / mp_calculate_ramps() for more details
 *
 * When mp_exec_aline() needs to grab a new planner buffer for a new move or command 
 *  (i.e. block state is inactive) it swaps (rolls) the run and planner BLOCKS so that
 *  mr->p (planner block) is now the mr->r (run block), and the old mr->r block becomes 
 *  available for planning; it becomes mr->p block.
 *
 * At the same time, it's when finished with its current run buffer (mb.r), it has already 
 *  advanced to the next buffer. mp_exec_move() does this at the end of previous move.
 *  Or in the bootstrap case, there never was a previous mb.r, so the current one is OK.
 * 
 * As if by magic, the new mb.r aligns with the run block that was just moved in from 
 *  the planning block.
 */

/**** NOTICE ** NOTICE ** NOTICE ****
 **
 **    mp_exec_aline() is called in
 **     --INTERRUPT CONTEXT!!--
 **
 **    Things we MUST NOT do (even indirectly):
 **       mp_plan_buffer()
 **       mp_plan_block_list()
 **       printf()
 **
 **** NOTICE ** NOTICE ** NOTICE ****/

stat_t mp_exec_aline(mpBuf_t *bf)
{
    // don't run the block if the machine is not in cycle
    if (cm_get_machine_state() != MACHINE_CYCLE)
    {
        return (STAT_NOOP);
    }

    // don't run the block if the block is inactive
    if (bf->block_state == BLOCK_INACTIVE)
    {
        return (STAT_NOOP);
    }

    stat_t status;

    // Initialize all new blocks, regardless of normal or feedhold operation
    if (mr->block_state == BLOCK_INACTIVE)
    {

        // ASSERTIONS

        // Zero length moves (and other too-short moves) should have already been removed earlier
        // But let's still alert the condition should it ever occur
        debug_trap_if_zero(bf->length, "mp_exec_aline() zero length move");

        // These equalities in the assertions must be true for this to work:
        //   entry_velocity <= cruise_velocity
        //   exit_velocity  <= cruise_velocity
        //
        // NB: Even if the move is head or tail only, cruise velocity needs to be valid.
        // This is because a "head" is *always* entry->cruise, and a "tail" is *always* cruise->exit,
        // even if there are no other sections in the move. (This is a significant time savings.)
        debug_trap_if_true((mr->entry_velocity > mr->r->cruise_velocity),
                           "mp_exec_aline() mr->entry_velocity > mr->r->cruise_velocity");

        debug_trap_if_true((mr->r->exit_velocity > mr->r->cruise_velocity),
                           "mp_exec_aline() mr->exit_velocity > mr->r->cruise_velocity");

        // Start a new move by setting up the runtime singleton (mr)
        memcpy(&mr->gm, &(bf->gm), sizeof(GCodeState_t)); // copy in the gcode model state
        bf->block_state = BLOCK_ACTIVE;                   // note that this buffer is running
        mr->block_state = BLOCK_INITIAL_ACTION;           // note the planner doesn't look at block_state

        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // !!! THIS IS THE ONLY PLACE WHERE mr->r AND mr->p ARE ALLOWED TO BE CHANGED !!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // Swap P and R blocks
        mr->r = mr->p;     // we are now going to run the planning block
        mr->p = mr->p->nx; // re-use the old running block as the new planning block

        // Check to make sure no sections are less than MIN_SEGMENT_TIME & adjust if necessary
        _exec_aline_normalize_block(mr->r);

        // transfer move parameters from planner buffer to the runtime
        copy_vector(mr->unit, bf->unit);
        copy_vector(mr->target, bf->gm.target);
        copy_vector(mr->axis_flags, bf->axis_flags);

        mr->run_bf = bf;      // DIAGNOSTIC: points to running bf
        mr->plan_bf = bf->nx; // DIAGNOSTIC: points to next bf to forward plan

        // characterize the move for starting section - head/body/tail
        mr->section_state = SECTION_NEW;
        mr->section = SECTION_HEAD;
        if (fp_ZERO(mr->r->head_length))
        {
            mr->section = SECTION_BODY;
            if (fp_ZERO(mr->r->body_length))
            {
                mr->section = SECTION_TAIL;
            }
        }

        // generate the way points for position correction at section ends
        for (uint8_t axis = 0; axis < AXES; axis++)
        {
            mr->waypoint[SECTION_HEAD][axis] = mr->position[axis] + mr->unit[axis] * mr->r->head_length;
            mr->waypoint[SECTION_BODY][axis] = mr->position[axis] + mr->unit[axis] * (mr->r->head_length + mr->r->body_length);
            mr->waypoint[SECTION_TAIL][axis] = mr->position[axis] + mr->unit[axis] * (mr->r->head_length + mr->r->body_length + mr->r->tail_length);
        }
    }

    // Feed Override Processing - We need to handle the following cases (listed in rough sequence order):

    // Feedhold Processing - We need to handle the following cases (listed in rough sequence order):
    if (cm->hold_state != FEEDHOLD_OFF)
    {
        // if running actions, or in HOLD state, or exiting with actions
        if (cm->hold_state >= FEEDHOLD_MOTION_STOPPED)
        {                       // handles _exec_aline_feedhold_processing case (7)
            return (STAT_NOOP); // VERY IMPORTANT to exit as a NOOP. Do not load another move
        }
        // STAT_OK terminates aline execution for this move
        // STAT_NOOP terminates execution and does not load another move
        status = _exec_aline_feedhold(bf);
        if ((status == STAT_OK) || (status == STAT_NOOP))
        {
            return (status);
        }
    }

    mr->block_state = BLOCK_ACTIVE;

    // NB: from this point on the contents of the bf buffer do not affect execution

    //**** main dispatcher to process segments ***
    status = STAT_OK;
    if (mr->section == SECTION_HEAD)
    {
        status = _exec_aline_head(bf);
    }
    else if (mr->section == SECTION_BODY)
    {
        status = _exec_aline_body(bf);
    }
    else if (mr->section == SECTION_TAIL)
    {
        status = _exec_aline_tail(bf);
    }
    else
    {
        return (cm_panic(STAT_INTERNAL_ERROR, "exec_aline()"));
    } // never supposed to get here

    // Conditionally set the move to be unplannable. We can't use the if/else block above,
    // since the head may call a body or a tail, and a body call tail, so we wait till after.
    //
    // Conditions are:
    //  - Allow 3 segments: 1 segment isn't enough, because there's one running as we execute,
    //    so it has to be the next one. There's a slight possibility we'll miss that, since we
    //    didn't necessarily start at the beginning, so three.
    //  - If it's a head/tail move and we've started the head we can't replan it anyway as
    //    the head can't be interrupted, and the tail is already as sharp as it can be (or there'd be a body)
    //  - ...so if you are in a body mark the body unplannable if we are too close to its end.
    if ((mr->section == SECTION_TAIL) || ((mr->section == SECTION_BODY) && (mr->segment_count < 3)))
    {
        bf->plannable = false;
    }

    // Feedhold Case (3): Look for the end of the deceleration to transition HOLD states
    // This code sets states used by _exec_feedhold_processing() helper.
    if (cm->hold_state == FEEDHOLD_DECEL_TO_ZERO)
    {
        if ((status == STAT_OK) || (status == STAT_NOOP))
        {
            cm->hold_state = FEEDHOLD_DECEL_COMPLETE;
            bf->block_state = BLOCK_INITIAL_ACTION; // reset bf so it can restart the rest of the move
        }
    }

    // Perform motion state transition. Also sets active model to RUNTIME
    if (cm->motion_state != MOTION_RUN)
    {
        cm_set_motion_state(MOTION_RUN);
    }

    // There are 4 things that can happen here depending on return conditions:
    //  status        bf->block_state       Description
    //  -----------   --------------        ----------------------------------------
    //  STAT_EAGAIN   <don't care>          mr buffer has more segments to run
    //  STAT_OK       BLOCK_ACTIVE          mr and bf buffers are done
    //  STAT_OK       BLOCK_INITIAL_ACTION  mr done; bf must be run again (it's been reused)
    //  STAT_NOOP     <don't care>          treated as a STAT_OK

    if (status == STAT_EAGAIN)
    {
        sr_request_status_report(SR_REQUEST_TIMED); // continue reporting mr buffer
                                                    // Note that that'll happen in a lower interrupt level.
    }
    else
    {
        mr->block_state = BLOCK_INACTIVE; // invalidate mr buffer (reset)
        mr->section_state = SECTION_OFF;
        mp->run_time_remaining = 0.0;              // it's done, so time goes to zero
        mr->entry_velocity = mr->r->exit_velocity; // feed the old exit into the entry.

        if (bf->block_state == BLOCK_ACTIVE)
        {
            if (mp_free_run_buffer())
            { // returns true of the buffer is empty
                if (cm->hold_state == FEEDHOLD_OFF)
                {
                    cm_set_motion_state(MOTION_STOP); // also sets active model to RUNTIME
                    cm_cycle_end();                   // free buffer & end cycle if planner is empty
                }
            }
            else
            {
                st_request_forward_plan();
            }
        }
    }
    return (status);
}

/*
 * Forward difference math explained:
 *
 *  We are using a quintic (fifth-degree) Bezier polynomial for the velocity curve.
 *  This gives us a "linear pop" velocity curve; with pop being the sixth derivative of position:
 *  velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
 *
 *  The Bezier curve takes the form:
 *
 *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
 *
 *  Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
 *  through B_5(t) are the Bernstein basis as follows:
 *
 *        B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
 *        B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
 *        B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
 *        B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
 *        B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
 *        B_5(t) =             t^5  =    t^5
 *                                      ^       ^       ^       ^       ^       ^
 *                                      |       |       |       |       |       |
 *                                      A       B       C       D       E       F
 *
 *
 *  We use forward-differencing to calculate each position through the curve.
 *    This requires a formula of the form:
 *
 *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
 *
 *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
 *  through t of the Bezier form of V(t), we can determine that:
 *
 *        A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
 *        B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
 *        C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
 *        D =  10*P_0 - 20*P_1 + 10*P_2
 *        E = - 5*P_0 +  5*P_1
 *        F =     P_0
 *
 *  Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
 *  We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
 *  which, after simplification, resolves to:
 *
 *        A = - 6*P_i +  6*P_t
 *        B =  15*P_i - 15*P_t
 *        C = -10*P_i + 10*P_t
 *        D = 0
 *        E = 0
 *        F = P_i
 *
 *  Given an interval count of I to get from P_i to P_t, we get the parametric "step" size of h = 1/I.
 *  We need to calculate the initial value of forward differences (F_0 - F_5) such that the inital
 *  velocity V = P_i, then we iterate over the following I times:
 *
 *        V   += F_5
 *        F_5 += F_4
 *        F_4 += F_3
 *        F_3 += F_2
 *        F_2 += F_1
 *
 *  See http://www.drdobbs.com/forward-difference-calculation-of-bezier/184403417 for an example of
 *  how to calculate F_0 - F_5 for a cubic bezier curve. Since this is a quintic bezier curve, we
 *  need to extend the formulas somewhat. I'll not go into the long-winded step-by-step here,
 *  but it gives the resulting formulas:
 *
 *        a = A, b = B, c = C, d = D, e = E, f = F
 *        F_5(t+h)-F_5(t) = (5ah)t^4 + (10ah^2 + 4bh)t^3 + (10ah^3 + 6bh^2 + 3ch)t^2 +
 *            (5ah^4 + 4bh^3 + 3ch^2 + 2dh)t + ah^5 + bh^4 + ch^3 + dh^2 + eh
 *
 *        a = 5ah
 *        b = 10ah^2 + 4bh
 *        c = 10ah^3 + 6bh^2 + 3ch
 *        d = 5ah^4 + 4bh^3 + 3ch^2 + 2dh
 *
 *  (After substitution, simplification, and rearranging):
 *        F_4(t+h)-F_4(t) = (20ah^2)t^3 + (60ah^3 + 12bh^2)t^2 + (70ah^4 + 24bh^3 + 6ch^2)t +
 *            30ah^5 + 14bh^4 + 6ch^3 + 2dh^2
 *
 *        a = (20ah^2)
 *        b = (60ah^3 + 12bh^2)
 *        c = (70ah^4 + 24bh^3 + 6ch^2)
 *
 *  (After substitution, simplification, and rearranging):
 *        F_3(t+h)-F_3(t) = (60ah^3)t^2 + (180ah^4 + 24bh^3)t + 150ah^5 + 36bh^4 + 6ch^3
 *
 *  (You get the picture...)
 *        F_2(t+h)-F_2(t) = (120ah^4)t + 240ah^5 + 24bh^4
 *        F_1(t+h)-F_1(t) = 120ah^5
 *
 *  Normally, we could then assign t = 0, use the A-F values from above, and get out initial F_* values.
 *  However, for the sake of "averaging" the velocity of each segment, we actually want to have the initial
 *  V be be at t = h/2 and iterate I-1 times. So, the resulting F_* values are (steps not shown):
 *
 *        F_5 = (121Ah^5)/16 + 5Bh^4 + (13Ch^3)/4 + 2Dh^2 + Eh
 *        F_4 = (165Ah^5)/2 + 29Bh^4 + 9Ch^3 + 2Dh^2
 *        F_3 = 255Ah^5 + 48Bh^4 + 6Ch^3
 *        F_2 = 300Ah^5 + 24Bh^4
 *        F_1 = 120Ah^5
 *
 *  Note that with our current control points, D and E are actually 0.
 */

// Total time: 147us
static void _init_forward_diffs(const float v_0, const float v_1)
{
    // Times from *here*
    /* Full formulation:
     const float fifth_T        = T * 0.2; //(1/5) T
     const float two_fifths_T   = T * 0.4; //(2/5) T
     const float twentienth_T_2 = T * T * 0.05; // (1/20) T^2

     const float P_0 = v_0;
     const float P_1 = v_0 +      fifth_T*a_0;
     const float P_2 = v_0 + two_fifths_T*a_0 + twentienth_T_2*j_0;
     const float P_3 = v_1 - two_fifths_T*a_1 + twentienth_T_2*j_1;
     const float P_4 = v_1 -      fifth_T*a_1;
     const float P_5 = v_1;

     const float A =  5*( P_1 - P_4 + 2*(P_3 - P_2) ) +   P_5 - P_0;
     const float B =  5*( P_0 + P_4 - 4*(P_3 + P_1)   + 6*P_2 );
     const float C = 10*( P_3 - P_0 + 3*(P_1 - P_2) );
     const float D = 10*( P_0 + P_2 - 2*P_1 );
     const float E =  5*( P_1 - P_0 );
     //const float F = P_0;
*/
    float A = -6.0 * v_0 + 6.0 * v_1;
    float B = 15.0 * v_0 - 15.0 * v_1;
    float C = -10.0 * v_0 + 10.0 * v_1;
    // D = 0
    // E = 0
    // F = Vi

    const float h = 1 / (mr->segments);
    const float h_2 = h * h;
    const float h_3 = h_2 * h;
    const float h_4 = h_3 * h;
    const float h_5 = h_4 * h;

    const float Ah_5 = A * h_5;
    const float Bh_4 = B * h_4;
    const float Ch_3 = C * h_3;

    const float const1 = 7.5625; // (121.0/16.0)
    const float const2 = 3.25;   // ( 13.0/ 4.0)
    const float const3 = 82.5;   // (165.0/ 2.0)

    /*
     *  F_5 = (121/16)A h^5 +  5 B h^4 + (13/4) C h^3 + 2 D h^2 + Eh
     *  F_4 =  (165/2)A h^5 + 29 B h^4 +     9  C h^3 + 2 D h^2
     *  F_3 =     255 A h^5 + 48 B h^4 +     6  C h^3
     *  F_2 =     300 A h^5 + 24 B h^4
     *  F_1 =     120 A h^5
     */

    mr->forward_diff_5 = const1 * Ah_5 + 5.0 * Bh_4 + const2 * Ch_3;
    mr->forward_diff_4 = const3 * Ah_5 + 29.0 * Bh_4 + 9.0 * Ch_3;
    mr->forward_diff_3 = 255.0 * Ah_5 + 48.0 * Bh_4 + 6.0 * Ch_3;
    mr->forward_diff_2 = 300.0 * Ah_5 + 24.0 * Bh_4;
    mr->forward_diff_1 = 120.0 * Ah_5;

    // Calculate the initial velocity by calculating V(h/2)
    const float half_h = h * 0.5; // h/2
    const float half_h_3 = half_h * half_h * half_h;
    const float half_h_4 = half_h_3 * half_h;
    const float half_h_5 = half_h_4 * half_h;

    const float half_Ch_3 = C * half_h_3;
    const float half_Bh_4 = B * half_h_4;
    const float half_Ah_5 = A * half_h_5;

    mr->segment_velocity = half_Ah_5 + half_Bh_4 + half_Ch_3 + v_0;
}

/*********************************************************************************************
 * _exec_aline_head()
 */

static stat_t _exec_aline_head(mpBuf_t *bf)
{
    bool first_pass = false;
    if (mr->section_state == SECTION_NEW)
    { // INITIALIZATION
        first_pass = true;
        if (fp_ZERO(mr->r->head_length))
        { // Needed here as feedhold may have changed the block
            mr->section = SECTION_BODY;
            return (_exec_aline_body(bf)); // skip ahead to the body generator
        }
        mr->segments = ceil(uSec(mr->r->head_time) / NOM_SEGMENT_USEC); // # of segments for the section
        mr->segment_count = (uint32_t)mr->segments;
        mr->segment_time = mr->r->head_time / mr->segments; // time to advance for each segment

        if (mr->segment_count == 1)
        {
            // We will only have one segment, simply average the velocities
            mr->segment_velocity = mr->r->head_length / mr->segment_time;
        }
        else
        {
            _init_forward_diffs(mr->entry_velocity, mr->r->cruise_velocity); // sets initial segment_velocity
        }
        if (mr->segment_time < MIN_SEGMENT_TIME)
        {
            debug_trap("mr->segment_time < MIN_SEGMENT_TIME (head)");
            return (STAT_OK); // exit without advancing position, say we're done
        }
        // If this trap ever fires put this statement back in: mr->section = SECTION_HEAD;
        debug_trap_if_true(mr->section != SECTION_HEAD, "exec_aline() Not section head");

        mr->section_state = SECTION_RUNNING;
    }
    else
    {
        mr->segment_velocity += mr->forward_diff_5;
    }

    if (_exec_aline_segment() == STAT_OK)
    { // set up for second half
        if ((fp_ZERO(mr->r->body_length)) && (fp_ZERO(mr->r->tail_length)))
        {
            return (STAT_OK); // ends the move
        }
        mr->section = SECTION_BODY; // advance to body
        mr->section_state = SECTION_NEW;
    }
    else if (!first_pass)
    {
        mr->forward_diff_5 += mr->forward_diff_4;
        mr->forward_diff_4 += mr->forward_diff_3;
        mr->forward_diff_3 += mr->forward_diff_2;
        mr->forward_diff_2 += mr->forward_diff_1;
    }
    return (STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_body()
 *
 *  The body is broken into little segments even though it is a straight line 
 *  so that feed holds can happen in the middle of a line with minimum latency
 */
static stat_t _exec_aline_body(mpBuf_t *bf)
{
    if (mr->section_state == SECTION_NEW)
    {
        if (fp_ZERO(mr->r->body_length))
        { // Needed here as feedhold may have changed the block
            mr->section = SECTION_TAIL;
            return (_exec_aline_tail(bf)); // skip ahead to tail generator
        }
        float body_time = mr->r->body_time;
        mr->segments = ceil(uSec(body_time) / NOM_SEGMENT_USEC);
        mr->segment_time = body_time / mr->segments;
        mr->segment_velocity = mr->r->cruise_velocity;
        mr->segment_count = (uint32_t)mr->segments;
        if (mr->segment_time < MIN_SEGMENT_TIME)
        {
            debug_trap("mr->segment_time < MIN_SEGMENT_TIME (body)");
            return (STAT_OK); // exit without advancing position, say we're done
        }
        // If this trap ever fires put this statement back in: mr->section = SECTION_BODY;
        debug_trap_if_true(mr->section != SECTION_BODY, "exec_aline() Not section body");

        mr->section_state = SECTION_RUNNING; // uses PERIOD_2 so last segment detection works
    }
    if (_exec_aline_segment() == STAT_OK)
    { // OK means this section is done
        if (fp_ZERO(mr->r->tail_length))
        {
            return (STAT_OK); // ends the move
        }
        mr->section = SECTION_TAIL; // advance to tail
        mr->section_state = SECTION_NEW;
    }
    return (STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_tail()
 */

static stat_t _exec_aline_tail(mpBuf_t *bf)
{
    bool first_pass = false;
    if (mr->section_state == SECTION_NEW)
    { // INITIALIZATION
        first_pass = true;
        bf->plannable = false; // Mark the block as unplannable

        if (fp_ZERO(mr->r->tail_length))
        {                     // Needed here as feedhold may have changed the block
            return (STAT_OK); // end the move
        }
        mr->segments = ceil(uSec(mr->r->tail_time) / NOM_SEGMENT_USEC); // # of segments for the section
        mr->segment_count = (uint32_t)mr->segments;
        mr->segment_time = mr->r->tail_time / mr->segments; // time to advance for each segment

        if (mr->segment_count == 1)
        {
            mr->segment_velocity = mr->r->tail_length / mr->segment_time;
        }
        else
        {
            _init_forward_diffs(mr->r->cruise_velocity, mr->r->exit_velocity); // sets initial segment_velocity
        }
        if (mr->segment_time < MIN_SEGMENT_TIME)
        {
            debug_trap("mr->segment_time < MIN_SEGMENT_TIME (tail)");
            return (STAT_OK); // exit without advancing position, say we're done
        }
        // If this trap ever fires put this statement back in: mr->section = SECTION_TAIL;
        debug_trap_if_true(mr->section != SECTION_TAIL, "exec_aline() Not section tail");

        mr->section_state = SECTION_RUNNING;
    }
    else
    {
        mr->segment_velocity += mr->forward_diff_5;
    }

    if (_exec_aline_segment() == STAT_OK)
    {
        return (STAT_OK); // STAT_OK completes the move
    }
    else if (!first_pass)
    {
        mr->forward_diff_5 += mr->forward_diff_4;
        mr->forward_diff_4 += mr->forward_diff_3;
        mr->forward_diff_3 += mr->forward_diff_2;
        mr->forward_diff_2 += mr->forward_diff_1;
    }
    return (STAT_EAGAIN);
}

/*********************************************************************************************
 * _exec_aline_segment() - segment runner helper
 *
 * NOTES ON STEP ERROR CORRECTION:
 *
 *  The commanded_steps are the target_steps delayed by one more segment.
 *  This lines them up in time with the encoder readings so a following error can be generated
 *
 *  The following_error term is positive if the encoder reading is greater than (ahead of)
 *  the commanded steps, and negative (behind) if the encoder reading is less than the
 *  commanded steps. The following error is not affected by the direction of movement -
 *  it's purely a statement of relative position. Examples:
 *
 *      Encoder  Commanded   Following Err
 *          100         90           +10        encoder is 10 steps ahead of commanded steps
 *          -90       -100           +10        encoder is 10 steps ahead of commanded steps
 *           90        100           -10        encoder is 10 steps behind commanded steps
 *         -100        -90           -10        encoder is 10 steps behind commanded steps
 */

static stat_t _exec_aline_segment()
{
    float travel_steps[MOTORS];

    // Set target position for the segment
    // If the segment ends on a section waypoint synchronize to the head, body or tail end
    // Otherwise if not at a section waypoint compute target from segment time and velocity
    // Don't do waypoint correction if you are going into a hold.

    if ((--mr->segment_count == 0) && (cm->hold_state == FEEDHOLD_OFF))
    {
        copy_vector(mr->gm.target, mr->waypoint[mr->section]);
    }
    else
    {
        float segment_length = mr->segment_velocity * mr->segment_time;
        // See https://en.wikipedia.org/wiki/Kahan_summation_algorithm
        // for the summation compensation description
        for (uint8_t a = 0; a < AXES; a++)
        {
            float to_add = (mr->unit[a] * segment_length) - mr->gm.target_comp[a];
            float target = mr->position[a] + to_add;
            mr->gm.target_comp[a] = (target - mr->position[a]) - to_add;
            mr->gm.target[a] = target;
            // the above replaces this line:
            // mr->gm.target[a] = mr->position[a] + (mr->unit[a] * segment_length);
        }
    }

    // Convert target position to steps
    // Bucket-brigade the old target down the chain before getting the new target from kinematics
    //
    // Very small travels of less than 0.01 step are truncated to zero. This is to correct a condition
    // where a rounding error in kinematics could reverse the direction of a move in the extreme head or tail.
    // Truncating the move contributes to positional error, but this is corrected by encoder feedback should
    // it ever accumulate to more than one step.
    //
    // NB: The direct manipulation of steps to compute travel_steps only works for Cartesian kinematics.
    //     Other kinematics may require transforming travel distance as opposed to simply subtracting steps.

    for (uint8_t m = 0; m < MOTORS; m++)
    {
        mr->commanded_steps[m] = mr->position_steps[m]; // previous segment's position, delayed by 1 segment
        mr->position_steps[m] = mr->target_steps[m];    // previous segment's target becomes position
        mr->encoder_steps[m] = en_read_encoder(m);      // get current encoder position (time aligns to commanded_steps)
        mr->following_error[m] = mr->encoder_steps[m] - mr->commanded_steps[m];
    }
    kn_inverse_kinematics(mr->gm.target, mr->target_steps); // now determine the target steps...

    for (uint8_t m = 0; m < MOTORS; m++)
    { // and compute the distances to be traveled
        travel_steps[m] = mr->target_steps[m] - mr->position_steps[m];
        if (fabs(travel_steps[m]) < 0.01)
        { // truncate very small moves to deal with rounding errors
            travel_steps[m] = 0;
        }
    }

    // Update the mb->run_time_remaining -- we know it's missing the current segment's time before it's loaded, that's ok.
    mp->run_time_remaining -= mr->segment_time;
    if (mp->run_time_remaining < 0)
    {
        mp->run_time_remaining = 0.0;
    }

    // Call the stepper prep function
    ritorno(st_prep_line(travel_steps, mr->following_error, mr->segment_time));
    copy_vector(mr->position, mr->gm.target); // update position from target
    if (mr->segment_count == 0)
    {
        return (STAT_OK); // this section has run all its segments
    }
    return (STAT_EAGAIN); // this section still has more segments to run
}

/*********************************************************************************************
 * _exec_aline_normalize_block() - re-organize block to eliminate minimum time segments
 *
 * Check to make sure no sections are less than MIN_SEGMENT_TIME & adjust if necessary
 */

static void _exec_aline_normalize_block(mpBlockRuntimeBuf_t *b)
{
    if ((b->head_length > 0) && (b->head_time < MIN_SEGMENT_TIME))
    {
        // Compute the new body time. head_time !== body_time
        b->body_length += b->head_length;
        b->body_time = b->body_length / b->cruise_velocity;
        b->head_length = 0;
        b->head_time = 0;
    }
    if ((b->tail_length > 0) && (b->tail_time < MIN_SEGMENT_TIME))
    {
        // Compute the new body time. tail_time !== body_time
        b->body_length += b->tail_length;
        b->body_time = b->body_length / b->cruise_velocity;
        b->tail_length = 0;
        b->tail_time = 0;
    }

    // At this point, we've already possibly merged head and/or tail into the body.
    // If the body is still too "short" (brief) we *might* be able to add it to a head or tail.
    // If there's still a head or a tail, we will add the body to whichever there is, maybe both.
    // We saved it for last since it's the most expensive.
    if ((b->body_length > 0) && (b->body_time < MIN_SEGMENT_TIME))
    {

        // We'll add the time to either the head or the tail or split it
        if (b->tail_length > 0)
        {
            if (b->head_length > 0)
            { // Split the body to the head and tail
                b->head_length += b->body_length * 0.5;
                b->tail_length += b->body_length * 0.5; // let the compiler optimize out one of these *
                b->head_time = (2.0 * b->head_length) / (mr->entry_velocity + b->cruise_velocity);
                b->tail_time = (2.0 * b->tail_length) / (b->cruise_velocity + b->exit_velocity);
                b->body_length = 0;
                b->body_time = 0;
            }
            else
            { // Put it all in the tail
                b->tail_length += b->body_length;
                b->tail_time = (2.0 * b->tail_length) / (b->cruise_velocity + b->exit_velocity);
                b->body_length = 0;
                b->body_time = 0;
            }
        }
        else if (b->head_length > 0)
        { // Put it all in the head
            b->head_length += b->body_length;
            b->head_time = (2.0 * b->head_length) / (mr->entry_velocity + b->cruise_velocity);
            b->body_length = 0;
            b->body_time = 0;
        }
        else
        { // Uh oh! We have a move that's all body, and is still too short!!
            debug_trap("_exec_aline_normalize_block() - found a move that is too short");
        }
    }
}

/*********************************************************************************************
 * _exec_aline_feedhold() - feedhold helper for mp_exec_aline()
 *
 *  This function performs the bulk of the feedhold state machine processing from within 
 *  mp_exec_aline(). There is also a little chunk labeled "Feedhold Case (3-continued)".
 *  Feedhold processing mostly manages the deceleration phase into the hold, and sets 
 *  state variables used in cycle_feedhold.cpp
 *
 *  Returns:
 *
 *    STAT_OK     exits from mp_exec_aline() but allows another segment to be loaded  
 *                and executed. This is used when the hold is still in continuous motion.
 *
 *    STAT_NOOP   exits from mp_exec_aline() and prevents another segment loading and
 *                executing. This is used when the hold has stopped at the hold point.
 *  
 *    STAT_EAGAIN allows mp_exec_aline() to continue execution, playing out a head, body
 *                or tail.
 */

static stat_t _exec_aline_feedhold(mpBuf_t *bf)
{
    // Case (4) - Wait for the steppers to stop and complete the feedhold
    if (cm->hold_state == FEEDHOLD_MOTION_STOPPING)
    {
        if (mp_runtime_is_idle())
        { // wait for steppers to actually finish

            // Motion has stopped, so we can rely on positions and other values to be stable

            // If hold was SKIP type, discard the remainder of the block and position to the next block
            if (cm->hold_type == FEEDHOLD_TYPE_SKIP)
            {
                copy_vector(mp->position, mr->position); // update planner position to the final runtime position
                mp_free_run_buffer();                    // advance to next block, discarding the rest of the move
            }

            // Otherwise setup the block to complete motion (regardless of how hold will ultimately be exited)
            else
            {
                bf->length = get_axis_vector_length(mr->position, mr->target); // update bf w/remaining length in move

                // If length ~= 0 it's because the deceleration was exact. Handle this exception to avoid planning errors
                if (bf->length < EPSILON4)
                {
                    copy_vector(mp->position, mr->position); // update planner position to the final runtime position
                    mp_free_run_buffer();                    // advance to next block, discarding the zero-length move
                }
                else
                {
                    bf->block_state = BLOCK_INITIAL_ACTION; // tell _exec to re-use the bf buffer
                    while (bf->buffer_state > MP_BUFFER_BACK_PLANNED)
                    {
                        bf->buffer_state = MP_BUFFER_BACK_PLANNED; // revert from RUNNING so it can be forward planned again
                        bf->plannable = true;                      // needed so block can be re-planned
                        bf = mp_get_next_buffer(bf);
                    }
                }
            }
            mr->reset(); // reset MR for next use and for forward planning
            cm_set_motion_state(MOTION_STOP);
            cm->hold_state = FEEDHOLD_MOTION_STOPPED;
            sr_request_status_report(SR_REQUEST_IMMEDIATE);
        }
        return (STAT_NOOP); // hold here. leave with a NOOP so it does not attempt another load and exec.
    }

    // Case (3') - Decelerated to zero. See also Feedhold Case (3) in mp_exec_aline()
    // This state is needed to return an OK to complete the aline exec before transitioning to case (4).
    if (cm->hold_state == FEEDHOLD_DECEL_COMPLETE)
    {
        cm->hold_state = FEEDHOLD_MOTION_STOPPING; // wait for motion to come to a complete stop
        return (STAT_OK);                          // exit from mp_exec_aline()
    }

    // Cases (1x), Case (2)
    // Build a tail-only move from here. Decelerate as fast as possible in the space available.
    if ((cm->hold_state == FEEDHOLD_SYNC) ||
        ((cm->hold_state == FEEDHOLD_DECEL_CONTINUE) && (mr->block_state == BLOCK_INITIAL_ACTION)))
    {

        // Case (1d) - Already decelerating (in a tail), continue the deceleration.
        if (mr->section == SECTION_TAIL)
        { // if already in a tail don't decelerate. You already are
            if (mr->r->exit_velocity < EPSILON2)
            { // allow near-zero velocities to be treated as zero
                cm->hold_state = FEEDHOLD_DECEL_TO_ZERO;
            }
            else
            {
                cm->hold_state = FEEDHOLD_DECEL_CONTINUE;
            }
            return (STAT_EAGAIN); // exiting with EAGAIN will continue exec_aline() execution
        }

        // Case (1a) - Currently accelerating (in a head), skip and waited for body or tail
        //             This is true because to do otherwise the jerk would not have returned to zero.
        // Small exception, if we *just started* the head, then we're not actually accelerating yet.
        if ((mr->section == SECTION_HEAD) && (mr->section_state != SECTION_NEW))
        {
            return (STAT_EAGAIN);
        }

        // Case (1b, 1c) - Block is in a body or about to start a new head. Turn it into a new tail.
        // In the new_head case plan deceleration move (tail) starting at the at the entry velocity
        mr->section = SECTION_TAIL;
        mr->section_state = SECTION_NEW;
        mr->entry_velocity = mr->segment_velocity;
        mr->r->cruise_velocity = mr->entry_velocity;                              // cruise velocity must be set even if there's no body
        mr->r->tail_length = mp_get_target_length(0, mr->r->cruise_velocity, bf); // braking length
        mr->r->head_length = 0;
        mr->r->body_length = 0;
        mr->r->head_time = 0;
        mr->r->body_time = 0;

        // The deceleration distance either fits in the available length or fits exactly or close
        // enough (to EPSILON2) (1e). Case 1e happens frequently when the tail in the move was
        // already planned to zero. EPSILON2 deals with floating point rounding errors that can
        // mis-classify this case. EPSILON2 is 0.0001, which is 0.1 microns in length.
        float available_length = get_axis_vector_length(mr->target, mr->position);

        // Cases (1b1, 1c1) deceleration will fit in the block
        if ((available_length + EPSILON2 - mr->r->tail_length) > 0)
        {
            cm->hold_state = FEEDHOLD_DECEL_TO_ZERO;
            mr->r->exit_velocity = 0;
            mr->r->tail_time = mr->r->tail_length * 2 / (mr->r->exit_velocity + mr->r->cruise_velocity);
            bf->block_time = mr->r->tail_time;
        }
        // Cases (1b2, 1c2) deceleration will not fit in the block
        else
        {
            cm->hold_state = FEEDHOLD_DECEL_CONTINUE;
            mr->r->tail_length = available_length;
            mr->r->exit_velocity = mp_get_decel_velocity(mr->r->cruise_velocity, mr->r->tail_length, bf);
            if (mr->r->exit_velocity >= 0)
            {
                mr->r->tail_time = mr->r->tail_length * 2 / (mr->r->exit_velocity + mr->r->cruise_velocity);
                bf->block_time = mr->r->tail_time;
            }
            // The following branch is rarely if ever taken. It's possible for the deceleration calculation
            // to return an error if the length is too short and other conditions exist. In that case
            // make the block into a cruise (body) and push the deceleration to the next block.
            else
            {
                mr->section = SECTION_BODY;
                mr->r->exit_velocity = mr->r->cruise_velocity; // both should be @ mr->segment_velocity
                mr->r->body_length = available_length;
                mr->r->body_time = mr->r->body_length / mr->r->cruise_velocity;
                mr->r->tail_length = 0;
                mr->r->tail_time = 0;
            }
        }
        _exec_aline_normalize_block(mr->r);
    }
    return (STAT_EAGAIN); // exiting with EAGAIN will continue exec_aline() execution
}
