/*
 * controller.h - g2core controller and main dispatch loop
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
#ifndef CONTROLLER_H_ONCE
#define CONTROLLER_H_ONCE

#include "xio.h"

// see also: g2core.h MESSAGE_LEN and config.h NV_ lengths
#define SAVED_BUFFER_LEN RX_BUFFER_SIZE //保存缓冲区大小（仅用于报表）
#define OUTPUT_BUFFER_LEN 512           //文本缓冲器大小

#define LED_NORMAL_BLINK_RATE 3000      //用于正常操作眨眼率（毫秒）
#define LED_ALARM_BLINK_RATE 750        //对报警状态闪烁速率（毫秒）
#define LED_SHUTDOWN_BLINK_RATE 300     //关机状态闪烁速率（毫秒）
#define LED_PANIC_BLINK_RATE 100        //恐慌状态闪烁速率（毫秒）

typedef enum {                          //管理启动行
    CONTROLLER_INITIALIZING = 0,        // 控制器正在初始化 - 尚未准备好使用
    CONTROLLER_NOT_CONNECTED,           // 尚未检测到与USB（或其他通信通道）的连接
    CONTROLLER_CONNECTED,               // 已连接到USB（或其他通讯通道）
    CONTROLLER_STARTUP,                 // 正在运行启动消息和行
    CONTROLLER_READY,                   // 处于活动状态并可以使用
    CONTROLLER_PAUSED                   // 暂停 - 大概是为队列刷新做准备
} csControllerState;

typedef struct controllerSingleton {    // main TG controller struct
    magic_t magic_start;                // magic number to test memory integrity
    float null;                         // dumping ground for items with no target

    // system identification values
    float fw_build;                     // firmware build number
    float fw_version;                   // firmware version number

    // system state variables
    csControllerState controller_state;
    uint32_t led_timer;                 // used to flash indicator LED
    uint32_t led_blink_rate;            // used to flash indicator LED

    // communications state variables
    // useful to know: 
    //      cs.comm_mode is the setting for the communications mode
    //      js.json_mode is the actual current mode (see also js.json_now)
    commMode comm_mode;                 // ej: 0=text mode sticky, 1=JSON mode sticky, 2=auto mode
    commMode comm_request_mode;         // mode of request (may be different than the setting)
    bool responses_suppressed;          // if true, responses are to be suppressed (for internal-file delivery)
    
    // controller serial buffers
    char *bufp;                         // pointer to primary or secondary in buffer
    uint16_t linelen;                   // length of currently processing line
    char out_buf[OUTPUT_BUFFER_LEN];    // output buffer
    char saved_buf[SAVED_BUFFER_LEN];   // save the input buffer

    // Exceptions - some exceptions cannot be notified by an ER because they are in interrupts 
    bool exec_aline_assertion_failure;  // record an exception deep inside mp_exec_aline()

    magic_t magic_end;
} controller_t;

extern controller_t cs;                 // controller state structure

/**** function prototypes ****/

void controller_init(void);
void controller_run(void);
void controller_set_connected(bool is_connected);
void controller_set_muted(bool is_muted);
bool controller_parse_control(char *p);

#endif // End of include guard: CONTROLLER_H_ONCE
