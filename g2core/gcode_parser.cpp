/*
 * gcode_parser.cpp - rs274/ngc Gcode parser
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2018 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2018 Rob Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "g2core.h" // #1
#include "config.h" // #2
#include "controller.h"
#include "gcode.h"
#include "canonical_machine.h"
#include "settings.h"
#include "spindle.h"
#include "coolant.h"
#include "util.h"
#include "xio.h" // for char definitions

#if MARLIN_COMPAT_ENABLED == true
#include "marlin_compatibility.h"
#include "json_parser.h" // so we can switch js.comm_mode on a marlin M-code
#endif

// Helpers

static stat_t _execute_gcode_block_marlin(void);

// 本地使用的枚举

typedef enum
{                       // 用于检测gcode错误。见NIST第3.4节
    MODAL_GROUP_G0 = 0, // {G10,G28,G28.1,G92}  非模态轴指令（注1）
    MODAL_GROUP_G1,     // {G0,G1,G2,G3,G80}    运动
    MODAL_GROUP_G2,     // {G17,G18,G19}        平面选择 - XY，YZ，ZX
    MODAL_GROUP_G3,     // {G90,G91}            距离模式（绝对/增量模式）
    MODAL_GROUP_G5,     // {G93,G94}            进给速率模式
    MODAL_GROUP_G6,     // {G20,G21}            单位 - 英寸/毫米
    MODAL_GROUP_G7,     // {G40,G41,G42}        刀具半径补偿
    MODAL_GROUP_G8,     // {G43,G49}            刀具长度偏移
    MODAL_GROUP_G9,     // {G98,G99}            固定循环中返回模式
    MODAL_GROUP_G12,    // {G54,G55,G56,G57,G58,G59} 坐标系选择
    MODAL_GROUP_G13,    // {G61,G61.1,G64}      路径控制模式
    MODAL_GROUP_M4,     // {M0,M1,M2,M30,M60}   停止
    MODAL_GROUP_M6,     // {M6}                 刀具更改
    MODAL_GROUP_M7,     // {M3,M4,M5}           主轴转动
    MODAL_GROUP_M8,     // {M7,M8,M9}           冷却液（M7和M8可能在一起运行）
    MODAL_GROUP_M9      // {M48,M49}            速度/进给倍率开关
} cmModalGroup;
#define MODAL_GROUP_COUNT (MODAL_GROUP_M9 + 1)
// 注1：我们的G0省略了G4，G30，G53，G92.1，G92.2，G92.3，因为它们没有轴组件进行错误检查

/* NextAction和MotionMode（在规范机器中）之间的区别在于
* NextAction由当前块使用，并且可以携带非模态命令，而
* MotionMode在块中保持不变（如G模式组1）
*/

typedef enum
{                                         // 这些是为了优化CASE语句
    NEXT_ACTION_DEFAULT = 0,              // 必须为零（调用运动模式）
    NEXT_ACTION_DWELL,                    // G4
    NEXT_ACTION_SET_G10_DATA,             // G10
    NEXT_ACTION_GOTO_G28_POSITION,        // G28 进入机器位置
    NEXT_ACTION_SET_G28_POSITION,         // G28.1 在abs坐标中设置位置
    NEXT_ACTION_SEARCH_HOME,              // G28.2 归位周期
    NEXT_ACTION_SET_ABSOLUTE_ORIGIN,      // G28.3 原点设置
    NEXT_ACTION_HOMING_NO_SET,            // G28.4 归位循环，无坐标设置
    NEXT_ACTION_GOTO_G30_POSITION,        // G30 进入机器位置
    NEXT_ACTION_SET_G30_POSITION,         // G30.1 在abs坐标中设置位置
    NEXT_ACTION_STRAIGHT_PROBE_ERR,       // G38.2
    NEXT_ACTION_STRAIGHT_PROBE,           // G38.3
    NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR,  // G38.4
    NEXT_ACTION_STRAIGHT_PROBE_AWAY,      // G38.5
    NEXT_ACTION_SET_TL_OFFSET,            // G43
    NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET, // G43.2
    NEXT_ACTION_CANCEL_TL_OFFSET,         // G49
    NEXT_ACTION_SET_G92_OFFSETS,          // G92
    NEXT_ACTION_RESET_G92_OFFSETS,        // G92.1
    NEXT_ACTION_SUSPEND_G92_OFFSETS,      // G92.2
    NEXT_ACTION_RESUME_G92_OFFSETS,       // G92.3
    NEXT_ACTION_JSON_COMMAND_SYNC,        // M100
    NEXT_ACTION_JSON_COMMAND_ASYNC,       // M100.1
    NEXT_ACTION_JSON_WAIT,                // M101

#if MARLIN_COMPAT_ENABLED == true          // 支持马林G代码和M代码。还有E.
    NEXT_ACTION_MARLIN_TRAM_BED,           // G29
    NEXT_ACTION_MARLIN_DISABLE_MOTORS,     // M84
    NEXT_ACTION_MARLIN_SET_MT,             // M84 (with S), M85
    NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP,  // M104, M109
    NEXT_ACTION_MARLIN_PRINT_TEMPERATURES, // M105
    NEXT_ACTION_MARLIN_SET_FAN_SPEED,      // M106
    NEXT_ACTION_MARLIN_STOP_FAN,           // M107
    NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP,   // M108
    NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS, // M110
    NEXT_ACTION_MARLIN_DEBUG_STATEMENTS,   // M111 (not used)
    NEXT_ACTION_MARLIN_PRINT_POSITION,     // M114
    NEXT_ACTION_MARLIN_REPORT_VERSION,     // M115
    NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN,  // M117
    NEXT_ACTION_MARLIN_SET_BED_TEMP,       // M140, M190
#endif

} gpNextAction;

//  Gcode解析器使用的结构

typedef struct GCodeInputValue
{ // Gcode输入 - 意思取决于上下文

    gpNextAction next_action; // 处理G模态组1移动和非模态
    cmMotionMode motion_mode; // Group1: G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89
    uint8_t program_flow;     // 仅由gcode_parser使用
    uint32_t linenum;         // gcode N word

    float target[AXES];  // XYZABC应该移动的地方
    float arc_offset[3]; // IJK - 由arc命令使用
    float arc_radius;    // R word - 圆弧半径模式下的半径值
    float F_word;        // F word - F字中存在的进给率（稍后将标准化）
    float P_word;        // P word - 用于停留时间的参数，以秒为单位，G10命令
    float S_word;        // S word - 通常以RPM为单位
    uint8_t H_word;      // H word - 由G43s使用
    uint8_t L_word;      // L word - 由G10s使用

    uint8_t feed_rate_mode;     // 有关设置，请参阅cmFeedRateMode
    uint8_t select_plane;       // G17,G18,G19 - 将平面设置为的值
    uint8_t units_mode;         // G20,G21 - 0=英寸 (G20), 1 = 毫米 (G21)
    uint8_t coord_system;       // G54-G59 - 选择坐标系 1-9
    uint8_t path_control;       // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    uint8_t distance_mode;      // G91   0=使用绝对坐标(G90), 1=增量运动
    uint8_t arc_distance_mode;  // G90.1=使用绝对IJK偏移, G91.1=增量IJK偏移
    uint8_t origin_offset_mode; // G92...TRUE=原点偏移模式
    uint8_t absolute_override;  // G53 TRUE = 使用机器坐标移动 - 仅此块（G53）

    uint8_t tool;            // T和M6之后的刀具 (tool_select and tool_change)
    uint8_t tool_select;     // T值 -  T设置此值
    uint8_t tool_change;     // M6工具更改标志 - 将“tool_select”移动到“刀具”
    uint8_t coolant_mist;    // TRUE = 喷雾开启 (M7)
    uint8_t coolant_flood;   // TRUE = 喷水开启 (M8)
    uint8_t coolant_off;     // TRUE = 关闭所有冷却剂 (M9)
    uint8_t spindle_control; // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)

    bool m48_enable;  // M48/M49 输入（用于进给和主轴）
    bool fro_control; // M50 进给倍率控制
    bool tro_control; // M50.1 遍历覆盖控制
    bool spo_control; // M51 主轴速度倍率控制

#if MARLIN_COMPAT_ENABLED == true       //支持马林G代码
    float E_word;                       // E  - “挤出机” - 可以通过多种方式进行解释
    bool marlin_relative_extruder_mode; // M82, M83 (仅限马林)
#endif

} GCodeValue_t;
// Gcode输入标志
typedef struct GCodeFlags
{ // Gcode输入标志

    bool next_action;
    bool motion_mode;
    bool program_flow;
    bool linenum;

    bool target[AXES];
    bool arc_offset[3];
    bool arc_radius;

    bool F_word;
    bool P_word;
    bool S_word;
    bool H_word;
    bool L_word;

    bool feed_rate_mode;
    bool select_plane;
    bool units_mode;
    bool coord_system;
    bool path_control;
    bool distance_mode;
    bool arc_distance_mode;
    bool origin_offset_mode;
    bool absolute_override;

    bool tool;
    bool tool_select;
    bool tool_change;
    bool coolant_mist;
    bool coolant_flood;
    bool coolant_off;
    bool spindle_control;

    bool m48_enable;
    bool fro_control;
    bool tro_control;
    bool spo_control;

    bool checksum;

#if MARLIN_COMPAT_ENABLED == true
    bool E_word;
    bool marlin_wait_for_temp; // M140 or M190 - wait for temperature (Marlin-only)
    bool marlin_relative_extruder_mode;
#endif

} GCodeFlag_t;

typedef struct GCodeParser
{
    bool modals[MODAL_GROUP_COUNT];
} GCodeParser_t;

GCodeParser_t gp; // 主解析器结构
GCodeValue_t gv;  // gcode输入值
GCodeFlag_t gf;   // gcode输入标志

// 本地帮助程序函数和宏
static void _normalize_gcode_block(char *str, char **active_comment, uint8_t *block_delete_flag);
static stat_t _get_next_gcode_word(char **pstr, char *letter, float *value, int32_t *value_int);
static stat_t _point(float value);
static stat_t _verify_checksum(char *str);
static stat_t _validate_gcode_block(char *active_comment);
static stat_t _parse_gcode_block(char *line, char *active_comment); // Parse the block into the GN/GF structs
static stat_t _execute_gcode_block(char *active_comment);           // Execute the gcode block

#define SET_MODAL(m, parm, val) \
    gv.parm = val;              \
    gf.parm = true;             \
    gp.modals[m] = true;        \
    break; //xzw168
#define SET_NON_MODAL(parm, val) \
    gv.parm = val;               \
    gf.parm = true;              \
    break; //xzw168
#define EXEC_FUNC(f, v)   \
    if (gf.v)             \
    {                     \
        status = f(gv.v); \
    }

/*
 * gcode_parser_init()
 */

void gcode_parser_init()
{
    memset(&gv, 0, sizeof(GCodeValue_t));
    memset(&gf, 0, sizeof(GCodeFlag_t));
}

/*
 * gcode_parser() - 解析gcode的块（行）
 *
 *  顶级gcode解析器。规范化块并查找特殊情况
 */

stat_t gcode_parser(char *block)
{
    char *str = block; // gcode command or NUL string
    char none = NUL;
    char *active_comment = &none; // gcode comment or NUL string
    uint8_t block_delete_flag;

    stat_t check_ret = _verify_checksum(str);
    if (check_ret != STAT_OK)
    {
        return check_ret;
    }
    //将gcode的块（行）标准化
    _normalize_gcode_block(str, &active_comment, &block_delete_flag);

    // TODO，现在MSG被置于活跃评论中，处理它。

    if (str[0] == NUL)
    {                     // normalization returned null string
        return (STAT_OK); // most likely a comment line
    }

    // 陷阱M30和M2为$ clear条件。 如果不在ALARM或SHUTDOWN中，则无效
    cm_parse_clear(str);      // 如果找到M30或M2，则解析Gcode并清除警报
    ritorno(cm_is_alarmed()); // 如果处于警报，关机或恐慌状态，则返回错误状态

    // 如果第一个空格中存在/ char，则块删除省略该行
    // 目前这是无条件的，并将永远删除
    //  if ((block_delete_flag == true) && (cm_get_block_delete_switch() == true)) {
    if (block_delete_flag == true)
    {
        return (STAT_NOOP);
    }
    return (_parse_gcode_block(block, active_comment));
}

/*
 * _verify_checksum() - 如果有校验和，请确保它是有效的
 *
 * 返回STAT_OK是否有效。
 * 如果校验和不匹配，则返回STAT_CHECKSUM_MATCH_FAILED。
 */
static stat_t _verify_checksum(char *str)
{
    bool has_line_number = false; // -1表示我们没有
    if (*str == 'N')
    {
        has_line_number = true;
    }

    char checksum = 0;
    char c = *str++;
    while (c && (c != '*') && (c != '\n') && (c != '\r'))
    {
        checksum ^= c;
        c = *str++;
    }

    // c在这里可能为0，在这种情况下我们没有得到校验和，我们返回STAT_OK

    if (c == '*')
    {
        *(str - 1) = 0; // null terminate, the parser won't like this * here!
        gf.checksum = true;
        if (strtol(str, NULL, 10) != checksum)
        {
            debug_trap("checksum failure");
            return STAT_CHECKSUM_MATCH_FAILED;
        }
        if (!has_line_number)
        {
            debug_trap("line number missing with checksum");
            return STAT_MISSING_LINE_NUMBER_WITH_CHECKSUM;
        }
    }
    return STAT_OK;
}

/****************************************************************************************
 * _normalize_gcode_block() - 将gcode的块（行）标准化
 *
 *  Baseline normalization functions:
 *   - Isolate comments. See below.
 *   The rest of this applies just to the GCODE string itself (not the comments):
 *   - Remove white space, control and other invalid characters
 *   - Convert all letters to upper case
 *   - Remove (erroneous) leading zeros that might be taken to mean Octal
 *   - Signal if a block-delete character (/) was encountered in the first space
 *   - NOTE: Assumes no leading whitespace as this was removed at the controller dispatch level
 *
 *  So this: "g1 x100 Y100 f400" becomes this: "G1X100Y100F400"
 *
 *  Comment, active comment and message handling:
 *   - Comment fields start with a '(' char or alternately a semicolon ';' or percent '%'
 *   - Semicolon ';' or percent '%' end the line. All characters past are discarded
 *   - Multiple embedded comments are acceptable if '(' form
 *   - Active comments start with exactly "({" and end with "})" (no relaxing, invalid is invalid)
 *   - Active comments are moved to the end of the string
 *   - Multiple active comments are merged and moved to the end of the string
 *   - Gcode message comments (MSG) are converted to ({msg:"blah"}) active comments
 *     - The 'MSG' specifier in comment can have mixed case but cannot cannot have embedded white spaces
 *     - Only ONE MSG comment will be accepted
 *   - Other "plain" comments are discarded
 *
 *  Returns:
 *   - com points to comment string or to NUL if no comment
 *   - msg points to message string or to NUL if no comment
 *   - block_delete_flag is set true if block delete encountered, false otherwise
 */
/* Active comment notes:
 *
 *   We will convert as follows:
 *   FROM: G0 ({blah: t}) x10 (comment)
 *   TO  : g0x10\0{blah:t}
 *   NOTES: Active comments moved to the end, stripped of (), everything lowercased, and plain comment removed.
 *
 *   FROM: M100 ({a:t}) (comment) ({b:f}) (comment)
 *   TO  : m100\0{a:t,b:f}
 *   NOTES: multiple active comments merged, stripped of (), and actual comments ignored.
 */

char _normalize_scratch[RX_BUFFER_SIZE];

void _normalize_gcode_block(char *str, char **active_comment, uint8_t *block_delete_flag)
{
    _normalize_scratch[0] = 0;

    char *gc_rd = str;                // read pointer
    char *gc_wr = _normalize_scratch; // write pointer
    char *ac_rd = str;                // Active Comment read pointer
    char *ac_wr = _normalize_scratch; // Active Comment write pointer
    bool last_char_was_digit = false; // used for octal stripping

    // Move the ac_wr point forward one for every non-AC character we KEEP (plus one for a NULL in between)
    ac_wr++; // account for the in-between NULL

    // mark block deletes
    if (*gc_rd == '/')
    {
        *block_delete_flag = true;
        gc_rd++;
    }
    else
    {
        *block_delete_flag = false;
    }

    while (*gc_rd != 0)
    {
        if ((*gc_rd == ';') || (*gc_rd == '%'))
        {               // check for ';' or '%' comments that end the line
            *gc_rd = 0; // go ahead and snap the string off cleanly here
            break;
        }

        // check for comment '('
        else if (*gc_rd == '(')
        {
            // We only care if it's a "({" in order to handle string-skipping properly
            gc_rd++;
            if ((*gc_rd == '{') || (((*gc_rd == 'm') || (*gc_rd == 'M')) &&
                                    ((*(gc_rd + 1) == 's') || (*(gc_rd + 1) == 'S')) &&
                                    ((*(gc_rd + 2) == 'g') || (*(gc_rd + 2) == 'G'))))
            {
                if (ac_rd == nullptr)
                {
                    ac_rd = gc_rd; // note the start of the first AC
                }

                // skip the comment, handling strings carefully
                bool in_string = false;
                while (*(++gc_rd) != 0)
                {
                    if (*gc_rd == '"')
                    {
                        in_string = true;
                    }
                    else if (in_string)
                    {
                        if ((*gc_rd == '\\') && (*(gc_rd + 1) != 0))
                        {
                            gc_rd++; // Skip it, it's escaped.
                        }
                    }
                    else if ((*gc_rd == ')'))
                    {
                        break;
                    }
                }
                if (*gc_rd == 0)
                { // We don't want the rd++ later to skip the NULL if we're at one
                    break;
                }
            }
            else
            {
                *(gc_rd - 1) = ' '; // Change the '(' to a space to simplify the comment copy later
                while ((*gc_rd != 0) && (*gc_rd != ')'))
                { // skip ahead until we find a ')' (or NULL)
                    gc_rd++;
                }
            }
        }
        else if (!isspace(*gc_rd))
        {
            bool do_copy = false;

            // Perform Octal stripping - remove invalid leading zeros in number strings
            // Otherwise number conversions can fail, as Gcode does not support octal but C libs do
            // Change 0123.004 to 123.004, or -0234.003 to -234.003
            if (isdigit(*gc_rd) || (*gc_rd == '.'))
            { // treat '.' as a digit so we don't strip after one
                if (last_char_was_digit || (*gc_rd != '0') || !isdigit(*(gc_rd + 1)))
                {
                    do_copy = true;
                }
                last_char_was_digit = true;
            }
            else if ((isalnum((char)*gc_rd)) || (strchr("-.", *gc_rd)))
            { // all valid characters
                last_char_was_digit = false;
                do_copy = true;
            }
            if (do_copy)
            {
                *(gc_wr++) = toupper(*gc_rd);
                ac_wr++; // move the ac start position
            }
        }
        gc_rd++;
    }

    // Enforce null termination
    *gc_wr = 0;
    char *comment_start = ac_wr; // note the beginning of the comments
    if (ac_rd != nullptr)
    {

        // Now we'll copy the comments to the scratch
        while (*ac_rd != 0)
        {
            // check for comment '('
            // Remember: we're only "counting characters" at this point, no more.
            if (*ac_rd == '(')
            {
                // We only care if it's a "({" in order to handle string-skipping properly
                ac_rd++;

                bool do_copy = false;
                bool in_msg = false;
                if (((*ac_rd == 'm') || (*ac_rd == 'M')) &&
                    ((*(ac_rd + 1) == 's') || (*(ac_rd + 1) == 'S')) &&
                    ((*(ac_rd + 2) == 'g') || (*(ac_rd + 2) == 'G')))
                {

                    ac_rd += 3;
                    if (*ac_rd == ' ')
                    {
                        ac_rd++; // skip the first space.
                    }

                    if (*(ac_wr - 1) == '}')
                    {
                        *(ac_wr - 1) = ',';
                    }
                    else
                    {
                        *(ac_wr++) = '{';
                    }
                    *(ac_wr++) = 'm';
                    *(ac_wr++) = 's';
                    *(ac_wr++) = 'g';
                    *(ac_wr++) = ':';
                    *(ac_wr++) = '"';

                    // TODO - FIX BUFFER OVERFLOW POTENTIAL
                    // "(msg)" is four characters. "{msg:" is five. If the write buffer is full, we'll overflow.
                    // Also " is MSG will be quoted, making one character into two.

                    in_msg = true;
                    do_copy = true;
                }

                else if (*ac_rd == '{')
                {
                    // merge json comments
                    if (*(ac_wr - 1) == '}')
                    {
                        *(ac_wr - 1) = ',';

                        // don't copy the '{'
                        ac_rd++;
                    }

                    do_copy = true;
                }

                if (do_copy)
                {
                    // skip the comment, handling strings carefully
                    bool in_string = false;
                    bool escaped = false;
                    while (*ac_rd != 0)
                    {
                        if (in_string && (*ac_rd == '\\'))
                        {
                            escaped = true;
                        }
                        else if (!escaped && (*ac_rd == '"'))
                        {
                            // In msg comments, we have to escape "
                            if (in_msg)
                            {
                                *(ac_wr++) = '\\';
                            }
                            else
                            {
                                in_string = !in_string;
                            }
                        }
                        else if (!in_string && (*ac_rd == ')'))
                        {
                            ac_rd++;
                            if (in_msg)
                            {
                                *(ac_wr++) = '"';
                                *(ac_wr++) = '}';
                            }
                            break;
                        }
                        else
                        {
                            escaped = false;
                        }

                        // Skip spaces if we're not in a string or msg (implicit string)
                        if (in_string || in_msg || (*ac_rd != ' '))
                        {
                            *ac_wr = *ac_rd;
                            ac_wr++;
                        }

                        ac_rd++;
                    }
                }

                // We don't want the rd++ later to skip the NULL if we're at one
                if (*ac_rd == 0)
                {
                    break;
                }
            }
            ac_rd++;
        }
    }

    // Enforce null termination
    *ac_wr = 0;

    // Now copy it all back
    memcpy(str, _normalize_scratch, (ac_wr - _normalize_scratch) + 1);

    *active_comment = str + (comment_start - _normalize_scratch);
}

/****************************************************************************************
 * _get_next_gcode_word() - 获取由字母和值组成的gcode单词
 *
 *  This function requires the Gcode string to be normalized.
 *  Normalization must remove any leading zeros or they will be converted to Octal
 *  G0X... is not interpreted as hexadecimal. This is trapped.
 */

static stat_t _get_next_gcode_word(char **pstr, char *letter, float *value, int32_t *value_int)
{
    if (**pstr == NUL)
    {
        return (STAT_COMPLETE);
    } // no more words

    // get letter part
    if (isupper(**pstr) == false)
    {
        return (STAT_INVALID_OR_MALFORMED_COMMAND);
    }
    *letter = **pstr;
    (*pstr)++;

    // Removed support for hex numbers
    //    // X-axis-becomes-a-hexadecimal-number get-value case, e.g. G0X100 --> G255
    //    if ((**pstr == '0') && (*(*pstr+1) == 'X')) {
    //        *value = 0;
    //        (*pstr)++;
    //        return (STAT_OK);                           // pointer points to X
    //    }
    //
    //    // get-value general case
    //    char *end;
    //    *value_int = atol(*pstr);                       // needed to get an accurate line number for N > 8,388,608
    //    *value = strtof(*pstr, &end);

    // get-value general case
    char *end = *pstr;
    *value = c_atof(end);
    *value_int = atol(*pstr); // needed to get an accurate line number for N > 8,388,608

    if (end == *pstr)
    {
#if MARLIN_COMPAT_ENABLED == true
        if (mst.marlin_flavor)
        {
            *value = 0;
        }
        else
        {
            return (STAT_BAD_NUMBER_FORMAT);
        }
#else
        return (STAT_BAD_NUMBER_FORMAT);
#endif
    } // more robust test then checking for value=0;
    *pstr = end;
    return (STAT_OK); // pointer points to next character after the word
}

/*
 * _point() - isolate the decimal point value as an integer
 */

static uint8_t _point(const float value)
{
    return ((uint8_t)(value * 10 - trunc(value) * 10)); // isolate the decimal point as an int
}

/****************************************************************************************
 * _validate_gcode_block() - 检查一些严重的Gcode块语义违规
 */

static stat_t _validate_gcode_block(char *active_comment)
{
    //  Check for modal group violations. From NIST, section 3.4 "It is an error to put
    //  a G-code from group 1 and a G-code from group 0 on the same line if both of them
    //  use axis words. If an axis word-using G-code from group 1 is implicitly in effect
    //  on a line (by having been activated on an earlier line), and a group 0 G-code that
    //  uses axis words appears on the line, the activity of the group 1 G-code is suspended
    //  for that line. The axis word-using G-codes from group 0 are G10, G28, G30, and G92"

    //  if ((gp.modals[MODAL_GROUP_G0] == true) && (gp.modals[MODAL_GROUP_G1] == true)) {
    //     return (STAT_MODAL_GROUP_VIOLATION);
    //  }

    // look for commands that require an axis word to be present
    //  if ((gp.modals[MODAL_GROUP_G0] == true) || (gp.modals[MODAL_GROUP_G1] == true)) {
    //     if (_axis_changed() == false)
    //     return (STAT_GCODE_AXIS_IS_MISSING);
    //  }
    return (STAT_OK);
}

/****************************************************************************************
 * _parse_gcode_block() - 解析一行NULL终止的G-Code。
 *
 *  All the parser does is load the state values in gn (next model state) and set flags
 *  in gf (model state flags). The execute routine applies them. The buffer is assumed to
 *  contain only uppercase characters and signed floats (no whitespace).
 */

static stat_t _parse_gcode_block(char *buf, char *active_comment)
{
    char *pstr = (char *)buf; // persistent pointer into gcode block for parsing words
    char letter;              // parsed letter, eg.g. G or X or Y
    float value = 0;          // value parsed from letter (e.g. 2 for G2)
    int32_t value_int = 0;    // integer value parsed from letter - needed for line numbers
    stat_t status = STAT_OK;

    // set initial state for new move
    memset(&gv, 0, sizeof(GCodeValue_t));       // clear all next-state values
    memset(&gf, 0, sizeof(GCodeFlag_t));        // clear all next-state flags
    gv.motion_mode = cm_get_motion_mode(MODEL); // get motion mode from previous block

    // Causes a later exception if
    //  (1) INVERSE_TIME_MODE is active and a feed rate is not provided or
    //  (2) INVERSE_TIME_MODE is changed to UNITS_PER_MINUTE and a new feed rate is missing
    if (cm->gm.feed_rate_mode == INVERSE_TIME_MODE)
    { // new feed rate required when in INV_TIME_MODE
        gv.F_word = 0;
        gf.F_word = true;
    }

    // extract commands and parameters
    while ((status = _get_next_gcode_word(&pstr, &letter, &value, &value_int)) == STAT_OK)
    {
        switch (letter)
        {
        case 'G':
            switch ((uint8_t)value)
            {
            case 0:
                SET_MODAL(MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_TRAVERSE);
            case 1:
                SET_MODAL(MODAL_GROUP_G1, motion_mode, MOTION_MODE_STRAIGHT_FEED);
            case 2:
                SET_MODAL(MODAL_GROUP_G1, motion_mode, MOTION_MODE_CW_ARC);
            case 3:
                SET_MODAL(MODAL_GROUP_G1, motion_mode, MOTION_MODE_CCW_ARC);
            case 4:
                SET_NON_MODAL(next_action, NEXT_ACTION_DWELL);
            case 10:
                SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G10_DATA);
            case 17:
                SET_MODAL(MODAL_GROUP_G2, select_plane, CANON_PLANE_XY);
            case 18:
                SET_MODAL(MODAL_GROUP_G2, select_plane, CANON_PLANE_XZ);
            case 19:
                SET_MODAL(MODAL_GROUP_G2, select_plane, CANON_PLANE_YZ);
            case 20:
                SET_MODAL(MODAL_GROUP_G6, units_mode, INCHES);
            case 21:
                SET_MODAL(MODAL_GROUP_G6, units_mode, MILLIMETERS);
            case 28:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G28_POSITION);
                case 1:
                    SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G28_POSITION);
                case 2:
                    SET_NON_MODAL(next_action, NEXT_ACTION_SEARCH_HOME);
                case 3:
                    SET_NON_MODAL(next_action, NEXT_ACTION_SET_ABSOLUTE_ORIGIN);
                case 4:
                    SET_NON_MODAL(next_action, NEXT_ACTION_HOMING_NO_SET);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
#if MARLIN_COMPAT_ENABLED == true
            case 29:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_TRAM_BED);
#endif
            case 30:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_GOTO_G30_POSITION);
                case 1:
                    SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G30_POSITION);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 38:
            {
                switch (_point(value))
                {
                case 2:
                    SET_NON_MODAL(next_action, NEXT_ACTION_STRAIGHT_PROBE_ERR);
                case 3:
                    SET_NON_MODAL(next_action, NEXT_ACTION_STRAIGHT_PROBE);
                case 4:
                    SET_NON_MODAL(next_action, NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR);
                case 5:
                    SET_NON_MODAL(next_action, NEXT_ACTION_STRAIGHT_PROBE_AWAY);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 40:
                break; // ignore cancel cutter radius compensation. But don't fail G40s.
            case 43:
            {
                switch (_point(value))
                {
                case 0:
                    SET_NON_MODAL(next_action, NEXT_ACTION_SET_TL_OFFSET);
                case 2:
                    SET_NON_MODAL(next_action, NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 49:
                SET_NON_MODAL(next_action, NEXT_ACTION_CANCEL_TL_OFFSET);
            case 53:
                SET_NON_MODAL(absolute_override, ABSOLUTE_OVERRIDE_ON_DISPLAY_WITH_NO_OFFSETS);
            case 54:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G54);
            case 55:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G55);
            case 56:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G56);
            case 57:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G57);
            case 58:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G58);
            case 59:
                SET_MODAL(MODAL_GROUP_G12, coord_system, G59);
            case 61:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G13, path_control, PATH_EXACT_PATH);
                case 1:
                    SET_MODAL(MODAL_GROUP_G13, path_control, PATH_EXACT_STOP);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 64:
                SET_MODAL(MODAL_GROUP_G13, path_control, PATH_CONTINUOUS);
            case 80:
                SET_MODAL(MODAL_GROUP_G1, motion_mode, MOTION_MODE_CANCEL_MOTION_MODE);
            case 90:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G3, distance_mode, ABSOLUTE_DISTANCE_MODE);
                case 1:
                    SET_MODAL(MODAL_GROUP_G3, arc_distance_mode, ABSOLUTE_DISTANCE_MODE);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 91:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G3, distance_mode, INCREMENTAL_DISTANCE_MODE);
                case 1:
                    SET_MODAL(MODAL_GROUP_G3, arc_distance_mode, INCREMENTAL_DISTANCE_MODE);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 92:
            {
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_G0, next_action, NEXT_ACTION_SET_G92_OFFSETS);
                case 1:
                    SET_NON_MODAL(next_action, NEXT_ACTION_RESET_G92_OFFSETS);
                case 2:
                    SET_NON_MODAL(next_action, NEXT_ACTION_SUSPEND_G92_OFFSETS);
                case 3:
                    SET_NON_MODAL(next_action, NEXT_ACTION_RESUME_G92_OFFSETS);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            }
            case 93:
                SET_MODAL(MODAL_GROUP_G5, feed_rate_mode, INVERSE_TIME_MODE);
            case 94:
                SET_MODAL(MODAL_GROUP_G5, feed_rate_mode, UNITS_PER_MINUTE_MODE);
                //              case 95: SET_MODAL (MODAL_GROUP_G5, feed_rate_mode, UNITS_PER_REVOLUTION_MODE);

            default:
                status = STAT_GCODE_COMMAND_UNSUPPORTED;
            }
            break;

        case 'M':
            switch ((uint8_t)value)
            {
            case 0:
            case 1:
            case 60:
                SET_MODAL(MODAL_GROUP_M4, program_flow, PROGRAM_STOP);
            case 2:
            case 30:
                SET_MODAL(MODAL_GROUP_M4, program_flow, PROGRAM_END);
            case 3:
                SET_MODAL(MODAL_GROUP_M7, spindle_control, SPINDLE_CW);
            case 4:
                SET_MODAL(MODAL_GROUP_M7, spindle_control, SPINDLE_CCW);
            case 5:
                SET_MODAL(MODAL_GROUP_M7, spindle_control, SPINDLE_OFF);
            case 6:
                SET_NON_MODAL(tool_change, true);
            case 7:
                SET_MODAL(MODAL_GROUP_M8, coolant_mist, COOLANT_ON);
            case 8:
                SET_MODAL(MODAL_GROUP_M8, coolant_flood, COOLANT_ON);
            case 9:
                SET_MODAL(MODAL_GROUP_M8, coolant_off, COOLANT_OFF);
            case 48:
                SET_MODAL(MODAL_GROUP_M9, m48_enable, true);
            case 49:
                SET_MODAL(MODAL_GROUP_M9, m48_enable, false);
            case 50:
                switch (_point(value))
                {
                case 0:
                    SET_MODAL(MODAL_GROUP_M9, fro_control, true);
                case 1:
                    SET_MODAL(MODAL_GROUP_M9, tro_control, true);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            case 51:
                SET_MODAL(MODAL_GROUP_M9, spo_control, true);
            case 100:
                switch (_point(value))
                {
                case 0:
                    SET_NON_MODAL(next_action, NEXT_ACTION_JSON_COMMAND_SYNC);
                case 1:
                    SET_NON_MODAL(next_action, NEXT_ACTION_JSON_COMMAND_ASYNC);
                default:
                    status = STAT_GCODE_COMMAND_UNSUPPORTED;
                }
                break;
            case 101:
                SET_NON_MODAL(next_action, NEXT_ACTION_JSON_WAIT);

#if MARLIN_COMPAT_ENABLED == true // Note: case ordering and presence/absence of break;s is very important
            case 20:
                marlin_list_sd_response();
                status = STAT_COMPLETE;
                break; // List SD card
            case 21:   // Initialize SD card
            case 22:
                status = STAT_COMPLETE;
                break; // Release SD card
            case 23:
                marlin_select_sd_response(pstr);
                status = STAT_COMPLETE;
                break; // Select SD file

            case 82:
                SET_NON_MODAL(marlin_relative_extruder_mode, false); // set relative extruder mode off
            case 83:
                SET_NON_MODAL(marlin_relative_extruder_mode, true); // set relative extruder mode on

            case 18: // compatibility alias for M84
            case 84:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_DISABLE_MOTORS); // disable all motors
            case 85:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_SET_MT); // set motor timeout

            case 105:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_PRINT_TEMPERATURES); // request temperature report
            case 106:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_SET_FAN_SPEED); // set fan speed range 0 - 255
            case 107:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_STOP_FAN); // stop fan (speed = 0)
            case 108:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP); // cancel wait for temperature
            case 114:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_PRINT_POSITION); // request position report

            case 109:
                gf.marlin_wait_for_temp = true; // NO break!       // set wait for temp and execute M104
            case 104:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP); // set extruder temperature

            case 190:
                gf.marlin_wait_for_temp = true; // NO break!       // set wait for temp and execute M140
            case 140:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_SET_BED_TEMP); // set heated bed temperature

            case 110:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS); // reset line numbers
            case 111:
                status = STAT_COMPLETE;
                break; // ignore M111 Marlin debug statements. Don't process contents of the line further

            case 115:
                SET_NON_MODAL(next_action, NEXT_ACTION_MARLIN_REPORT_VERSION); // report version information
            case 117:
                status = STAT_COMPLETE;
                break; //SET_NON_MODAL (next_action, NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN);
#endif                 // MARLIN_COMPAT_ENABLED

            default:
                status = STAT_MCODE_COMMAND_UNSUPPORTED;
            }
            break;

        case 'T':
            SET_NON_MODAL(tool_select, (uint8_t)trunc(value));
        case 'F':
            SET_NON_MODAL(F_word, value);
        case 'P':
            SET_NON_MODAL(P_word, value); // used for dwell time, G10 coord select
        case 'S':
            SET_NON_MODAL(S_word, value);
        case 'X':
            SET_NON_MODAL(target[AXIS_X], value);
        case 'Y':
            SET_NON_MODAL(target[AXIS_Y], value);
        case 'Z':
            SET_NON_MODAL(target[AXIS_Z], value);
        case 'A':
            SET_NON_MODAL(target[AXIS_A], value);
        case 'B':
            SET_NON_MODAL(target[AXIS_B], value);
        case 'C':
            SET_NON_MODAL(target[AXIS_C], value);
        case 'U':
            SET_NON_MODAL(target[AXIS_U], value);
        case 'V':
            SET_NON_MODAL(target[AXIS_V], value);
        case 'W':
            SET_NON_MODAL(target[AXIS_W], value);
        case 'H':
            SET_NON_MODAL(H_word, value);
        case 'I':
            SET_NON_MODAL(arc_offset[0], value);
        case 'J':
            SET_NON_MODAL(arc_offset[1], value);
        case 'K':
            SET_NON_MODAL(arc_offset[2], value);
        case 'L':
            SET_NON_MODAL(L_word, value);
        case 'R':
            SET_NON_MODAL(arc_radius, value);
        case 'N':
            SET_NON_MODAL(linenum, value_int); // line number handled as special case to preserve integer value

#if MARLIN_COMPAT_ENABLED == true
        case 'E':
            SET_NON_MODAL(E_word, value); // extruder value
#endif
        default:
            status = STAT_GCODE_COMMAND_UNSUPPORTED;
        }
        if (status != STAT_OK)
            break;
    }
    if ((status != STAT_OK) && (status != STAT_COMPLETE))
        return (status);
	//#define ritorno(a) if((status_code=a) != STAT_OK) { return(status_code); }
    ritorno(_validate_gcode_block(active_comment)); //检查一些严重的Gcode块语义违规
    return (_execute_gcode_block(active_comment));  // if successful execute the block
}

/****************************************************************************************
 * _execute_gcode_block（） - 执行解析块
 *
 *有条件地（根据是否在gf中设置标志）调用规范加工
*按执行顺序执行。源自RS274NGC_3表8：
 *
 * 0.记录行号
 * 1.评论（包括消息）[在块规范化期间处理]
* 1a。启用或禁用覆盖（M48，M49）
* 1b。设定进给倍率（M50）
* 1c。设置遍历覆盖率（M50.1）
* 1d。设定主轴倍率（M51）
 * 2.设定进给速率模式（G93，G94  - 反时限或每分钟）
 * 3.设定进给速度（F）
* 3a。马林功能（可选）
* 3b。设定进给倍率（M50.1）
* 3c。设置导线倍率（M50.2）
 * 4.设定主轴转速（S）
 * 5.选择工具（T）
 * 6.更换工具（M6）
 * 7.主轴开启或关闭（M3，M4，M5）
 * 8.冷却液开启或关闭（M7，M8，M9）
 * // 9.启用或禁用覆盖（M48，M49，M50，M51）（参见1a）
 * 10.居住（G4）
 * 11.设置有效平面（G17，G18，G19）
 * 12.设定长度单位（G20，G21）
 * 13.开启或关闭刀具半径补偿（G40，G41，G42）
 * 14.开关长度补偿（G43，G49）
 * 15.坐标系选择（G54，G55，G56，G57，G58，G59）
 * 16.设定路径控制模式（G61，G61.1，G64）
 * 17.设定距离模式（G90，G91）
* 17a。设定弧距模式（G90.1，G91.1）
 * 18.设置缩进模式（G98，G99）
* 19a。归位功能（G28.2，G28.3，G28.1，G28，G30）
* 19b。更新系统数据（G10）
* 19c。设置轴偏移（G92，G92.1，G92.2，G92.3）
 * 20.执行G53（可能）修改的动作（G0至G3，G80-G89）
 * 21.停止和结束（M0，M1，M2，M30，M60）
 *
 * gv中的值是原始单位，不应事先进行单位转换
 *调用规范函数（执行单位转换）
 */

stat_t _execute_gcode_block(char *active_comment)
{
    stat_t status = STAT_OK;

    cm_cycle_start(); // 任何G，M或其他单词将自动启动循环，如果尚未启动

    if (gf.linenum)
    {
        cm_set_model_linenum(gv.linenum);
    }

    EXEC_FUNC(cm_m48_enable, m48_enable);

    if (gf.fro_control)
    { // feedrate override
        ritorno(cm_fro_control(gv.P_word, gf.P_word));
    }
    if (gf.tro_control)
    { // traverse override
        ritorno(cm_tro_control(gv.P_word, gf.P_word));
    }
    if (gf.spo_control)
    { // spindle speed override
        ritorno(spindle_override_control(gv.P_word, gf.P_word));
    }
    //#define EXEC_FUNC(f,v) if(gf.v) { status=f(gv.v);}
    EXEC_FUNC(cm_set_feed_rate_mode, feed_rate_mode); // G93, G94
    EXEC_FUNC(cm_set_feed_rate, F_word);              // F

    ritorno(_execute_gcode_block_marlin()); // 如果启用Marlin兼容性，则执行Marlin命令
    if (gf.linenum && gf.checksum)
    {
        ritorno(cm_check_linenum());
    }

    EXEC_FUNC(spindle_speed_sync, S_word);  // S
    EXEC_FUNC(cm_select_tool, tool_select); // T - tool_select is where it's written
    EXEC_FUNC(cm_change_tool, tool_change); // M6 - is where it's effected

    if (gf.spindle_control)
    { // M3, M4, M5 (spindle OFF, CW, CCW)
        ritorno(spindle_control_sync((spControl)gv.spindle_control));
    }
    if (gf.coolant_mist)
    {
        ritorno(coolant_control_sync((coControl)gv.coolant_mist, COOLANT_MIST)); // M7
    }
    if (gf.coolant_flood)
    {
        ritorno(coolant_control_sync((coControl)gv.coolant_flood, COOLANT_FLOOD)); // M8
    }
    if (gf.coolant_off)
    {
        ritorno(coolant_control_sync((coControl)gv.coolant_off, COOLANT_BOTH)); // M9
    }
    if (gv.next_action == NEXT_ACTION_DWELL)
    {                                 // G4 - dwell
        ritorno(cm_dwell(gv.P_word)); // return if error, otherwise complete the block
    }
    EXEC_FUNC(cm_select_plane, select_plane); // G17, G18, G19
    EXEC_FUNC(cm_set_units_mode, units_mode); // G20, G21
    //--> cutter radius compensation goes here

    switch (gv.next_action)
    { // Tool length offsets
    case NEXT_ACTION_SET_TL_OFFSET:
    { // G43
        ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, false));
        break;
    }
    case NEXT_ACTION_SET_ADDITIONAL_TL_OFFSET:
    { // G43.2
        ritorno(cm_set_tl_offset(gv.H_word, gf.H_word, true));
        break;
    }
    case NEXT_ACTION_CANCEL_TL_OFFSET:
    { // G49
        ritorno(cm_cancel_tl_offset());
        break;
    }
    default:
    {
    } // quiet the compiler warning about all the things we don't handle here
    }

    EXEC_FUNC(cm_set_coord_system, coord_system); // G54, G55, G56, G57, G58, G59

    if (gf.path_control)
    { // G61, G61.1, G64
        status = cm_set_path_control(MODEL, gv.path_control);
    }

    EXEC_FUNC(cm_set_distance_mode, distance_mode);         // G90, G91
    EXEC_FUNC(cm_set_arc_distance_mode, arc_distance_mode); // G90.1, G91.1
    //--> set retract mode goes here

    switch (gv.next_action)
    {
    case NEXT_ACTION_SET_G28_POSITION:
    {
        status = cm_set_g28_position();
        break;
    } // G28.1
    case NEXT_ACTION_GOTO_G28_POSITION:
    {
        status = cm_goto_g28_position(gv.target, gf.target);
        break;
    } // G28
    case NEXT_ACTION_SET_G30_POSITION:
    {
        status = cm_set_g30_position();
        break;
    } // G30.1
    case NEXT_ACTION_GOTO_G30_POSITION:
    {
        status = cm_goto_g30_position(gv.target, gf.target);
        break;
    } // G30

    case NEXT_ACTION_SEARCH_HOME:
    {
        status = cm_homing_cycle_start(gv.target, gf.target);
        break;
    } // G28.2
    case NEXT_ACTION_SET_ABSOLUTE_ORIGIN:
    {
        status = cm_set_absolute_origin(gv.target, gf.target);
        break;
    } // G28.3
    case NEXT_ACTION_HOMING_NO_SET:
    {
        status = cm_homing_cycle_start_no_set(gv.target, gf.target);
        break;
    } // G28.4

    case NEXT_ACTION_STRAIGHT_PROBE_ERR:
    {
        status = cm_straight_probe(gv.target, gf.target, true, true);
        break;
    } // G38.2
    case NEXT_ACTION_STRAIGHT_PROBE:
    {
        status = cm_straight_probe(gv.target, gf.target, true, false);
        break;
    } // G38.3
    case NEXT_ACTION_STRAIGHT_PROBE_AWAY_ERR:
    {
        status = cm_straight_probe(gv.target, gf.target, false, true);
        break;
    } // G38.4
    case NEXT_ACTION_STRAIGHT_PROBE_AWAY:
    {
        status = cm_straight_probe(gv.target, gf.target, false, false);
        break;
    } // G38.5

    case NEXT_ACTION_SET_G10_DATA:
    {
        status = cm_set_g10_data(gv.P_word, gf.P_word, // G10
                                 gv.L_word, gf.L_word,
                                 gv.target, gf.target);
        break;
    }

    case NEXT_ACTION_SET_G92_OFFSETS:
    {
        status = cm_set_g92_offsets(gv.target, gf.target);
        break;
    } // G92
    case NEXT_ACTION_RESET_G92_OFFSETS:
    {
        status = cm_reset_g92_offsets();
        break;
    } // G92.1
    case NEXT_ACTION_SUSPEND_G92_OFFSETS:
    {
        status = cm_suspend_g92_offsets();
        break;
    } // G92.2
    case NEXT_ACTION_RESUME_G92_OFFSETS:
    {
        status = cm_resume_g92_offsets();
        break;
    } // G92.3

    case NEXT_ACTION_JSON_COMMAND_SYNC:
    {
        status = cm_json_command(active_comment);
        break;
    } // M100.0
    case NEXT_ACTION_JSON_COMMAND_ASYNC:
    {
        status = cm_json_command_immediate(active_comment);
        break;
    } // M100.1
    case NEXT_ACTION_JSON_WAIT:
    {
        status = cm_json_wait(active_comment);
        break;
    } // M101

    case NEXT_ACTION_DEFAULT:
    {
        cm_set_absolute_override(MODEL, gv.absolute_override); // 应用绝对覆盖并显示为绝对值
        switch (gv.motion_mode)
        {
        case MOTION_MODE_CANCEL_MOTION_MODE:// G80
        {
            cm->gm.motion_mode = gv.motion_mode;
            break;
        } 
        case MOTION_MODE_STRAIGHT_TRAVERSE:// G0
        {
            status = cm_straight_traverse(gv.target, gf.target, PROFILE_NORMAL);
            break;
        } 
        case MOTION_MODE_STRAIGHT_FEED:// G1
        {
            status = cm_straight_feed(gv.target, gf.target, PROFILE_NORMAL);
            break;
        }                        
        case MOTION_MODE_CW_ARC: // G2
        case MOTION_MODE_CCW_ARC:// G3
        {
            status = cm_arc_feed(gv.target, gf.target, 
                                 gv.arc_offset, gf.arc_offset,
                                 gv.arc_radius, gf.arc_radius,
                                 gv.P_word, gf.P_word,
                                 gp.modals[MODAL_GROUP_G1],
                                 gv.motion_mode);
            break;
        }
        default:
            break;
        }
        cm_set_absolute_override(MODEL, ABSOLUTE_OVERRIDE_OFF); // un-set absolute override once the move is planned
    }
    default:
        // quiet the compiler warning about all the things we don't handle
        break;
    }

    // do the program stops and ends : M0, M1, M2, M30, M60
    if (gf.program_flow == true)
    {
        if (gv.program_flow == PROGRAM_STOP)
        {
            cm_program_stop();
        }
        else
        {
            cm_program_end();
        }
    }
    return (status);
}

/***********************************************************************************
 * _execute_gcode_block_marlin() - collect Marlin Gcode execution functions here
 */

static stat_t _execute_gcode_block_marlin()
{
#if MARLIN_COMPAT_ENABLED == true
    // Check for sequential line numbers
    if (gf.linenum && gf.checksum)
    {
        if (gv.next_action != NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS)
        {
            ritorno(cm_check_linenum());
        }
        cm->gmx.last_line_number = cm->gm.linenum;

        // since this is handled, clear gf.checksum so it doesn't again
        gf.checksum = false;
    }

    // E should ONLY be seen in marlin flavor
    if (gf.E_word)
    {
        mst.marlin_flavor = true;
    }

    // adjust T real quick
    if (mst.marlin_flavor && gf.tool_select)
    {
        gv.tool_select += 1;
        cm->gm.tool_select = gv.tool_select; // We need to go ahead and apply to tool select, and in Marlin 0 is valid, so add 1
        cm->gm.tool = cm->gm.tool_select;    // Also, in Marlin, tool changes are effective immediately :facepalm:
        gf.tool_select = false;              // prevent a tool_select command from being buffered (planning to zero)
    }
    else if (cm->gm.tool_select == 0)
    {
        cm->gm.tool_select = 1;           // We need to ensure we have a valid tool selected, often Marlin gcode won't have a T word at all
        cm->gm.tool = cm->gm.tool_select; // Also, in Marlin, tool changes are effective immediately :facepalm:
    }

    // Deal with E
    if (gf.marlin_relative_extruder_mode)
    { // M82, M83
        marlin_set_extruder_mode(gv.marlin_relative_extruder_mode);
    }
    if (gf.E_word)
    {
        // Ennn T0 -> Annn
        if (cm->gm.tool_select == 1)
        {
            gf.target[AXIS_A] = true;
            gv.target[AXIS_A] = gv.E_word;
        }
        // Ennn T1 -> Bnnn
        else if (cm->gm.tool_select == 2)
        {
            gf.target[AXIS_B] = true;
            gv.target[AXIS_B] = gv.E_word;
        }
        else
        {
            debug_trap("invalid tool selection");
            return (STAT_INPUT_VALUE_RANGE_ERROR);
        }
    }

    if ((mst.marlin_flavor || (MARLIN_COMM_MODE == js.json_mode)) &&
        (NEXT_ACTION_GOTO_G28_POSITION == gv.next_action))
    {
        gv.next_action = NEXT_ACTION_SEARCH_HOME;
    }

    switch (gv.next_action)
    {
    case NEXT_ACTION_MARLIN_PRINT_TEMPERATURES:
    {                                    // M105
        js.json_mode = MARLIN_COMM_MODE; // we use M105 to know when to switch
        ritorno(marlin_request_temperature_report());
        break;
    }
    case NEXT_ACTION_MARLIN_PRINT_POSITION:
    {                                    // M114
        js.json_mode = MARLIN_COMM_MODE; // we use M105 to know when to switch
        ritorno(marlin_request_position_report());
        break;
    }
    case NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP: // M104 or M109
    case NEXT_ACTION_MARLIN_SET_BED_TEMP:
    {                             // M140 or M190
        mst.marlin_flavor = true; // these gcodes are ONLY in marlin flavor
        float temp = 0;
        if (gf.S_word)
        {
            temp = gv.S_word;
        }
        if (gf.P_word)
        {
            temp = gv.P_word;
        } // we treat them the same, for now

        uint8_t tool = (gv.next_action == NEXT_ACTION_MARLIN_SET_EXTRUDER_TEMP) ? cm->gm.tool_select : 3;
        ritorno(marlin_set_temperature(tool, temp, gf.marlin_wait_for_temp));

        gf.P_word = false;
        gf.S_word = false;
        break;
    }
    case NEXT_ACTION_MARLIN_CANCEL_WAIT_TEMP:
    {                                    // M108
        js.json_mode = MARLIN_COMM_MODE; // we use M105 to know when to switch
        cm_request_feedhold(FEEDHOLD_TYPE_HOLD, FEEDHOLD_EXIT_STOP);
        cm_request_queue_flush();
        break;
    }
    case NEXT_ACTION_MARLIN_TRAM_BED:
    {                             // G29
        mst.marlin_flavor = true; // these gcodes are ONLY in marlin flavor
        ritorno(marlin_start_tramming_bed());
        break;
    }
    case NEXT_ACTION_MARLIN_SET_FAN_SPEED:
    {                             // M106
        mst.marlin_flavor = true; // these gcodes are ONLY in marlin flavor
        ritorno(marlin_set_fan_speed(
            gf.P_word ? gv.P_word : 0,
            gf.S_word ? gv.S_word : 0));
        gf.P_word = false;
        gf.S_word = false;
        break;
    }
    case NEXT_ACTION_MARLIN_STOP_FAN:
    {                             // M107
        mst.marlin_flavor = true; // these gcodes are ONLY in marlin flavor
        ritorno(marlin_set_fan_speed(gf.P_word ? gv.P_word : 0, 0));
        gf.P_word = false;
        gf.S_word = false;
        break;
    }
    // adjust G28 (already adjusted to G28.2)
    // with no X, Y, or Z, we assume all three
    case NEXT_ACTION_SEARCH_HOME:
    { // G28 (g2core G28.2)
        if (!gf.target[AXIS_X] && !gf.target[AXIS_Y] && !gf.target[AXIS_Z])
        {
            gv.target[AXIS_X] = 0.0;
            gf.target[AXIS_X] = true;
            gv.target[AXIS_Y] = 0.0;
            gf.target[AXIS_Y] = true;
            gv.target[AXIS_Z] = 0.0;
            gf.target[AXIS_Z] = true;
        }
        break;
    }
    case NEXT_ACTION_MARLIN_DISABLE_MOTORS:
    { // M84 and M18
        if (gf.S_word)
        {
            ritorno(marlin_set_motor_timeout(gv.S_word));
            gf.S_word = false;
        }
        else
        {
            ritorno(marlin_disable_motors());
        }
        break;
    }
    case NEXT_ACTION_MARLIN_SET_MT:
    { // M85
        if (gf.S_word)
        {
            ritorno(marlin_set_motor_timeout(gv.S_word));
            gf.S_word = false;
        }
        else
        {
            return (STAT_OK); // this means nothing, but it's not an error
        }
    }
    case NEXT_ACTION_MARLIN_DISPLAY_ON_SCREEN:
    {                     // M117
        return (STAT_OK); // ignore for now
    }
    case NEXT_ACTION_MARLIN_REPORT_VERSION:
    { // M115
        js.json_mode = MARLIN_COMM_MODE;
        ritorno(marlin_report_version());
        break;
    }
    case NEXT_ACTION_MARLIN_RESET_LINE_NUMBERS:
    {                                    // M110
        js.json_mode = MARLIN_COMM_MODE; // we already did this above in if (gf.linenum) {}
        return (STAT_OK);
    }
    case NEXT_ACTION_DEFAULT:
    {
        if (mst.marlin_flavor)
        {
            if (gf.motion_mode)
            { // adjust G0 to almost always be the same as G1
                if (gf.E_word && (!gf.target[AXIS_X] && !gf.target[AXIS_Y] && !gf.target[AXIS_Z]))
                {
                    gv.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE; // G0
                }
                else
                {
                    gv.motion_mode = MOTION_MODE_STRAIGHT_FEED; // G1
                }
            }
        }
        break;
    }
    default:
        // quiet the compiler warning about all the things we don't handle
        break;
    } // switch (gv.next_action)

#endif // MARLIN_COMPAT_ENABLED

    return (STAT_OK);
}

/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

stat_t gc_get_gc(nvObj_t *nv)
{
    ritorno(nv_copy_string(nv, cs.saved_buf));
    nv->valuetype = TYPE_STRING;
    return (STAT_OK);
}

stat_t gc_run_gc(nvObj_t *nv)
{
    return (gcode_parser(*nv->stringp));
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

// no text mode functions here. Move along

#endif // __TEXT_MODE
