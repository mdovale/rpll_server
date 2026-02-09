#include "../cmd_parse.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    uint32_t command[2];
    int need_8;
    size_t consumed;

    /* Legacy: addr=1 (PLL0 reset), value=0 -> single uint32 0x01000000 */
    need_8 = 0;
    uint8_t buf_legacy[] = { 0x00, 0x00, 0x00, 0x01 };
    assert(parse_one_command(buf_legacy, 4, &need_8, command, &consumed) == 1);
    assert(consumed == 4);
    assert(command[0] == 0x01000000u);
    assert(command[1] == 0u);
    assert(need_8 == 0);

    /* Legacy: addr=0, value=1 (start measuring) -> 0x00000001 */
    need_8 = 0;
    uint8_t buf_start[] = { 0x01, 0x00, 0x00, 0x00 };
    assert(parse_one_command(buf_start, 4, &need_8, command, &consumed) == 1);
    assert(consumed == 4);
    assert(command[0] == 0x00000001u);
    assert(command[1] == 1u);

    /* Extended: first word has addr in high byte, low24==0 -> need 8 bytes */
    need_8 = 0;
    uint8_t buf_ext[] = { 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00 };
    assert(parse_one_command(buf_ext, 4, &need_8, command, &consumed) == 0);
    assert(need_8 == 1);
    assert(parse_one_command(buf_ext, 8, &need_8, command, &consumed) == 1);
    assert(consumed == 8);
    assert(need_8 == 0);
    assert(command[0] == 0u);
    assert(command[1] == 2u);

    /* Need more data: only 2 bytes */
    need_8 = 0;
    uint8_t buf_short[] = { 0x01, 0x00 };
    assert(parse_one_command(buf_short, 2, &need_8, command, &consumed) == 0);

    printf("test_parse_command: all checks passed\n");
    return 0;
}
