// Compatibility handlers for pcba_* commands.
//
// These commands are query from  Anycubic go-klipper:
//   - pcba_write flag=%c data=%*s
//   - pcba_read
// and returned:
//   - pcba_data flag=%u data=%*s
//
// The original proprietary behavior is not available, so this is a dummy implementation.
// It keeps a small stateful shim that preserves the protocol surface expected by
// host-side code.
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "command.h" // DECL_COMMAND, MESSAGE_PAYLOAD_MAX, sendf

static uint8_t pcba_flag;
static uint8_t pcba_len;
static uint8_t pcba_data[MESSAGE_PAYLOAD_MAX];

void
command_pcba_write(uint32_t *args)
{
    pcba_flag = args[0];
    uint8_t len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    if (len > sizeof(pcba_data))
        len = sizeof(pcba_data);
    memcpy(pcba_data, data, len);
    pcba_len = len;
}
DECL_COMMAND(command_pcba_write, "pcba_write flag=%c data=%*s");

void
command_pcba_read(uint32_t *args)
{
    sendf("pcba_data flag=%u data=%*s", (uint32_t)pcba_flag, pcba_len, pcba_data);
}
DECL_COMMAND(command_pcba_read, "pcba_read");
