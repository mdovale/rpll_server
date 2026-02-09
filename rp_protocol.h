#ifndef RP_PROTOCOL_H
#define RP_PROTOCOL_H

/**
 * Shared Red Pitaya server protocol constants for esw and sw.
 * Wire format and behavior are defined by the server (esw); this header
 * keeps port, frame size, and command address space in sync across clients.
 */

#define RP_DEFAULT_PORT 1001

#define RP_FFT_SIZE 513
#define RP_FRAME_SIZE_DOUBLES (2 * RP_FFT_SIZE + 16)
#define RP_FRAME_SIZE_BYTES (RP_FRAME_SIZE_DOUBLES * 8)

/**
 * Command address space: 0--34. Server process_command dispatches by
 * high byte of first word (legacy: single uint32) or by first word (new: two uint32s).
 * 0=start_measuring, 1--2=PLL reset, 3--4=freq init, 5--8=PLL gains,
 * 9--19=servo0, 20--30=servo1, 31--32=noise floor, 33--34=noise corner.
 */
#define RP_CMD_ADDR_MAX 34

/**
 * Optional partial read: counter + FFT chan1.
 * Some clients recv only this many bytes instead of full frame.
 */
#define RP_RECV_CHUNK_DOUBLES (1 + RP_FFT_SIZE)
#define RP_RECV_CHUNK_BYTES (RP_RECV_CHUNK_DOUBLES * 8)

/**
 * Capability handshake: server sends "RP_CAP:<variant>\n" on connect.
 * Client reads this before sending commands. Variant: "laser_lock" or "phasemeter".
 */
#define RP_CAP_PREFIX "RP_CAP:"
#define RP_CAP_LASER_LOCK "laser_lock"
#define RP_CAP_PHASEMETER "phasemeter"
#define RP_CAP_LINE_MAX 32

#endif
