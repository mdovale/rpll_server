# rpll server

Embedded and host-side server components for the rpll Red Pitaya laser offset locking system. The main server runs on the Red Pitaya (Zynq PS), listens for TCP clients, reads FPGA memory (PLL, servo, scope), computes FFT on scope data, and streams binary frames to the client. Commands from the client (register writes, resets) are processed and applied to the FPGA.

## Layout

| Path | Role |
|------|------|
| **esw/** | Embedded software: server binary and helpers that run on the Red Pitaya. |
| **sw/** | Optional simple C client (`client.c`) for testing: connects, sends commands, receives frames. |
| **rp_protocol.h** | Shared protocol constants (port 1001, frame size, command address space) for esw and sw. |

## Embedded software (esw)

The server in `esw/`:

- Listens on TCP port **1001** (see `rp_protocol.h`).
- Maps FPGA regions via `memory_map.c` / `memory_map.h` (AXI/BRAM).
- Reads PLL state, servo outputs, and scope buffers from the FPGA; runs FFT on scope data (real FFT 1024, in-tree implementation) and fills a double-precision frame.
- Sends one frame per client request (binary stream of doubles).
- Processes 8-byte binary commands (register address + value); see `cmd_parse.c` and `server/rp_protocol.h`.

### Build (on Red Pitaya or cross-compile)

From `server/esw/`:

```bash
make
```

Produces the `server` binary. Optional Make variables: `NFFT`, `NUMFFTS`, `DATATYPE` (e.g. `make NFFT=1800`).

Run tests (no FPGA):

```bash
make test
```

This builds and runs `test_parse_command` (command parsing) and `test_frame_layout` (frame layout constants vs client).

### Run manually

On the Red Pitaya, after loading the FPGA bitfile (e.g. `cat …/system_wrapper.bit > /dev/xdevcfg` for 125-14, or `red_pitaya_top.bit` for 250-12):

```bash
./server
```

The Python GUI client in `client/` connects to the board’s IP on port 1001.

### Debian package and systemd

From the project root, build the Debian package (see root [README](../README.md)) so that the **server** binary is installed as `/usr/bin/rpll_esw`, plus bitfiles and scripts.

- **lol-setup.service** — systemd unit: at boot it runs `/usr/bin/lol-setup`, which flashes the FPGA and starts `rpll_esw`.
- **bin/lol-setup.sh** — flashes `/usr/share/rpll/rpll.bit` and runs `rpll_esw`.
- **bin/get_temperature.sh** — reads XADC temperature and prints timestamp + value (for logging).
- **bin/start_phasemeter_sync.sh** — NTP sync, flash bitfile, start temperature logging and server with optional PLL CSV logging; used for timed/synchronized phasemeter runs.

Install paths (from `debian/rp-ll-esw.install`): `server` → `/usr/bin/rpll_esw`, bitfiles under `/usr/share/rpll/`, scripts under `/usr/bin/`.

### Documentation

Doxygen comments are in the esw sources. To build the PDF (Doxygen + LaTeX required):

```bash
cd server/esw
make doc/latex/refman.pdf
```

## Simple C client (sw)

Minimal client that connects to the server, sends a fixed command sequence, and receives frame data. Useful for quick tests without the Python GUI.

Build and run from `server/sw/`:

```bash
make
./client
```

Edit `client.c` or run with arguments if your build supports it to change target IP/port or output file. The protocol (port, frame size, command format) is defined in `../rp_protocol.h` and must match the server (esw).
