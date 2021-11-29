# I²C Example to connect to a A5600

# Hardware: Seeeduino XIAO

This crate provides a type-safe API for working with the [Seeed Studio
Seeeduino XIAO](http://wiki.seeedstudio.com/Seeeduino-XIAO/).

## Prerequisites

- Install the cross compile toolchain `rustup target add thumbv6m-none-eabi`
- Install the [cargo-hf2 tool](https://crates.io/crates/cargo-hf2) however your
  platform requires

## Uploading the software

- Be in the project directory
- Put your device in bootloader mode by bridging the `RST` pads _twice_ in
  quick succession. The orange LED will pulse when the device is in bootloader
  mode.
- Build and upload in one step: `cargo hf2 --release`
  - Note that if you're using an older `cargo-hf2` that you'll need to specify
    the VID/PID when flashing: `cargo hf2 --vid 0x2886 --pid 0x002f --release`

Check out [the
repository](https://github.com/atsamd-rs/atsamd/tree/master/boards/xiao_m0/examples)
for examples.


## Debugging openOCD + GDB

### OpenOCD Interface

Find your debuging module in the list of interfaces in the interface folder. (e.g.: fedora 35 `/usr/share/openocd/scripts/interface/`). 

Tested with the FTDI FT232H USB module: `interface/ftdi/ft232h-module-swd.cfg`

If you don't find your module, you could create a new definition or modify an existing one to your module.

### OpenOCD target

Find your target in the targets list in the target folder. (e.g.: fedora 35 `/usr/share/openocd/scripts/target/`). 

Tested with the seeeduino XIAO cortex_m0+ module: `target/at91samdXX.cfg`

If you don't find your target, you could create a new definition or modify an existing one to your target.


### start openOCD

start openOCD with your interface and the target.

`openOCD -f <interface> -d <target>`

e.g.
```sh
> openocd -f interface/ftdi/ft232h-module-swd.cfg -f target/at91samdXX.cfg
```

your expected output looks like this:

```
○ → openocd -f interface/ftdi/ft232h-module-swd.cfg -f target/at91samdXX.cfg
Open On-Chip Debugger 0.11.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : FTDI SWD mode enabled
swd
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 400 kHz
Info : SWD DPIDR 0x0bc11477
Info : at91samd.cpu: hardware has 4 breakpoints, 2 watchpoints
Info : starting gdb server for at91samd.cpu on 3333
Info : Listening on port 3333 for gdb connections
```

If you get something else, check your wiring. I missed the second line with the 470R resistor to the SWDIO pin at the first try.

### telnet

You could use telnet to control the board.

e.g.:

- switch to bootloader mode: `reset {enter} {up} {enter}`
- halt: `halt`

### start arm gdb

You can use gdb to debug your chip. First instal the arm gdb version `arm-none-eabi-gdb`.

If you load gdb with the source including the symbols you can run step by step through your code.

There are to options. 
1. load the whole bin into gdb `arm-none-eabi-gdb target/thumbv6m-none-eabi/debug/stepper`
2. load teh symboles only `arm-none-eabi-gdb -q target/thumbv6m-none-eabi/debug/stepper`

after launching gdb, you have to connet to the target like so:

```sh
arm-none-eabi-gdb target/thumbv6m-none-eabi/debug/stepper
> target extended-remote :3333
```

### configure VS-Code
- add a config to the `.vscode/launch.json` file in the project

```
"configurations": [
    {
        "name": "Remote debug",
        "type": "gdb",
        "request": "launch",
        "cwd": "${workspaceRoot}",
        "target": "${workspaceRoot}/target/thumbv6m-none-eabi/debug/stepper",
        "gdbpath": "arm-none-eabi-gdb",
        "autorun": [
            "source -v debug.gdb",
        ]
    }
]
```    