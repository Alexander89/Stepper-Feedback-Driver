# Stepper Motor driver

With I²C A5600 feedback loop running on a small scale Seeed Studio Seeeduino XIAO.

The AS5600 is used to verify that the triggered step is executed and the step motor is in the desired position.

## Features 

- 1250Hz max speed
- not inferring I²C connection to AS5600
- ramp-up to avoid overdrive
- Stuck detection to ramp-up motor again

## Config

- max speed 
- min speed (no ramp-up)
- ramp-up slope
- steps per evolution

## Todo / nice to have

- Test with 48Mhz
- Test all calculations for edge-cases
- better panic handling

## Hardware: Seeeduino XIAO

This project should (!!) work with all cortex_m0+ MPUs I used the [Seeed Studio Seeeduino XIAO](http://wiki.seeedstudio.com/Seeeduino-XIAO/) for this project.
I run it on 8Mhz you could go up to 48Mhz if you need more speed (not tested)

I used the AS5600 to read the motor position.

3D printable mount for the sensor to the motor: (coming soon, update to V3 desired)


## Development / Setup

### Prerequisites

- Install the cross compile toolchain `rustup target add thumbv6m-none-eabi`
- Install the [cargo-hf2 tool](https://crates.io/crates/cargo-hf2) however your
  platform requires

### Uploading the software

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


### Debugging manually - openOCD + GDB

#### OpenOCD Interface

Find your debuging module in the list of interfaces in the interface folder. (e.g.: fedora 35 `/usr/share/openocd/scripts/interface/`). 

Tested with the FTDI FT232H USB module: `interface/ftdi/ft232h-module-swd.cfg`

If you don't find your module, you could create a new definition or modify an existing one to your module.

#### OpenOCD target

Find your target in the targets list in the target folder. (e.g.: fedora 35 `/usr/share/openocd/scripts/target/`). 

Tested with the seeeduino XIAO cortex_m0+ module: `target/at91samdXX.cfg`

If you don't find your target, you could create a new definition or modify an existing one to your target.


#### start openOCD

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

#### telnet

You could use telnet to control the board.

e.g.:

- switch to bootloader mode: `reset {enter} {up} {enter}`
- halt: `halt`

#### start arm gdb

You can use gdb to debug your chip. First instal the arm gdb version `arm-none-eabi-gdb`.

If you load gdb with the source including the symbols you can run step by step through your code.

There are to options. 
1. load the whole bin into gdb `arm-none-eabi-gdb target/thumbv6m-none-eabi/debug/stepper`
2. load teh symboles only `arm-none-eabi-gdb -q target/thumbv6m-none-eabi/debug/stepper`

after launching gdb, you have to connet to the target like so:

```sh
> arm-none-eabi-gdb target/thumbv6m-none-eabi/debug/stepper
> target extended-remote :3333
```

### Use VS-Code extension

I used the vscode extension: marus25.cortex-debug . If openOCD and Arm-GDB is installed, it will start automatically the required services and connect to your MCU.

#### Setup:

- add a config to the `.vscode/launch.json` file in the project

```json
"configurations": [
  {
      "name": "Remote Cortex_m",
      "type": "cortex-debug",
      "request": "launch",
      "cwd": "${workspaceRoot}",
      "executable": "${workspaceRoot}/target/thumbv6m-none-eabi/debug/stepper",
      "servertype": "openocd",
      "runToMain": true,
      "gdbpath": "arm-none-eabi-gdb",
      "configFiles": [
          "interface/ftdi/ft232h-module-swd.cfg",
          "target/at91samdXX.cfg"
      ],
  }
]
```    

#### Hint:

1. If you double click the On/Off button, you could turn your board into the bootloader mode.
2. restart your debugging session after deploying a new firmware to fix the debug symbols.
3. use `--release` if you like to test the performance of your code.

## Any questions:

don't be afraid to Open an issue.
