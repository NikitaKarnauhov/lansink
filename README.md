LANSink
=======

LANSink is simple unreliable half-duplex audio transport over UDP implemented
as an ALSA plugin on sender side and as a stand-alone daemon on receiver side.
It currently performs no audio compression or latency control. It is designed
as a means to connect HTPC with other computers on the same network.

## Installation

### Runtime requirements

* protobuf
* alsa

### Build requirements

* g++
* cmake >= 2.8.5
* pkg-config
* alsa-lib
* git (for getting the source code)

Under Fedora you can install required packages using:

```
$ sudo yum install cmake gcc-c++ alsa-lib-devel protobuf-devel git
```

### Installing from source

Clone git repository:

```
$ git clone https://github.com/NikitaKarnauhov/lansink.git
```

Build:

```
$ cd lansink
$ mkdir Release
$ cd Release
$ cmake CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr ..
$ make
$ sudo make install
```

This installs both ALSA plugin and receiver daemon. You will likely need to
repeat intallation both on sender and on receiver side.

## Configuring sender

Audio sender is implemented as an ALSA plugin `lansink`, see `asoundrc.sample`
in the source tree for annotated sample configuration (copy it's contents into
your $HOME/.asoundrc). Only `type` parameter is strictly required but you
probably should also specify `host` to point to the receiver machine.

You can test configuration with `speaker-test` program from `alsa-utils`
package:

```
$ speaker-test -c 2 -b 20000 -D lansink
```

It shouldn't produce any errors though no sound will actually appear until
receiver daemon is set up.

## Configuring reveiver

If you followed this instructions receiver daemon can be launched with
'lansinkd' command. To test the installation run:

```
$ src/lansinkd -L3 -n
```

This command launches receiver attached to the terminal with log level set to
`info`. The output should be approximately like the following:

```
$ lansinkd -L3 -n
2013-08-27T21:20:58+0700: INFO: Logging to /tmp/lansinkd.log
2013-08-27T21:20:58+0700: INFO: PID file written to /tmp/lansinkd.pid
2013-08-27T21:20:58+0700: INFO: Server listening on 0.0.0.0:26751
```

On incoming connections ALSA device `default` will be opened. To override this
use `-D` command line argument, e.g. run `lansinkd -D hw:0,0` to use hardware
output. When using plain sound card (without PulseAudio/dmix layer) that
doesn't support mixing use `-x` parameter to prevent 'Device or resource busy'
errors on multiple simultaneous connections. See `lansinkd.conf.sample` file
for annotated configuration. You can specify custom config with `-c` argument.

You may repeat running `speaker-test` on the sender machine. You will hear
noise if everything is set up properly.

## Using PulseAudio on sender side

For ease of switching outputs you can use LANSink from PulseAudio with
`alsa-sink` module. Add following line to the PulseAudio configuration
($HOME/.config/pulse/default.pa or /etc/pulse/default.pa):

```
load-module module-alsa-sink device=lansink sink_name=lansink tsched=0 fragment_size=6144 fragments=1
```

`fragment_size` and `fragments` parameters are not required but can help when
playing audo from network, e.g. with Flash Player from your browser.

## Using PulseAudio on receiver side

PulseAudio can also be used on receiver in order to mix streams and convert
audio formats to ones supported by hardware. You will need to install 'pulse'
plugin for ALSA. On Fedora run:

```
$ sudo yum install alsa-plugins-pulseaudio
```

It will add `pulse` alsa device that can then be passed to `lansinkd` with
`-D` argument.
