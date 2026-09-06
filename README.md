# SoapyKA9Q

`SoapyKA9Q` is an initial receive-only SoapySDR plugin for
[ka9q-radio](https://github.com/ka9q/ka9q-radio). It asks `radiod` to create a
dynamic, two-channel linear-IQ stream encoded as little-endian float32, joins
the advertised RTP multicast destination, filters on the assigned SSRC, and
returns `CF32` samples through the SoapySDR streaming API.

## Status

This is an initial interface module, intended to establish and test the API
boundary. It currently supports:

- One receive channel and one stream per device instance
- `CF32` samples
- Center frequency, output sample rate, and symmetric filter bandwidth
- IPv4 multicast control/status and RTP
- `.local` control-group resolution through the host resolver
- Optional multicast interface selection
- RTP sequence checking and timestamp-based zero filling
- Dynamic-channel cleanup when the stream is closed

It does not yet browse `_rtp._udp` services, expose radiod hardware gain
controls, provide absolute hardware time, use source-specific multicast, or
rejoin the RTP group if a controller changes the output destination.

## Build and install

Install SoapySDR and its development files, then run:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

Verify that SoapySDR sees the module:

```sh
SoapySDRUtil --info
```

## Use

The `radio` argument is the radiod control/status multicast DNS name or a
literal multicast address. As in ka9q-radio, an unqualified name gets
`.local` appended.

```sh
SoapySDRUtil --probe='driver=ka9q,radio=rx888-status.local'
```

An interface may be included in the radio string or supplied separately:

```sh
SoapySDRUtil --probe='driver=ka9q,radio=rx888-status.local,iface=en0'
```

Literal control groups are also accepted:

```sh
SoapySDRUtil --probe='driver=ka9q,radio=239.1.2.3:5006,iface=en0'
```

The standard `RADIO` environment variable works when `radio` is omitted:

```sh
export RADIO=rx888-status.local
SoapySDRUtil --probe='driver=ka9q'
```

Initial settings can be supplied in the device string:

```text
driver=ka9q,radio=rx888-status.local,
frequency=145650000,rate=24000,bandwidth=12000
```

The plugin allocates a random SSRC by default. Set `ssrc=` to make it stable.
Because an explicitly selected SSRC might belong to another controller, this
initial implementation assumes ownership of it and sets its lifetime to one
frame when the stream closes.

## Design notes

SoapySDR device support is normally a loadable plugin. Applications link only
against SoapySDR; the registry loads this module when `driver=ka9q` is
requested.

The plugin intentionally asks radiod for `F32LE`, two-channel, linear output
with AGC, envelope detection, and PLL disabled. Radiod represents I and Q as
the left and right channels of the RTP stream, respectively.

