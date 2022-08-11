<p align="center">
<img src="https://github.com/SignalHound/gr-sm/blob/master/docs/SH-GR.jpg" width="75%" />
</p>

## A [GNU Radio](https://www.gnuradio.org) module for the [Signal Hound SM Series 20/43.5 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/sm435b-43-5-ghz-real-time-spectrum-analyzer/)

### Requirements

- 64-bit Linux operating system
  - Tested on Ubuntu 20.
- Native USB 3.0 support (SM200A/B and SM435B)
- 10 GbE SFP+ network interface adapter (SM200C and SM435C)

### Prerequisites

1. [Install GNU Radio](https://wiki.gnuradio.org/index.php/InstallingGR).
    - GNU Radio 3.9 is currently used. Older versions remain as branches.
2. [Install the Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/).
    - Follow directions in _device_apis/sm_series/lib/linux/README.txt_.

### Installation

1. Clone this repository.
2. Run the following commands from the root directory of the repository:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```

### Usage

- Add the __SM: IQ Source__ block to flowgraphs in the GNU Radio Companion. It is located under the __Signal Hound__ category.
    - See _examples_ folder for demos.
- Use the block in Python with `import sm`.
- Set the `networked` parameter to `True` for SM200C and SM435C devices.
