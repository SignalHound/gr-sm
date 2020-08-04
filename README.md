<p align="center">
<img src="https://github.com/SignalHound/gr-sm200/blob/master/docs/SH-GR.jpg" width="75%" />
</p>

## A [GNU Radio](https://www.gnuradio.org) module for the [Signal Hound SM200 20 GHz Real-Time Spectrum Analyzer](https://signalhound.com/products/sm200c-20-ghz-real-time-spectrum-analyzer-with-10gbe/)

### Requirements

- 64-bit Linux operating system
    - Tested on Ubuntu 18.04
- Native USB 3.0 support

### Prerequisites

1. [Install GNU Radio](https://wiki.gnuradio.org/index.php/InstallingGR).
    - GNURadio 3.8, currently a release candidate, updates to Python 3, which is not currently supported by this module. Look [here](https://wiki.gnuradio.org/index.php/GNU_Radio_3.8_OOT_Module_Porting_Guide) if you are interested in porting the code.
2. [Install the Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/).
    - Follow directions in _device_apis/sm_series/linux/README.txt_.

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

- Add the __SM200: IQ Source__ block to flowgraphs in the GNU Radio Companion. It is located under the __Signal Hound SM200__ category.
    - See _examples_ folder for demos.
- Use the block in Python with `import sm200`.
- Set the `networked` parameter to `True` for SM200C devices.
