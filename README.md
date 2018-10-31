# gr-sm200a

## A [GNU Radio](https://www.gnuradio.org) module for the [Signal Hound SM200A 20 GHz Real-Time Spectrum Analyzer](https://signalhound.com/products/sm200a-20-ghz-real-time-spectrum-analyzer/)

### Requirements

- 64-bit Linux operating system
- Native USB 3.0 support

### Prerequisites

1. [Install GNU Radio](https://wiki.gnuradio.org/index.php/InstallingGR).
2. [Install the Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/).
    - Follow directions in _device_apis/sm_series/linux/README.txt_.

### Installation

1. Clone this repository.
2. Run the following code from the root directory of the repository:

```console
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```

### Usage

- Add the __SM200A: IQ Source__ block to flowgraphs in the GNU Radio Companion. It is located under the __Signal Hound SM200A__ category.
    - See examples folder for demos.
- Use the block in Python code with `import sm200a`.
