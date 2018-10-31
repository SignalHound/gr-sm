# gr-sm200a

## A [https://www.gnuradio.org]<GNU Radio> module for the [Signal Hound SM200A 20 GHz Real-Time Spectrum Analyzer]<https://signalhound.com/products/sm200a-20-ghz-real-time-spectrum-analyzer/>

### Requirements

- 64-bit Linux operating system
- Native USB 3.0 support

### Prerequisites

1. [Install GNU Radio]<https://wiki.gnuradio.org/index.php/InstallingGR>.
2. [Install the Signal Hound SDK]<https://signalhound.com/software/signal-hound-software-development-kit-sdk/>.
    - Follow directions in device_apis/sm_series/linux/README.txt.

### Installation

1. Clone this repository.
2. Run the following code from the root directory of the repository:

```
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### Usage

- Add the "SM200A: IQ Source" block to flowgraphs in the GNU Radio Companion. It is located under the "Signal Hound SM200A" category.
    - See examples folder for demos.
- Use the block in Python code with "import sm200a."