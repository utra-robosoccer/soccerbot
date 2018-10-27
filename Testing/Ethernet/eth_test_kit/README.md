# eth\_test\_kit

These are custom scripts to carry out ethernet tests, convert test data between formats, analyze and plot test data.

Requires having anaconda3 installed, and a forked python module called scheddl (https://github.com/rfairley/scheddl). Must be run on Linux (Ubuntu has worked), as scheddl makes Linux kernel syscalls. TODO: parameter in script to specify host OS.

## 1. Installation

To install anaconda3, use the Linux installer https://www.anaconda.com/download/#linux.

To install scheddl to the anaconda3 python distribution you installed, do the following (from https://docs.python.org/3.7/install/):

```
git clone https://github.com/rfairley/scheddl.git
cd scheddl
/path/to/anaconda3/bin/python setup.py build
/path/to/anaconda3/bin/python setup.py install
```

Matplotlib also needs to be installed.

## 2. Setup

The MCU board must be connected by an ethernet cable. Flash the program in `Development/Ethernet/lwip-rtos-config/lwip-rtos-config` (or any TCP or UDP echo program) to the board. A static IP must be configured on the host workstation the board is connected to. `lwip-rtos-config/src/lwip.c` shows the IP address that the board is set to.

Check the parameters in the script eth_echo_test.py. `/path/to/anaconda3/bin/python eth_echo_test.py -h` shows what arguments can be passed to the script.

TODO: more in depth walkthrough on the setup and script parameters.

## 3. Usage

To collect test results, convert test data to readable format, and plot the data from readable format:

```
/path/to/anaconda3/bin/python eth_echo_test.py
/path/to/anaconda3/bin/python quick_stats.py $(ls \*.json)
/path/to/anaconda3/bin/python plot_from_quick_stats.py $(ls \*-quick_result.json)
```

Additionally, `/path/to/anaconda3/bin/python quick_stats_json2csv.py $(ls \*-quick_result.py)` converts the json results to csv, which makes it easier to load into Excel or other spreadsheet program.
