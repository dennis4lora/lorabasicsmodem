# Features description

----
## Hardware specificities  

### Reference board
The soft modem reference board is ST Microelectronics NucleoL073 + Semtech SX1280 dev kit board

### Supported Radios  
sx1280 radio is used in case of WW2G4 region  

----
## LoRaWAN  

### Version  
The LoRaWAN version that is currently implemented in the modem is: 1.0.3

### Supported Regions  
* WW2G4  

----
## Supported Modem Services 

### Large files upload

### ROSE Streaming

### Alc_sync - Clock synchronization

# Build options

----
## Toolchain choice

The gcc compiler bin path can be either defined in make command via `GCC_PATH` variable either it can be added to the PATH environment variable.
> make GCC_PATH=xxx

----
## Clean options

Clean the current build directories:  
>make clean

----
## Multiple targets options

There are 4 targets that can be built with the Makefile:

### Soft Modem 2G4  
>make modem_2_4  

The Binary file will like: `/build/build_modem_2_4/soft_modem_2g4.bin`

----
## General options

`-j` option can be added to the make command in order to fasten the build using all machine cores

To make from scratch all the targets quickly  
>make clean; make all -j

----
## Hardware modem overlay
Just add `hard_modem` after the chosen target  
Example: to build all targets with hardware modem activated:  
>make clean; make all -j hard_modem

----
## Board options

In the Makefile file, the default board `BOARD_L073` is used to compile HAL
