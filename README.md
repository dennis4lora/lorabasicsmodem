# Disclaimer

This is an cloned repository. It includes a few small issues based on the `2.0.0-Alpha` tag, and it supports 4-NAV
multi-frame format by default. It also merged the Alpha branch back to `master` branch to make things simpler.

Please be noted that the latest lorabasicsmodem development is happening at [SDK](https://github.com/Lora-net/SWSD001).
This one is just for users who still need it for their own reason.

This is not an official Semtech repository. The author and Semtech takes no responsibility for any issue caused by this repository.

# Hardware specificities  

### Reference board
The Basic Modem reference board is ST Microelectronics NucleoL476 + Semtech lr1110 evaluation module (China version)

### Supported Radios  
LR1110 

----
# LoRaWAN  

### Version  
The LoRaWAN version that is currently implemented in the modem is: 1.0.3

### Supported Regions  
* CN470 from Regional Parameters 1.0

----
# Supported Modem Services 

* Large files upload
* ROSE Streaming
* Alc_sync - Clock synchronization
* Almanac Update

# Build options

----
### Toolchain choice

The gcc compiler bin path can be either defined in make command via `GCC_PATH` variable either it can be added to the PATH environment variable.
> make all GCC_PATH=xxx

----
### Makefile help

To see all makefile options:  
>make help

----
### Clean options

Clean the current build directory:  
>make clean
----
### Full build

Clean and build all:  
>make full

----
### Application example

A geolocation application example is given as an example of modem use.

### Monochannel option

The project can be built for a single chanel gateway. In this case please add the `HYBRID_CHINA=yes` option to the make command
>make full HYBRID_CHINA=yes

----

# Modem Use

### Modem API
Modem API definitions can be found in `smtc_modem_api`

All generic modem functions are defined in `smtc_modem_api/smtc_modem_api.h`

### Modem engine

To be functional modem shall be initialized and periodically called in the main loop.

These functions can be found in `smtc_modem_api/smtc_modem_utilities.h`

