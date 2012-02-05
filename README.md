
Package (.ipk file) building resources for the ToA Lightning Detection
Tracker Program for OpenWrt.

## Building

Obtain access to any Linux box with a reasonably new environment.

#### Download the SDK

Download the correct SDK file from http://downloads.openwrt.org/. Extract it to a convenient folder.

#### Copy the source files

Copy the `blitzortung_tracker` folder from the `package` directory of this repository to the `package` directory inside the SDK you downloaded.

#### Make

Invoke `make`.

### Done.

The .ipk file will end up as `bin/<arch>/packages/blitzortung_tracker_<version>_<arch>.ipk`.

