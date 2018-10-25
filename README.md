# phidgets_interface_kit

This package provides support for using the Phidgets InterfaceKit.

## Running the phidgets without using root

First connect the phidget through the usb cable and look at the different usb devices that are connected.
```
lsusb
```

You should see something resembling the following:
```
Bus 001 Device 009: ID 06c2:0045 Phidgets Inc. (formerly GLAB) PhidgetInterface Kit 8-8-8
```

The part that says `06c2:0045` describes the product id and the vendor id, respectively. You must add these, along with a group and a mode to a udev rule. To do so, create a file called `/etc/udev/rules.d/11-ftdi.rules` and add the following line to that file.
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="06c2", ATTRS{idProduct}=="0045", GROUP="dialout", MODE="0666"
```

Whichever group you use (`dialout` in this case), make sure your user is added to that group.
```
sudo usermod -aG dialout $USER
```

I've seen tutorials that suggest dialout as the group. Those tutorials have also suggested adding the user to the `tty` group. I did this but haven't verified that it is necessary. To do so, type the follwing:
```
sudo usermod -aG tty $USER
```

There are ways to reload the rules without restaring once you create them. However, I just had to restart the computer to get everything working correctly.

## Published Topics

 * `analog_in` (phidgets_interface_kit/AnalogArray)
 * `digital_in` (phidgets_interface_kit/DigitalArray)
 * `digital_out` (phidgets_interface_kit/DigitalArray)

## Subscribed Topics

 * `cmd_digital_out` (phidgets_interface_kit/DigitalArray)
