# Xmega usb serial demo

Basic hello world demo for Atmel Xmega USB chips (tested with the
Xmega32a4u).

Based on the [LUFA](http://www.fourwalledcubicle.com/LUFA.php) library
by Dean Camera.

## Setup

Requires a copy (or link) to the latest LUFA source code in the
current directory.  Makefile assumes LUFA-130303.

The Makefile probably needs to be modified to support the particular
chip and programmer.

## Using

    $ make
    $ make program

## Behavior

Should blink an LED on E3 3 times.  When you connect to the serial
port, it will blink 1, 2, and 3 times and print out `Hello World!
{count}` three times.

XXX For some reason, the first line never shows up for me.
