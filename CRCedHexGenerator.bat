@echo off

REM %1 = Output filename sans extension 
REM %2 = MAX Length Location
REM %3 = CRC Location

REM Get the end address of length
SET /A CROP_TO=%2+4

REM Write the Maximum address to a file
"../srec_cat" "%1.s19" -Motorola 2 -exclusive-maximum-b-e %2 4 -crop %2 %CROP_TO% -output - -hex-dump | "../awk" -F"[:#]" "{gsub(/[ \t]/, """""", $2); gsub(""^0*"""", """""", $2); print $2}" > %1-Max.tmp

REM Read the max address into a variable and trash the file
set /p MAX_PROG_ADDRESS= < "%1-Max.tmp"
del "%1-Max.tmp"

REM build the hex file, filling any holes and putting the length and CRC in the correct spots
"../srec_cat" "%1.s19" -Motorola 2 -maximum-b-e %2 4 -fill 0xff %2 0x%MAX_PROG_ADDRESS% -CRC16_Big_Endian %3 -Cyclic_Redundancy_Check_16_XMODEM -Output "%1-JTAG.s19" -Motorola 2  -disable=data-count
"../srec_cat" "%1.s19" -Motorola 2 -maximum-b-e %2 4 -fill 0xff %2 0x%MAX_PROG_ADDRESS% -CRC16_Big_Endian %3 -Cyclic_Redundancy_Check_16_XMODEM -Output "%1-TEMP.hex" -Intel_Hex_16
"../srec_cat" "%1-TEMP.hex" -Intel_Hex_16 -Byte_Swap -Output "%1-BOOTLOAD.hex" -Intel_Hex_16
del "%1-TEMP.hex"

REM show some output
echo ' '
echo Hex file generated and CRCed! - %1.hex
echo ---CRC Placed at:  %3
echo ---Max Length Placed at: %2
echo ---Max address is:  0x%MAX_PROG_ADDRESS%