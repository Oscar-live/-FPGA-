@echo

copy gpio.bin G:\
start "program" cmd /k call program.bat

g:
cd G:\FPGA_Competition\C_Compile\tools\openocd
openocd.exe -f tinyriscv.cfg

rem start "" cmd /k call program.bat

