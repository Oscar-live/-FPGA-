@echo

copy main.bin e:\
start "program" cmd /k call program.bat

e:
cd \My_Document\WSJ_Master\Competition\FPGA_Design_Conpetition\my_PDS_code\C_Compile\tools\openocd
openocd.exe -f tinyriscv.cfg

rem start "" cmd /k call program.bat

