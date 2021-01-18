@echo off
if exist %1.ps  ps2pdf -dEPSCrop %1.ps %1.pdf 
if exist %1.eps ps2pdf -dEPSCrop %1.eps %1.pdf 
