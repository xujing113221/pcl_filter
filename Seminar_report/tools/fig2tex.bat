@echo off

fig2dev -L pstex -m 1.00 %1.fig %1.ps 
fig2dev -L pstex_t -m 1.00 -p %1 -F %1.fig %1.tex
ps2pdf -dEPSCrop %1.ps %1.pdf 

