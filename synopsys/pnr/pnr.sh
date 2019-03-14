rm -rf work
mkdir work
cp pnr.tcl work/
cd work/
icc_shell -f pnr.tcl | tee -i out.txt 

