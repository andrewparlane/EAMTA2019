rm -rf work
mkdir work
cp synth.tcl work/
cd work/
dc_shell -f synth.tcl | tee -i out.log

