fir:
	xvlog -f ./include.rtl.list.xsim ./tbta.v
	xelab -top fir_tb -snapshot tb_elab
	xsim tb_elab -R
clean:
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb
	rm fir.vcd