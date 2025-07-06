# 让 Make 使用环境变量 ROOT_PATH，如果没有设置则退出
ifndef ROOT_PATH
$(error ROOT_PATH is not set (source project.sh first))
endif

# 定义源文件路径
SRCS = $(ROOT_PATH)/verification/tb/testbench.v \
       $(ROOT_PATH)/rtl/easyaxi.v

# 输出名称和波形
OUT = simv
FST = dump.fst

# 默认目标
all: sim

# 仿真流程：编译 + 运行 + 输出 .fst 
sim:
	iverilog -g2012 -o $(OUT) $(SRCS)
	vvp $(OUT) -fst

# 清理中间文件
clean:
	rm -f $(OUT) $(FST)


# # all: filelist com_fsdb sim verdi
# #---------------------------------------------------------------
# #filelist:
# #    find ../de ../dv -name "*.v" -o -name "*.sv" > filelist.f
# #---------------------------------------------------------------
# compile:
# 	vcs -f ../tb/tb.f \
# 	-timescale=1ns/1ps \
# 	-full64 +v2k -sverilog -Mupdate +define+DUMP_FSDB \
# 	-P ${VERDI_HOME}/share/PLI/VCS/LINUX64/novas.tab ${VERDI_HOME}/share/PLI/VCS/LINUX64/pli.a \
# 	-debug_acc+all -debug_region+cell+encrypt \
# 	|tee vcs.log 
# #---------------------------------------------------------------
# run:
# 	vcs -f ../tb/tb.f \
# 	-timescale=1ns/1ps \
# 	-full64 +v2k -sverilog -Mupdate +define+DUMP_FSDB \
# 	-R \
# 	-P ${VERDI_HOME}/share/PLI/VCS/LINUX64/novas.tab ${VERDI_HOME}/share/PLI/VCS/LINUX64/pli.a \
# 	-debug_acc+all -debug_region+cell+encrypt \
# 	|tee vcs.log 
# #---------------------------------------------------------------
# sim:
# 	./simv |tee sim.log
# #---------------------------------------------------------------
# #dve:
# #	dve -vpd vcdplus.vpd -script dve.tcl 
# #---------------------------------------------------------------
# verdi:
# 	verdi -f ../tb/tb.f  
# #---------------------------------------------------------------
# clean:
# 	rm -rf csrc DVEfiles novas* *.log simv* *fsdb* ucli.key *.vpd verdiLog
