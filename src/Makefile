OUT  = ../out
QOUT = ../qout
QOUTREL = ../src

MOD  = mod
TMOD = tmod

CPUM = cpum
INC  = inc

TABLE = table
TABLE_OUT = $(OUT)/table_out

GENTABLE = python ../util/table-generate2.py

tables  = $(wildcard $(TABLE)/*.table)
table_v = $(patsubst $(TABLE)/%.table,$(TABLE_OUT)/%.table.v,$(tables))

INCLUDE = $(INC) $(CPUM) $(TABLE_OUT)

includes = \
	$(wildcard $(INC)/*.v) \
	$(wildcard $(CPUM)/*.v) \
	$(wildcard $(TABLE_OUT)/*.v)

toplevel = main

allmods  = $(wildcard $(MOD)/*.v)
mods     = \
	$(MOD)/clock_generator.v \
	$(MOD)/cpu.v \
	$(MOD)/pc.v \
	$(MOD)/imem.v \
	$(MOD)/decoder.v \
	$(MOD)/decoder_signal_generator.v \
	$(MOD)/decoder_signal_pack.v \
	$(MOD)/alu.v  \
	$(MOD)/vreg.v \
	$(MOD)/vmem.v \
	$(MOD)/dmem.v

tmods    = $(patsubst $(TMOD)/%.v,%,$(wildcard $(TMOD)/*.v))

.PHONY: default prepare gen_table clean

default: main


prepare:
	mkdir -p $(OUT)
	mkdir -p $(TABLE_OUT)

gen_table: $(table_v)

$(TABLE_OUT)/%.table.v: $(TABLE)/%.table
	$(GENTABLE) < $< > $@

$(patsubst %,v%,$(tmods)): prepare $(mods) $(includes) gen_table
	iverilog $(IVERFLAGS) -o$@ -s $(toplevel) $(patsubst %,-I%,$(INCLUDE)) -- $(mods) $(patsubst v%,$(TMOD)/%.v,$@)

prep: prepare $(mods) $(includes) gen_table
	iverilog $(IVERFLAGS) -E -o$@ -s $(toplevel) $(patsubst %,-I%,$(INCLUDE)) -- $(mods) $(patsubst v%,$(TMOD)/%.v,$@)

$(patsubst %,l%,$(tmods)): prepare $(mods) $(includes) gen_table
	verilator $(VERIFLAGS) -o$@ -s $(toplevel) $(patsubst %,-I%,$(INCLUDE)) -- $(mods) $(patsubst v%,$(TMOD)/%.v,$@)

quartus-srcs: prepare $(allmods) $(includes) gen_table
	mkdir -p $(QOUT)
	ln -fvst $(QOUT) -- $(patsubst %,$(QOUTREL)/%,$(allmods) $(includes))

clean:
	rm -rf -- $(OUT)
