HBM Simulator based on DRAMSim2; a small source-code change with HBM configuration

1 Build------------------------------------------------------------------------

type:
$ make

2 Usage------------------------------------------------------------------------

DRAMSim2 can be built as a dynamic shared library which is convenient for
connecting it to CPU simulators or other custom front ends. A MemorySystem
object encapsulates the functionality of the memory system. 

For details usage, please refer to SST's memHierarchy component source code.
Specifically, <SST_ROOT>/sst/elements/memHierarchy/membackend/dramSimBackend.[h.cc]

Brielf explanations on arguments for HBM simulation; 
arg 1: device.ini (absoluth path will override arg 3)
arg 2: system.ini (absoluth path will override arg 3)
arg 3: search path for the files 
arg 4: trace file (which is obsolete for HBM simulation)
arg 5: channel size
