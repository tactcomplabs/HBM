/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland 
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

#include "MemorySystem.h"
#include "IniReader.h"
#include <unistd.h>

using namespace std;

unsigned NUM_RANKS;
unsigned NUM_RANKS_LOG;
unsigned NUM_BANKS_PER_BANKGROUP;

namespace DRAMSim {

MemorySystem::MemorySystem(unsigned sid, unsigned cid) :
  ReturnReadData(NULL), 
  WriteDataDone(NULL),
  stackID(sid), 
  channelID(cid)
{
  currentClockCycle = 0;

  DEBUG("====== Stack " << stackID << " - Channel" << channelID << " ======");

  /*
   * In legacy mode, each read or write transaction transfers 256 bits in a burst that consists of 2 
   * cycles of 128 bits each. 
   * In pseudo-channel mode, the 128-bit bus is split into 2 individual 64-bit segments. On each 
   * segment, a read or write transaction transfers 256 bits as well, but in a burst that lasts 4 
   * cycles (of 64 bits each).
   *
   * The pseudo channel concept essentially divides the memory of a single channel in half and 
   * assigns each half to a fixed pseudo channel.
   *
   * Note that pseudo channel share the same address and command bus: you can send a command and 
   * adresses to one pseudo channel or the other, but not to both.
   */

  if (operationMode == LegacyMode) {
    // One rank with 128-bits bus width and burst length of 2.
    NUM_RANKS = 1;
  } else { // operationMode == PseudoChannelMode
    // Each rank is regarded as a pseudo channel in PseudoChannelMode.
    // Two ranks, each with 64-bits bus width and burst length of 4.
    NUM_RANKS = 2;
  }

  NUM_RANKS_LOG = log2(NUM_RANKS);
  NUM_BANKS_PER_BANKGROUP = NUM_BANKS / NUM_BANKGROUPS;

  memoryController = new MemoryController(stackID, channelID, this);

  ranks = new vector<Rank *>();
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    Rank *r = new Rank();
    r->setId(i);
    r->attachMemoryController(memoryController);
    ranks->push_back(r);
  }

  memoryController->attachRanks(ranks);
}

MemorySystem::~MemorySystem()
{
  delete memoryController;

  for (unsigned i = 0; i < NUM_RANKS; ++i)
    delete (*ranks)[i];

  delete ranks;
}

bool MemorySystem::WillAcceptTransaction()
{
  return memoryController->WillAcceptTransaction();
}

bool MemorySystem::addTransaction(bool isWrite, uint64_t addr)
{
  TransactionType type = isWrite ? DATA_WRITE : DATA_READ;
  Transaction *trans = new Transaction(type, addr, NULL);

  if (memoryController->WillAcceptTransaction()) {
    return memoryController->addTransaction(trans);
  } else {
    pendingTransactions.push_back(trans);
    return true;
  }
}

bool MemorySystem::getStats( double *stat, DSIM_STAT metric ){
  return memoryController->getStats(stat, metric);
}

bool MemorySystem::getStats( uint64_t *stat, DSIM_STAT metric ){
  return memoryController->getStats(stat, metric);
}

void MemorySystem::printStats(bool finalStats)
{
  memoryController->printStats(finalStats);
}

//update the memory systems state
void MemorySystem::update()
{
  for (unsigned i = 0; i < NUM_RANKS; ++i)
    (*ranks)[i]->update();

  if (pendingTransactions.size() > 0 && memoryController->WillAcceptTransaction()) {
    memoryController->addTransaction(pendingTransactions.front());
    pendingTransactions.pop_front();
  }

  memoryController->update();
  
  for (unsigned i = 0; i < NUM_RANKS; ++i) 
    (*ranks)[i]->step();
  memoryController->step();
  this->step();
}

void MemorySystem::RegisterCallbacks(Callback_t* readCB, Callback_t* writeCB)
{
  ReturnReadData = readCB;
  WriteDataDone = writeCB;
}

} /*namespace DRAMSim */

// This function can be used by autoconf AC_CHECK_LIB since
// apparently it can't detect C++ functions.
// Basically just an entry in the symbol table
extern "C"
{
	void libdramsim_is_present(void)
	{
		;
	}
}
