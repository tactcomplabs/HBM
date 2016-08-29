/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland dramninjas [at] gmail [dot] com
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
#include <errno.h> 
#include <sstream>
#include <stdlib.h>

#include "MultiChannelMemorySystem.h"
#include "AddressMapping.h"
#include "IniReader.h"

using namespace DRAMSim; 

std::atomic<int> MultiChannelMemorySystem::stackCount(0);

MultiChannelMemorySystem::MultiChannelMemorySystem(const string &dev, const string &sys, const 
    string &pwd_, unsigned megsOfMemory_) :
  deviceIniFilename(dev),
  systemIniFilename(sys), 
  pwd(pwd_),
  megsOfMemory(megsOfMemory_)
{
  stackID = stackCount++;
  currentClockCycle = 0; 

  if (!isPowerOfTwo(megsOfMemory)) {
    ERROR("Please specify a power of 2 memory size"); 
    abort(); 
  }

  if (pwd.length() > 0) {
    //ignore the pwd argument if the argument is an absolute path
    if (deviceIniFilename[0] != '/') deviceIniFilename = pwd + "/" + deviceIniFilename;
    if (systemIniFilename[0] != '/') systemIniFilename = pwd + "/" + systemIniFilename;
  }

  IniReader::ReadIniFile(deviceIniFilename, false);
  IniReader::ReadIniFile(systemIniFilename, true);
  IniReader::InitEnumsFromStrings();
  if (!IniReader::CheckIfAllSet()) 
    exit(-1);

  if (NUM_CHANS == 0) {
    ERROR("Zero channels"); 
    abort(); 
  }

  for (unsigned i = 0; i < NUM_CHANS; ++i) {
    MemorySystem *channel = new MemorySystem(stackID, i);
    channels.push_back(channel);
  }
}

MultiChannelMemorySystem::~MultiChannelMemorySystem()
{
  for (auto x: channels) 
    delete x;
}

void MultiChannelMemorySystem::update()
{
  if (currentClockCycle % EPOCH_LENGTH == 0)
    for (auto x: channels) 
      x->printStats(false);

  for (auto x: channels) 
    x->update();

  ++currentClockCycle; 
}

unsigned MultiChannelMemorySystem::findChannelNumber(uint64_t addr)
{
  if (NUM_CHANS == 1)
    return 0; 

  unsigned chan, rank, bank, row, col;
  addressMapping(addr, chan, rank, bank, row, col); 
  if (chan >= NUM_CHANS) {
    ERROR("Got channel index " << chan << " but only " << NUM_CHANS << " exist"); 
    abort();
  }

  return chan;
}

bool MultiChannelMemorySystem::addTransaction(bool isWrite, uint64_t addr)
{
  unsigned chan = findChannelNumber(addr); 
  return channels[chan]->addTransaction(isWrite, addr); 
}

bool MultiChannelMemorySystem::willAcceptTransaction(uint64_t addr)
{
  unsigned chan, rank, bank, row, col; 
  addressMapping(addr, chan, rank, bank, row, col); 
  return channels[chan]->WillAcceptTransaction();
}

void MultiChannelMemorySystem::printStats(bool finalStats) 
{
  for (unsigned i = 0; i < NUM_CHANS; ++i) {
    //PRINT("==== Channel [" << i << "] ====");
    channels[i]->printStats(finalStats); 
    //PRINT("//// Channel [" << i << "] ////");
  }
}

void MultiChannelMemorySystem::RegisterCallbacks(TransactionCompleteCB *readDone, 
    TransactionCompleteCB *writeDone)
{
  for (auto x: channels) 
    x->RegisterCallbacks(readDone, writeDone);
}

namespace DRAMSim 
{
  MultiChannelMemorySystem *getMemorySystemInstance(const string &dev, const string &sys, const 
      string &pwd, unsigned megsOfMemory) 
  {
    return new MultiChannelMemorySystem(dev, sys, pwd, megsOfMemory);
  }
}
