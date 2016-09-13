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


#ifndef MEMORYCONTROLLER_H
#define MEMORYCONTROLLER_H

#include <map>
#include <deque>

#include "SimulatorObject.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "CommandQueue.h"
#include "BusPacket.h"
#include "BankState.h"
#include "Rank.h"

using namespace std;

namespace DRAMSim
{
#ifdef DEBUG_LATENCY
class LatencyBreakdown
{
  public:
    LatencyBreakdown() {}  // default constructor
    LatencyBreakdown(uint64_t tattq, bool read = true) : 
      isRead(read),
      timeAddedToTransactionQueue(tattq),
      timeAddedToCommandQueue(0),
      timeScheduled(0),
      timeRowCommandExecuted(0),
      timeColumnCommandExecuted(0),
      timeWriteDone(0),
      timeReadDone(0),
      timeReturned(0) {} // constructor

  public:
    bool isRead;
    uint64_t timeAddedToTransactionQueue;
    uint64_t timeAddedToCommandQueue;
    uint64_t timeScheduled; 
    uint64_t timeRowCommandExecuted; //TODO
    uint64_t timeColumnCommandExecuted; //TODO
    uint64_t timeWriteDone;
    uint64_t timeReadDone;
    uint64_t timeReturned;
};
#endif

class MemorySystem;
class MemoryController : public SimulatorObject
{

public:
  MemoryController(unsigned sid, unsigned cid, MemorySystem* ms);
  virtual ~MemoryController();

  bool addTransaction(Transaction *trans);
  bool WillAcceptTransaction();
  void returnReadData(Transaction *trans);
  void receiveFromBus(BusPacket *bpacket);
  void attachRanks(vector<Rank *> *ranks) { this->ranks = ranks; };
  void updateBankStates();
  void update();
  void printStats(bool finalStats = false);
  void resetStats(); 

public:
  vector<Transaction*> transactionQueue;
#ifdef DEBUG_LATENCY
  map<uint64_t, deque<LatencyBreakdown>> latencyBreakdowns;
#endif

private:
  unsigned stackID;
  unsigned channelID;

  MemorySystem* parentMemorySystem;
  vector<vector<BankState>> bankStates;

  CommandQueue commandQueue;
  BusPacket* poppedBusPacket;
  vector<unsigned> refreshCountdown;
  vector<BusPacket*> writeDataToSend;
  vector<unsigned> writeDataCountdown;
  vector<Transaction*> returnTransaction;
  vector<Transaction*> pendingReadTransactions;
  map<unsigned,unsigned> latencies; // latencyValue -> latencyCount
  vector<bool> powerDown;

  vector<Rank*>* ranks;

  // these packets are counting down waiting to be transmitted on the "bus"
  BusPacket* outgoingRowCmdPacket;
  BusPacket* outgoingColCmdPacket;
  unsigned rowCmdCyclesLeft;
  unsigned colCmdCyclesLeft;
  vector<BusPacket*> outgoingDataPackets;
  vector<unsigned> dataCyclesLeft;

  uint64_t totalTransactions;
  vector<uint64_t> grandTotalBankAccesses; 
  vector<uint64_t> totalReadsPerBank;
  vector<uint64_t> totalWritesPerBank;
  vector<uint64_t> totalReadsPerRank;
  vector<uint64_t> totalWritesPerRank;
  vector<uint64_t> totalEpochLatency;

  unsigned channelBitWidth;
  unsigned rankBitWidth;
  unsigned bankBitWidth;
  unsigned rowBitWidth;
  unsigned colBitWidth;
  unsigned byteOffsetWidth;

  unsigned refreshRank;
};
}

#endif

