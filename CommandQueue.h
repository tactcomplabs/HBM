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

#ifndef CMDQUEUE_H
#define CMDQUEUE_H

#include "BusPacket.h"
#include "BankState.h"
#include "Transaction.h"
#include "SystemConfiguration.h"
#include "SimulatorObject.h"

using namespace std;

namespace HBMDRAMSim {
class CommandQueue : public SimulatorObject
{
  private:
    CommandQueue();

  public:
    CommandQueue(vector<vector<BankState>> &states);
    virtual ~CommandQueue(); 

    void enqueue(BusPacket *newBusPacket)
    {
      queues[newBusPacket->rank].push_back(newBusPacket);
    }

    bool pop(BusPacket **busPacket);
    bool hasRoomFor(unsigned numberToEnqueue, unsigned rank);
    bool isIssuable(BusPacket *busPacket);

    bool isEmpty(unsigned rank)
    {
      return queues[rank].empty();
    }

    // indicate a particular rank is in need of a refresh
    void needRefresh(unsigned rank) 
    {
      refreshWaiting = true;
      refreshRank = rank;
    }
    void print();
    void update() //SimulatorObject requirement
    {
      // do nothing since pop() is effectively update(),
      // TODO: make CommandQueue not a SimulatorObject
    } 

  public:
    vector<vector<BusPacket*>> queues;
    vector<vector<BankState>> &bankStates;

  private:
    void nextRankAndBank(unsigned &rank, unsigned &bank);
    unsigned nextRank;
    unsigned nextBank;
    unsigned nextRankPRE;
    unsigned nextBankPRE;

    unsigned refreshRank;
    bool refreshWaiting;

    vector<vector<unsigned>> tFAWCountdown;
    vector<vector<unsigned>> rowAccessCounters;
}; //class CommandQueue
} //namespace HBMDRAMSim

#endif

