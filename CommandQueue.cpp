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

#include "CommandQueue.h"
#include "MemoryController.h"
#include <assert.h>

using namespace DRAMSim;

CommandQueue::CommandQueue(vector<vector<BankState>>& states) :
  bankStates(states),
  nextRank(0),
  nextBank(0),
  nextRankPRE(0),
  nextBankPRE(0),
  refreshRank(0),
  refreshWaiting(false)
{
  // set here to avoid compile errors
  currentClockCycle = 0;

  // vector of counters used to ensure rows don't stay open too long
  rowAccessCounters = vector<vector<unsigned>>(NUM_RANKS, vector<unsigned>(NUM_BANKS, 0));

  // create per-rank queues
  queues = vector<vector<BusPacket*>>(NUM_RANKS);
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    queues.push_back(vector<BusPacket*>());
  }

  // create per-rank tFAW-counters to count the number of activations within a given window
  // each decrementing counter starts at tFAW and when the 0-th element reaches 0, it's removed
  tFAWCountdown.reserve(NUM_RANKS);
  for (unsigned i = 0; i < NUM_RANKS; ++i)
    tFAWCountdown.push_back(vector<unsigned>());
}

CommandQueue::~CommandQueue()
{
  for (auto rankQueue: queues)
    for (auto command: rankQueue)
      delete command;
}

bool CommandQueue::pop(BusPacket **busPacket)
{
  // tFAW bookkeeping is done here because pop() is called every clock cycle 
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    // decrement all the counters we have
    for (unsigned j = 0; j < tFAWCountdown[i].size(); ++j)
      tFAWCountdown[i][j]--;

    // the head will always be the smallest counter, so check if it has reached 0
    if (tFAWCountdown[i].size() > 0 && tFAWCountdown[i][0] == 0)
      tFAWCountdown[i].erase(tFAWCountdown[i].begin());
  }

  /* 
   * Now we need to find a packet to issue. 
   * When the code picks a packet, it will set *busPacket = [some eligible packet]
   * First the code looks if any refreshes need to go
   * Then it looks for data packets
   * Otherwise, it starts looking for rows to close (in open page)
   */

  if (rowBufferPolicy == ClosePage) {
    bool sendingREF = false;

    // if a rank, specified as refreshRank, is in need of a refresh
    if (refreshWaiting) {
      bool foundActiveOrTooEarly = false;
      auto& commandQueue = queues[refreshRank];
      // ensure all banks are idle
      for (unsigned b = 0; b < NUM_BANKS; ++b) {
        if (bankStates[refreshRank][b].currentBankState == RowActive) { // if a bank is open
          foundActiveOrTooEarly = true; // we cannot send a REF

          // ensure there is nothing else going there before we close it
          for (unsigned j = 0; j < commandQueue.size(); ++j) {
            BusPacket* packet = commandQueue[j];
            if ((packet->bank == b) && (packet->row == bankStates[refreshRank][b].openRowAddress)) {
              if (packet->busPacketType != ACTIVATE && isIssuable(packet)) {
                *busPacket = packet;
                commandQueue.erase(commandQueue.begin()+j);
                sendingREF = true;
              }
              break;
            }
          }
          break;
        }
        // checks nextActivate time for each bank to make sure tRP is being satisfied.
        // the next ACT and next REF can be issued at the same point in the future, 
        // so just use nextActivate field instead of creating a nextRefresh field
        else if (bankStates[refreshRank][b].nextActivate > currentClockCycle) {
          foundActiveOrTooEarly = true;
          break;
        }
      }

      // if there are no open banks and timing has been met
      // then send out the refresh, reset flags and rank pointer
      if (!foundActiveOrTooEarly && bankStates[refreshRank][0].currentBankState != PowerDown) {
        *busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0);
        refreshRank = -1;
        refreshWaiting = false;
        sendingREF = true;
      }
    }

    // if we're not sending a REF, look for an issuable command
    if (!sendingREF) {
      bool foundIssuable = false;
      unsigned startingRank = nextRank;
      do {
        auto& commandQueue = queues[nextRank];
        if (!commandQueue.empty() && !(refreshWaiting && (nextRank == refreshRank))) {
          for (unsigned i = 0; i < commandQueue.size(); ++i) {
            BusPacket* packet = commandQueue[i];
            if (isIssuable(packet)) {
              // ensure we don't remove a read/write paired with an activate
              if (i > 0 && commandQueue[i-1]->busPacketType == ACTIVATE && 
                  commandQueue[i-1]->physicalAddress == packet->physicalAddress)
                continue;

              *busPacket = packet;
              commandQueue.erase(commandQueue.begin()+i);
              foundIssuable = true;
              break;
            }
          }
        }

        if (foundIssuable) 
          break;

        nextRank = (nextRank + 1) % NUM_RANKS;
      } while (nextRank != startingRank);

      // if we cannot find anything to send, return false
      if (!foundIssuable) 
        return false;
    }
  } else if (rowBufferPolicy == OpenPage) {
    bool sendingREForPRE = false;

    // if a rank, specified as refreshRank, is in need of a refresh
    if (refreshWaiting) {
      bool sendingREF = true;
      auto& commandQueue = queues[refreshRank];
      // ensure all banks are idle and timing is met for a REF
      for (unsigned b = 0; b < NUM_BANKS; ++b) {
        if (bankStates[refreshRank][b].currentBankState == RowActive) { // if a bank is open
          sendingREF = false; // we cannot send a REF

          // ensure there is nothing else going there before we close it
          bool closeRow = true;
          for (unsigned j = 0; j < commandQueue.size(); ++j) {
            BusPacket *packet = commandQueue[j];
            if ((packet->bank == b) && (packet->row == bankStates[refreshRank][b].openRowAddress)) {
              if (packet->busPacketType != ACTIVATE) {
                closeRow = false;
                if (isIssuable(packet)) {
                  *busPacket = packet;
                  commandQueue.erase(commandQueue.begin()+j);
                  sendingREForPRE = true;
                }
                break;
              } else { // packet->busPacketType == ACTIVATE
                // if we've encountered another ACT, no other command will be of interest
                break;
              }
            }
          }

          // if a bank is open and we are allowed to close it, then send a PRE
          if (closeRow && (currentClockCycle >= bankStates[refreshRank][b].nextPrecharge)) {
            rowAccessCounters[refreshRank][b] = 0;
            *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, refreshRank, b, 0);
            sendingREForPRE = true;
          }
          break;
        }
        // checks nextActivate time for each bank to make sure tRP is being satisfied.
        // the next ACT and next REF can be issued at the same point in the future, 
        // so just use nextActivate field instead of creating a nextRefresh field
        else if (bankStates[refreshRank][b].nextActivate > currentClockCycle) {
          sendingREF = false;
          break;
        }
      }

      // if there are no open banks and timing has been met
      // then send out the refresh, reset flags and rank pointer
      if (sendingREF && bankStates[refreshRank][0].currentBankState != PowerDown) {
        *busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0);
        refreshRank = -1;
        refreshWaiting = false;
        sendingREForPRE = true;
      }
    }

    // if we're not sending a REF or PRE, look for an issuable command
    if (!sendingREForPRE) {
      bool foundIssuable = false;
      unsigned startingRank = nextRank;
      do {
        auto& commandQueue = queues[nextRank];
        if (!commandQueue.empty() && !(refreshWaiting && (nextRank == refreshRank))) {
          for (unsigned i = 0; i < commandQueue.size(); ++i) {
            BusPacket* packet = commandQueue[i];
            if (isIssuable(commandQueue[i])) {
              // check for dependencies
              bool dependencyFound = false;
              for (unsigned j = 0; j < i; ++j) {
                BusPacket* prevPacket = commandQueue[j];
                if (prevPacket->bank == packet->bank && prevPacket->busPacketType != ACTIVATE && 
                    prevPacket->row == packet->row) {
                  dependencyFound = true;
                  break;
                }
              }

              if (dependencyFound) 
                continue;

              *busPacket = packet;

              // if there is an ACT paired with this column command we are removing, 
              // then remove that ACT as well 
              if (i > 0 && commandQueue[i-1]->busPacketType == ACTIVATE) {
                rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
                delete commandQueue[i-1];
                commandQueue.erase(commandQueue.begin()+i-1, commandQueue.begin()+i+1);
              } else {
                commandQueue.erase(commandQueue.begin()+i);
              }

              foundIssuable = true;
              break;
            }
          }
        }

        if (foundIssuable)
          break;

        nextRank = (nextRank + 1) % NUM_RANKS;
      } while (nextRank != startingRank);

      // if nothing is issuable, 
      // then see if we can issue a PRE to an open bank that has no other commands waiting
      if (!foundIssuable) {
        bool sendingPRE = false;
        unsigned startingRank = nextRankPRE;
        unsigned startingBank = nextBankPRE;

        do {
          bool found = false;
          auto& commandQueue = queues[nextRankPRE];
          if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive) {
            for (unsigned i = 0; i < commandQueue.size(); ++i) {
              //if there is something going to that bank and row, then we don't want to send a PRE
              if ((commandQueue[i]->bank == nextBankPRE) && 
                  (commandQueue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)) {
                found = true;
                break;
              }
            }

            // if nothing is going to that bank and row or too many accesses have made, close it
            if (!found || rowAccessCounters[nextRankPRE][nextBankPRE] == TOTAL_ROW_ACCESSES) {
              if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge) {
                sendingPRE = true;
                rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0);
                break;
              }
            }
          }
          nextRankAndBank(nextRankPRE, nextBankPRE);
        } while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));

        // if no PRE could be sent, return
        if (!sendingPRE) 
          return false;
      }
    }
  }

  // if an ACT is to be sent, add a tFAW counter
  if ((*busPacket)->busPacketType == ACTIVATE)
    tFAWCountdown[(*busPacket)->rank].push_back(tFAW);

  return true;
}

bool CommandQueue::hasRoomFor(unsigned numberToEnqueue, unsigned rank)
{
  auto& commandQueue = queues[rank]; 
  return ((CMD_QUEUE_DEPTH - commandQueue.size()) >= numberToEnqueue);
}

void CommandQueue::print()
{
  PRINTN("cycle:" << currentClockCycle << " printing per-rank command queue");
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    PRINT(" rank[" << i << "] size:" << queues[i].size());
    for (unsigned j = 0; j < queues[i].size(); ++j) {
      PRINTN("    "<< j << "]");
      queues[i][j]->print();
    }
  }
}

bool CommandQueue::isIssuable(BusPacket *busPacket)
{
  bool issuable = false;
  bool state = false;
  bool timing = false;
  bool restriction = false;
  bool rowReady = false;

  switch (busPacket->busPacketType) {
    case REFRESH: 
      break;

    case ACTIVATE:
      state = (bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle ||
          bankStates[busPacket->rank][busPacket->bank].currentBankState == Refreshing);
      timing = (currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextActivate);
      restriction = (tFAWCountdown[busPacket->rank].size() < 4);

      if (state && timing && restriction)
        issuable = true;
      break;

    case WRITE:
    case WRITE_P:
      state = (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive);
      timing = (currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextWrite);
      rowReady = (busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress);
      restriction = (rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES);

      if (state && timing && rowReady && restriction)
        issuable = true;
      break;

    case READ_P:
    case READ:
      state = (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive);
      timing = (bankStates[busPacket->rank][busPacket->bank].nextRead <= currentClockCycle);
      rowReady = (busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress);
      restriction = (rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES);

      if (state && timing && rowReady && restriction)
        issuable = true;
      break;

    case PRECHARGE:
      state = (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive);
      timing = (currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge);
      
      if (state && timing)
        issuable = true;
      break;

    default:
      ERROR("== Error - Trying to issue a crazy bus packet type : ");
      busPacket->print();
      exit(0);
  }

  return issuable;
}

void CommandQueue::nextRankAndBank(unsigned &rank, unsigned &bank)
{
  if (schedulingPolicy == RankThenBankRoundRobin) {
    rank = (rank + 1) % NUM_RANKS;
    if (rank == 0) 
      bank = (bank + 1) % NUM_BANKS;
  } else if (schedulingPolicy == BankThenRankRoundRobin) {
    bank = (bank + 1) % NUM_BANKS;
    if (bank == 0)
      rank = (rank + 1) % NUM_RANKS;
  } else {
    ERROR("== Error - Unknown scheduling policy");
    exit(0);
  }
}
