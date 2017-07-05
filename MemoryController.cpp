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

#include <algorithm>

#include "MemoryController.h"
#include "MemorySystem.h"
#include "AddressMapping.h"

#define SEQUENTIAL(rank,bank) (rank*NUM_BANKS)+bank

using namespace DRAMSim;

MemoryController::MemoryController(unsigned sid, unsigned cid, MemorySystem *parent) :
  stackID(sid),
  channelID(cid),
  bankStates(NUM_RANKS, vector<BankState>(NUM_BANKS)),
  commandQueue(bankStates),
  poppedBusPacket(NULL),
  totalTransactions(0),
  refreshRank(0)
{
  parentMemorySystem = parent;
  
  // outgoingRowCmdPacket and outgoingColCmdPacket represent shared row and column command bus 
  // between ranks, or pseudo channels.
  outgoingRowCmdPacket = NULL;
  outgoingColCmdPacket = NULL;
  rowCmdCyclesLeft = 0;
  colCmdCyclesLeft = 0;

  // outgoingDataPackets represent per-pseudo-channel I/Os for write
  outgoingDataPackets.reserve(NUM_RANKS);
  dataCyclesLeft.reserve(NUM_RANKS);
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    outgoingDataPackets[i] = NULL;
    dataCyclesLeft[i] = -1;
  }

  writeDataToSend.reserve(NUM_RANKS);
  writeDataCountdown.reserve(NUM_RANKS);
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    writeDataToSend[i] = NULL;
    writeDataCountdown[i] = -1;
  }
  
  //set here to avoid compile errors
  currentClockCycle = 0;
  
  //reserve memory for vectors
  transactionQueue.reserve(TRANS_QUEUE_DEPTH);

  powerDown = vector<bool>(NUM_RANKS, false);
  grandTotalBankAccesses = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
  totalReadsPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
  totalWritesPerBank = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
  totalReadsPerRank = vector<uint64_t>(NUM_RANKS, 0);
  totalWritesPerRank = vector<uint64_t>(NUM_RANKS, 0);
  totalEpochLatency = vector<uint64_t>(NUM_RANKS * NUM_BANKS, 0);
  
  refreshCountdown.reserve(NUM_RANKS);
  for (unsigned i = 0; i < NUM_RANKS; ++i)
    refreshCountdown.push_back((int)((REFRESH_PERIOD/tCK)/NUM_RANKS)*(i+1));
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket)
{
  if (bpacket->busPacketType != DATA) {
    ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - received a non-DATA bus packet from rank");
    bpacket->print();
    exit(0);
  }
  
  //add to return read data queue
  returnTransaction.push_back(new Transaction(RETURN_DATA, bpacket->physicalAddress, bpacket->data));
  totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;

#ifdef DEBUG_LATENCY
  deque<LatencyBreakdown> &dq = latencyBreakdowns[bpacket->physicalAddress];
  for (auto it = dq.begin(); it != dq.end(); ++it) {
    if (it->isRead && it->timeReadDone == 0) {
      it->timeReadDone = currentClockCycle;
      break;
    }
  }
#endif

  // this delete statement saves a mindboggling amount of memory
  delete(bpacket);
}

//sends read data back to the CPU
void MemoryController::returnReadData(Transaction *trans)
{
  if (parentMemorySystem->ReturnReadData != NULL)
    (*parentMemorySystem->ReturnReadData)(parentMemorySystem->channelID, trans->getAddress(), 
        currentClockCycle);
}

void MemoryController::updateBankStates()
{
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    for (unsigned j = 0; j < NUM_BANKS; ++j) {
      if (bankStates[i][j].stateChangeCountdown > 0) {
        bankStates[i][j].stateChangeCountdown--;
        if (bankStates[i][j].stateChangeCountdown == 0) {
          // for the commands that have an implicit state change
          switch (bankStates[i][j].lastCommand) {
            case WRITE_P:
            case READ_P:
              bankStates[i][j].currentBankState = Precharging;
              bankStates[i][j].lastCommand = PRECHARGE;
              bankStates[i][j].stateChangeCountdown = tRP;
              break;

            case REFRESH:
            case PRECHARGE:
              bankStates[i][j].currentBankState = Idle;
              break;

            default:
              break;
          }
        }
      }
    }
  }
}

void MemoryController::update()
{
  updateBankStates();

  // check for outgoing row command packets and handle countdowns
  if (outgoingRowCmdPacket != NULL) {
    rowCmdCyclesLeft--;
    if (rowCmdCyclesLeft == 0) { // packet is ready to be received by rank
      (*ranks)[outgoingRowCmdPacket->rank]->receiveFromBus(outgoingRowCmdPacket);
      outgoingRowCmdPacket = NULL;
    }
  }

  // check for outgoing column command packets and handle countdowns
  if (outgoingColCmdPacket != NULL) {
    colCmdCyclesLeft--;
    if (colCmdCyclesLeft == 0) { // packet is ready to be received by rank
      (*ranks)[outgoingColCmdPacket->rank]->receiveFromBus(outgoingColCmdPacket);
      outgoingColCmdPacket = NULL;
    }
  }

  // check for outgoing data packets and handle countdowns
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    if (outgoingDataPackets[i] != NULL) {
      dataCyclesLeft[i]--;
      if (dataCyclesLeft[i] == 0) {
        // inform upper levels that a write is done
        if (parentMemorySystem->WriteDataDone != NULL)
          (*parentMemorySystem->WriteDataDone)(parentMemorySystem->channelID, 
              outgoingDataPackets[i]->physicalAddress, currentClockCycle);

#ifdef DEBUG_LATENCY
        deque<LatencyBreakdown> &dq = latencyBreakdowns[outgoingDataPackets[i]->physicalAddress];
        for (auto it = dq.begin(); it != dq.end(); ++it) {
          if (!it->isRead && it->timeWriteDone == 0) {
            it->timeWriteDone = currentClockCycle;
            it->timeReturned = currentClockCycle;

            LatencyBreakdown &lbd = *it;
            DEBUG("[" << stackID << "][" << channelID << "] addr:" << hex << 
                " 0x" << outgoingDataPackets[i]->physicalAddress << dec << 
                " timeAddedToTransactionQueue: " << lbd.timeAddedToTransactionQueue <<
                " timeAddedToCommandQueue: " << lbd.timeAddedToCommandQueue <<
                " timeScheduled: " << lbd.timeScheduled <<
                " timeWriteDone: " << lbd.timeWriteDone <<
                " timeReturned: " << lbd.timeReturned);

            dq.erase(it);
            break;
          }
        }
#endif

        (*ranks)[outgoingDataPackets[i]->rank]->receiveFromBus(outgoingDataPackets[i]);
        outgoingDataPackets[i] = NULL;
      }
    }
  }

  for_each(writeDataCountdown.begin(), writeDataCountdown.end(), [](unsigned &n){ n--; });

  // if any outstanding write data needs to be sent and 
  // the appropriate amount of time (WL) has passed, 
  // then send data on bus 
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    if (writeDataToSend[i] != NULL) {
      writeDataCountdown[i]--;
      if (writeDataCountdown[i] == 0) {
        if (outgoingDataPackets[i] != NULL) {
          ERROR("== Error - Data Bus Collision");
          exit(-1);
        }

        unsigned bank = writeDataToSend[i]->bank;

        outgoingDataPackets[i] = writeDataToSend[i];
        dataCyclesLeft[i] = BL/2;

        writeDataToSend[i] = NULL;
        writeDataCountdown[i] = -1;

        totalTransactions++;
        totalWritesPerBank[SEQUENTIAL(i,bank)]++;
      }
    }
  }

  // if it's time for a refresh, issue a refresh 
  if (refreshCountdown[refreshRank] == 0) {
    commandQueue.needRefresh(refreshRank);
    (*ranks)[refreshRank]->refreshWaiting = true;
    refreshCountdown[refreshRank] = REFRESH_PERIOD/tCK;
    refreshRank = (refreshRank + 1) % NUM_RANKS;
  }
  // if a rank is powered down, power it up in time for a refresh
  else if (powerDown[refreshRank] && (refreshCountdown[refreshRank] <= tXP))
    (*ranks)[refreshRank]->refreshWaiting = true;

  // pop() returns true if there is something valid in poppedBusPacket
  if (commandQueue.pop(&poppedBusPacket)) {
    if (poppedBusPacket->busPacketType == WRITE || poppedBusPacket->busPacketType == WRITE_P) {
      writeDataToSend[poppedBusPacket->rank] = new BusPacket(DATA, poppedBusPacket->physicalAddress, 
          poppedBusPacket->column, poppedBusPacket->row, poppedBusPacket->rank, 
          poppedBusPacket->bank, poppedBusPacket->data);
      writeDataCountdown[poppedBusPacket->rank] = WL;
    }

    //update each bank's state based on the command just popped out of the command queue
    unsigned rank = poppedBusPacket->rank;
    unsigned bank = poppedBusPacket->bank;

    switch (poppedBusPacket->busPacketType) {
      case READ_P:
      case READ:
        if (poppedBusPacket->busPacketType == READ_P) {
          bankStates[rank][bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
              bankStates[rank][bank].nextActivate);
          bankStates[rank][bank].lastCommand = READ_P;
          bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
        } else if (poppedBusPacket->busPacketType == READ) {
          bankStates[rank][bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY,
              bankStates[rank][bank].nextPrecharge);
          bankStates[rank][bank].lastCommand = READ;
        }

        for (unsigned i = 0; i < NUM_RANKS; ++i) {
          for (unsigned j = 0; j < NUM_BANKS; ++j) {
            if (i != poppedBusPacket->rank) {
              if (bankStates[i][j].currentBankState == RowActive) {
                bankStates[i][j].nextRead = max(currentClockCycle + BL/2 + tRTRS,
                    bankStates[i][j].nextRead);
                bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                    bankStates[i][j].nextWrite);
              }
            } else {
              if (BANK_GROUPS_ENABLED) {
                if (poppedBusPacket->bankGroup == j / NUM_BANKS_PER_BANKGROUP) {
                  // Accesses within the same bank group
                  bankStates[i][j].nextRead = max(currentClockCycle + max(tCCDL, BL/2),
                      bankStates[i][j].nextRead);
                  bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                      bankStates[i][j].nextWrite);
                } else {
                  // Accesses to different bank groups
                  bankStates[i][j].nextRead = max(currentClockCycle + max(tCCDS, BL/2),
                      bankStates[i][j].nextRead);
                  bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                      bankStates[i][j].nextWrite);
                }
              } else { // BANK_GROUPS_DISABLED
                bankStates[i][j].nextRead = max(currentClockCycle + max(tCCDS, BL/2),
                    bankStates[i][j].nextRead);
                bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                    bankStates[i][j].nextWrite);
              }
            }
          }
        }

        if (poppedBusPacket->busPacketType == READ_P) {
          //set read/write to nextActivate so the state table will prevent a read or write being
          //issued (in cq.isIssuable()) before the bank state has been changed because of the
          //auto-precharge associated with this command
          bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
          bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
        }

        if (outgoingColCmdPacket != NULL) {
          ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - command bus collision");
          exit(-1);
        }

        outgoingColCmdPacket = poppedBusPacket;
        colCmdCyclesLeft = tCMD;
        break;

      case WRITE_P:
      case WRITE:
        if (poppedBusPacket->busPacketType == WRITE_P) {
          bankStates[rank][bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
              bankStates[rank][bank].nextActivate);
          bankStates[rank][bank].lastCommand = WRITE_P;
          bankStates[rank][bank].stateChangeCountdown = WRITE_TO_PRE_DELAY;
        } else if (poppedBusPacket->busPacketType == WRITE) {
          bankStates[rank][bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
              bankStates[rank][bank].nextPrecharge);
          bankStates[rank][bank].lastCommand = WRITE;
        }

        for (unsigned i = 0; i < NUM_RANKS; ++i) {
          for (unsigned j = 0; j < NUM_BANKS; ++j) {
            if (i != poppedBusPacket->rank) {
              if (bankStates[i][j].currentBankState == RowActive) {
                bankStates[i][j].nextWrite = max(currentClockCycle + BL/2 + tRTRS,
                    bankStates[i][j].nextWrite);
                bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_R,
                    bankStates[i][j].nextRead);
              }
            } else {
              if (BANK_GROUPS_ENABLED) {
                if (poppedBusPacket->bankGroup == j / NUM_BANKS_PER_BANKGROUP) { 
                  // Accesses within the same bank group
                  bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCDL),
                      bankStates[i][j].nextWrite);
                  bankStates[i][j].nextRead = max(currentClockCycle + WL + BL/2 + tWTRL,
                      bankStates[i][j].nextRead);
                } else { 
                  // Accesses to different bank groups
                  bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCDS),
                      bankStates[i][j].nextWrite);
                  bankStates[i][j].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS,
                      bankStates[i][j].nextRead);
                }
              } else { // BANK_GROUPS_DISABLED
                bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCDS),
                    bankStates[i][j].nextWrite);
                bankStates[i][j].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS, 
                    bankStates[i][j].nextRead);
              }
            }
          }
        }

        if (poppedBusPacket->busPacketType == WRITE_P) {
          // set nextRead and nextWrite to nextActivate so the state table will prevent a read or 
          // write from being issued before the bank state has been changed because of the 
          // auto-precharge associated with this command
          bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
          bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
        }

        if (outgoingColCmdPacket != NULL) {
          ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - command bus collision");
          exit(-1);
        }

        outgoingColCmdPacket = poppedBusPacket;
        colCmdCyclesLeft = tCMD;
        break;

      case ACTIVATE:
        bankStates[rank][bank].currentBankState = RowActive;
        bankStates[rank][bank].lastCommand = ACTIVATE;
        bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
        bankStates[rank][bank].nextActivate = max(currentClockCycle + tRC,
            bankStates[rank][bank].nextActivate);
        bankStates[rank][bank].nextPrecharge = max(currentClockCycle + tRAS,
            bankStates[rank][bank].nextPrecharge);

        // if we are using posted-CAS, the next column access can be sooner than normal operation
        bankStates[rank][bank].nextRead = max(currentClockCycle + (tRCD-AL),
            bankStates[rank][bank].nextRead);
        bankStates[rank][bank].nextWrite = max(currentClockCycle + (tRCD-AL),
            bankStates[rank][bank].nextWrite);

        for (unsigned i = 0; i < NUM_BANKS; ++i) {
          if (BANK_GROUPS_ENABLED) {
            if (i != poppedBusPacket->bank) {
              if (poppedBusPacket->bankGroup == i / NUM_BANKS_PER_BANKGROUP)
                // Accesses within the same bank group
                bankStates[rank][i].nextActivate = max(currentClockCycle + tRRDL, 
                    bankStates[rank][i].nextActivate);
              else
                // Accesses to different bank groups
                bankStates[rank][i].nextActivate = max(currentClockCycle + tRRDS, 
                    bankStates[rank][i].nextActivate);
            }
          } else { // BANK_GROUPS_DISABLED
            if (i != poppedBusPacket->bank)
              bankStates[rank][i].nextActivate = max(currentClockCycle + tRRDS, 
                  bankStates[rank][i].nextActivate);
          }
        }

        if (outgoingRowCmdPacket != NULL) {
          ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - command bus collision");
          exit(-1);
        }

        outgoingRowCmdPacket = poppedBusPacket;
        rowCmdCyclesLeft = tCMD;
        break;

      case PRECHARGE:
        bankStates[rank][bank].currentBankState = Precharging;
        bankStates[rank][bank].lastCommand = PRECHARGE;
        bankStates[rank][bank].stateChangeCountdown = tRP;
        bankStates[rank][bank].nextActivate = max(currentClockCycle + tRP,
            bankStates[rank][bank].nextActivate);

        if (outgoingRowCmdPacket != NULL) {
          ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - command bus collision");
          exit(-1);
        }

        outgoingRowCmdPacket = poppedBusPacket;
        rowCmdCyclesLeft = tCMD;
        break;

      case REFRESH:
        for (unsigned i = 0; i < NUM_BANKS; ++i) {
          bankStates[rank][i].nextActivate = currentClockCycle + tRFC;
          bankStates[rank][i].currentBankState = Refreshing;
          bankStates[rank][i].lastCommand = REFRESH;
          bankStates[rank][i].stateChangeCountdown = tRFC;
        }

        if (outgoingRowCmdPacket != NULL) {
          ERROR("[" << stackID << "][" << channelID << "] cycle:" << currentClockCycle << " Error - command bus collision");
          exit(-1);
        }

        outgoingRowCmdPacket = poppedBusPacket;
        rowCmdCyclesLeft = tCMD;
        break;

      default:
        ERROR("== Error - command we shouldn't have of type : " << poppedBusPacket->busPacketType);
        exit(0);
    }

#ifdef DEBUG_LATENCY
    deque<LatencyBreakdown> &dq = latencyBreakdowns[poppedBusPacket->physicalAddress];
    for (auto it = dq.begin(); it != dq.end(); ++it) {
      if (it->timeScheduled == 0) {
        it->timeScheduled = currentClockCycle;
        break;
      }
    }
#endif
  }

  for (unsigned i = 0; i < transactionQueue.size(); ++i) {
    // pop off top transaction from queue
    Transaction *transaction = transactionQueue[i];

    // map address to rank, bank, row, col
    unsigned newChan, newRank, newBank, newRow, newCol;
    addressMapping(transaction->getAddress(), newChan, newRank, newBank, newRow, newCol);

    // break up the transaction into the appropriate commands and add them to the command queue
    if (commandQueue.hasRoomFor(2, newRank)) {
      if (DEBUG_ADDR_MAP) {
        PRINTN("[" << stackID << "][" << channelID << "] "); 
        PRINTN("cycle:" << currentClockCycle << " new transaction 0x" << hex << 
            transaction->getAddress() << dec);
        PRINTN((transaction->getTransactionType() == DATA_READ ? " read " : " write "));
        PRINT("ra:" << newRank << " ba:" << newBank << " ro:" << newRow << " co:" << newCol);
      }

      // remove the transaction from the transaction queue
      transactionQueue.erase(transactionQueue.begin()+i);

      // create an activate command
      BusPacket *rowCommand = new BusPacket(ACTIVATE, transaction->getAddress(), newCol, newRow, 
          newRank, newBank, 0);

      // create a read or write command
      BusPacket *colCommand = new BusPacket(transaction->getBusPacketType(), 
          transaction->getAddress(), newCol, newRow, newRank, newBank, transaction->getData());

#ifdef DEBUG_LATENCY
      deque<LatencyBreakdown> &dq = latencyBreakdowns[transaction->getAddress()];
      for (auto it = dq.begin(); it != dq.end(); ++it) {
        if (it->timeAddedToCommandQueue == 0) {
          it->timeAddedToCommandQueue = currentClockCycle;
          break;
        }
      }
#endif

      // enqueue both
      commandQueue.enqueue(rowCommand);
      commandQueue.enqueue(colCommand);

      // If we have a read, save the transaction so when the data comes back in a bus packet, we can 
      // staple it back into a transaction and return it
      if (transaction->getTransactionType() == DATA_READ)
        pendingReadTransactions.push_back(transaction);
      else
        delete transaction; 

      /* 
       * only allow one transaction to be scheduled per cycle -- this should
       * be a reasonable assumption considering how much logic would be
       * required to schedule multiple entries per cycle (parallel data
       * lines, switching logic, decision logic)
       */
      break;
    } else { // no room, do nothing this cycle
      PRINT("== Warning - No room in command queue" << endl);
    }
  }

  //check for outstanding data to return to the CPU
  if (returnTransaction.size() > 0) {
    if (DEBUG_BUS)
      PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);

    totalTransactions++;

    bool foundMatch = false;
    //find the pending read transaction to calculate latency
    for (unsigned i = 0; i < pendingReadTransactions.size(); ++i) {
      if (pendingReadTransactions[i]->getAddress() == returnTransaction[0]->getAddress()) {
#ifdef DEBUG_LATENCY
        deque<LatencyBreakdown> &dq = latencyBreakdowns[pendingReadTransactions[i]->getAddress()];
        for (auto it = dq.begin(); it != dq.end(); ++it) {
          if (it->isRead && it->timeReadDone != 0) {
            it->timeReturned = currentClockCycle;

            LatencyBreakdown &lbd = *it;
            DEBUG("[" << stackID << "][" << channelID << "] addr:" << hex << 
                " 0x" << pendingReadTransactions[i]->getAddress() << dec << 
                " timeAddedToTransactionQueue: " << lbd.timeAddedToTransactionQueue <<
                " timeAddedToCommandQueue: " << lbd.timeAddedToCommandQueue <<
                " timeScheduled: " << lbd.timeScheduled <<
                " timeReadDone: " << lbd.timeReadDone <<
                " timeReturned: " << lbd.timeReturned);

            dq.erase(it);
            break;
          }
        }
#endif

        returnReadData(pendingReadTransactions[i]);
        delete pendingReadTransactions[i];
        pendingReadTransactions.erase(pendingReadTransactions.begin()+i);
        foundMatch = true; 
        break;
      }
    }

    if (!foundMatch) {
      ERROR("Can't find a matching transaction for 0x" << hex << returnTransaction[0]->getAddress() << dec);
      abort(); 
    }

    delete returnTransaction[0];
    returnTransaction.erase(returnTransaction.begin());
  }

  //decrement refresh counters
  for (unsigned i = 0; i < NUM_RANKS; ++i)
    refreshCountdown[i]--;

  commandQueue.step();
}

bool MemoryController::WillAcceptTransaction()
{
  return (transactionQueue.size() < TRANS_QUEUE_DEPTH);
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans)
{
  if (WillAcceptTransaction()) {
    trans->setTimeAdded(currentClockCycle);
    transactionQueue.push_back(trans);

#ifdef DEBUG_LATENCY
    LatencyBreakdown lbd(currentClockCycle, (trans->getTransactionType() == DATA_READ));
    uint64_t addr = trans->getAddress();
    auto it = latencyBreakdowns.find(addr);
    if (it == latencyBreakdowns.end()) //not found
      latencyBreakdowns[addr] = deque<LatencyBreakdown>();
    latencyBreakdowns[addr].push_back(lbd);
#endif
    return true;
  } else {
    return false;
  }
}

void MemoryController::resetStats()
{
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    for (unsigned j = 0; j < NUM_BANKS; ++j) {
      grandTotalBankAccesses[SEQUENTIAL(i,j)] += totalReadsPerBank[SEQUENTIAL(i,j)] + totalWritesPerBank[SEQUENTIAL(i,j)];
      totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
      totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
      totalEpochLatency[SEQUENTIAL(i,j)] = 0;
    }

    totalReadsPerRank[i] = 0;
    totalWritesPerRank[i] = 0;
  }
}

//retrieves the target 'metric' stat and returns the value as a double in *stat
bool MemoryController::getStats( double *stat, DSIM_STAT metric ){
  double totalBandwidth=0.0;
  uint64_t cyclesElapsed =
    (currentClockCycle % EPOCH_LENGTH == 0) ? EPOCH_LENGTH : currentClockCycle % EPOCH_LENGTH;
  unsigned bytesPerTransaction = JEDEC_DATA_BUS_BITS*BL/8;
  double secondsThisEpoch = (double)cyclesElapsed * tCK * 1E-9;
  vector<double> bandwidth = vector<double>(NUM_RANKS*NUM_BANKS,0.0);

  switch( metric ){
  case TOTAL_BANDWIDTH:
    for (unsigned i = 0; i < NUM_RANKS; ++i) {
      for (unsigned j = 0; j < NUM_BANKS; ++j) {
        bandwidth[SEQUENTIAL(i,j)] =
          (((double)(totalReadsPerBank[SEQUENTIAL(i,j)] +
                     totalWritesPerBank[SEQUENTIAL(i,j)]) *
            (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) /
          secondsThisEpoch;
        totalBandwidth += bandwidth[SEQUENTIAL(i,j)];
      }
    }
    *stat = totalBandwidth;
    return true;
    break;
  case TOTAL_TRANSACTIONS:
  case TOTAL_BYTES_TRANSFERRED:
  case TOTAL_READS:
  case TOTAL_WRITES:
  case PENDING_READ_TRANSACTIONS:
  case PENDING_RTN_TRANSACTIONS:
  default:
    // none of these are double values
    return false;
    break;
  }
}

//retrieves the target 'metric' stat and returns the value as a uint64_t in *stat
bool MemoryController::getStats( uint64_t *stat, DSIM_STAT metric ){
  unsigned bytesPerTransaction = JEDEC_DATA_BUS_BITS*BL/8;
  uint64_t totalReads = 0x00ull;
  uint64_t totalWrites = 0x00ull;

  switch( metric ){
  case TOTAL_BYTES_TRANSFERRED:
    if (operationMode == PseudoChannelMode)
      bytesPerTransaction /= 2;

    *stat = (uint64_t)(totalTransactions * bytesPerTransaction);
    return true;
    break;
  case TOTAL_READS:
    for (unsigned i = 0; i < NUM_RANKS; ++i) {
      for (unsigned j = 0; j < NUM_BANKS; ++j) {
        totalReads += totalReadsPerBank[SEQUENTIAL(i,j)];
        totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
      }
    }
    *stat = totalReads;
    return true;
    break;
  case TOTAL_WRITES:
    for (unsigned i = 0; i < NUM_RANKS; ++i) {
      for (unsigned j = 0; j < NUM_BANKS; ++j) {
        totalWrites += totalWritesPerBank[SEQUENTIAL(i,j)];
      }
    }
    *stat = totalWrites;
    return true;
    break;
  case TOTAL_TRANSACTIONS:
    *stat = (uint64_t)(transactionQueue.size());
    return true;
    break;
  case PENDING_READ_TRANSACTIONS:
    *stat = (uint64_t)(pendingReadTransactions.size());
    return true;
    break;
  case PENDING_RTN_TRANSACTIONS:
    *stat = (uint64_t)(returnTransaction.size());
    return true;
    break;
  case TOTAL_BANDWIDTH:
  default:
    // none of these are double values
    return false;
    break;
  }
}

//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats)
{
  if (!finalStats)
    return;

  uint64_t cyclesElapsed = (currentClockCycle % EPOCH_LENGTH == 0) ? EPOCH_LENGTH : currentClockCycle % EPOCH_LENGTH;
  unsigned bytesPerTransaction = JEDEC_DATA_BUS_BITS*BL/8;
  if (operationMode == PseudoChannelMode)
    bytesPerTransaction /= 2;

  uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
  double secondsThisEpoch = (double)cyclesElapsed * tCK * 1E-9;

  // per bank variables
  //vector<double> averageLatency = vector<double>(NUM_RANKS*NUM_BANKS,0.0);
  vector<double> bandwidth = vector<double>(NUM_RANKS*NUM_BANKS,0.0);

  double totalBandwidth=0.0;
  for (unsigned i = 0; i < NUM_RANKS; ++i) {
    for (unsigned j = 0; j < NUM_BANKS; ++j) {
      bandwidth[SEQUENTIAL(i,j)] = (((double)(totalReadsPerBank[SEQUENTIAL(i,j)] + totalWritesPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
      //averageLatency[SEQUENTIAL(i,j)] = ((float)totalEpochLatency[SEQUENTIAL(i,j)] / (float)(totalReadsPerBank[SEQUENTIAL(i,j)])) * tCK;
      totalBandwidth += bandwidth[SEQUENTIAL(i,j)];
      totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
      totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
    }
  }

  cout.precision(3);
  cout.setf(ios::fixed, ios::floatfield);

  PRINT("Channel " << parentMemorySystem->channelID << " statistics");
  PRINTN(" Total Return Transactions: " << totalTransactions);
  PRINT( " (" << totalBytesTransferred << " bytes) aggregate average bandwidth " << totalBandwidth << "GB/s");

  //double totalAggregateBandwidth = 0.0;  
  for (unsigned r = 0; r < NUM_RANKS; ++r) {
    PRINTN(" Rank " << r << " - ");
    PRINTN(" Reads: " << totalReadsPerRank[r]);
    PRINTN(" (" << totalReadsPerRank[r] * bytesPerTransaction << " bytes)");
    PRINTN(" Writes: " << totalWritesPerRank[r]);
    PRINT(" (" << totalWritesPerRank[r] * bytesPerTransaction << " bytes)");
    //for (unsigned j = 0; j < NUM_BANKS; ++j) {
      //PRINT( "        -Bandwidth / Latency  (Bank " <<j <<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns");
    //}
  }

  resetStats();
}

MemoryController::~MemoryController()
{
  for (auto x: writeDataToSend) delete x;
  for (auto x: returnTransaction) delete x;
  for (auto x: pendingReadTransactions) delete x;
}
