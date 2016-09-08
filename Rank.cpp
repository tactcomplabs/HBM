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

#include "Rank.h"
#include "MemoryController.h"

using namespace std;
using namespace DRAMSim;

Rank::Rank() :
  id(-1),
  isPowerDown(false),
  refreshWaiting(false),
  readReturnCountdown(0),
  bankStates(NUM_BANKS, BankState())
{
  memoryController = NULL;
  outgoingDataPacket = NULL;
  dataCyclesLeft = 0;
  currentClockCycle = 0;
}

Rank::~Rank()
{
  for (auto x: readReturnPacket) delete x;
  if (outgoingDataPacket) delete outgoingDataPacket; 
}

void Rank::receiveFromBus(BusPacket *packet)
{
  switch (packet->busPacketType) {
    case READ:
      // make sure a read is allowed
      if (bankStates[packet->bank].currentBankState != RowActive || 
          currentClockCycle < bankStates[packet->bank].nextRead || 
          packet->row != bankStates[packet->bank].openRowAddress) {
        packet->print();
        ERROR("== Error - Rank " << id << " received a READ when not allowed");
        exit(0);
      }

      // update state table
      bankStates[packet->bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY, 
          bankStates[packet->bank].nextPrecharge);
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        if (BANK_GROUPS_ENABLED) {
          if (packet->bankGroup == i / NUM_BANKS_PER_BANKGROUP) {
            // Accesses within the same bank group
            bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDL), 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                bankStates[i].nextWrite);
          } else {
            // Accesses to different bank groups
            bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDS), 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                bankStates[i].nextWrite);
          }
        } else { // BANK_GROUPS_DISABLED
          bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDS), 
              bankStates[i].nextRead);
          bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
              bankStates[i].nextWrite);
        }
      }

      packet->busPacketType = DATA;
      readReturnPacket.push_back(packet);
      readReturnCountdown.push_back(RL);
      break;

    case READ_P:
      //make sure a read is allowed
      if (bankStates[packet->bank].currentBankState != RowActive || 
          currentClockCycle < bankStates[packet->bank].nextRead || 
          packet->row != bankStates[packet->bank].openRowAddress) {
        ERROR("== Error - Rank " << id << " received a READ_P when not allowed");
        exit(-1);
      }

      //update state table
      bankStates[packet->bank].currentBankState = Idle;
      bankStates[packet->bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
          bankStates[packet->bank].nextActivate);
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        // set next read/write for all banks - including current 
        // should not matter since it's now idle
        if (BANK_GROUPS_ENABLED) {
          if (packet->bankGroup == i / NUM_BANKS_PER_BANKGROUP) {
            // Accesses within the same bank group
            bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDL), 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                bankStates[i].nextWrite);
          } else {
            // Accesses to different bank groups
            bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDS), 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                bankStates[i].nextWrite);
          }
        } else { // BANK_GROUPS_DISABLED
          bankStates[i].nextRead = max(currentClockCycle + max(BL/2, tCCDS), 
              bankStates[i].nextRead);
          bankStates[i].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
              bankStates[i].nextWrite);
        }
      }

      packet->busPacketType = DATA;
      readReturnPacket.push_back(packet);
      readReturnCountdown.push_back(RL);
      break;

    case WRITE:
      // make sure a write is allowed
      if (bankStates[packet->bank].currentBankState != RowActive || 
          currentClockCycle < bankStates[packet->bank].nextWrite || 
          packet->row != bankStates[packet->bank].openRowAddress) {
        ERROR("== Error - Rank " << id << " received a WRITE when not allowed");
        bankStates[packet->bank].print();
        exit(0);
      }

      // update state table
      bankStates[packet->bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
          bankStates[packet->bank].nextPrecharge);
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        if (BANK_GROUPS_ENABLED) {
          if (packet->bankGroup == i / NUM_BANKS_PER_BANKGROUP) {
            // Accesses within the same bank group
            bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRL, 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDL), 
                bankStates[i].nextWrite);
          } else {
            // Accesses to different bank groups
            bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS, 
                bankStates[i].nextRead);
            bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDS), 
                bankStates[i].nextWrite);
          }
        } else { // BANK_GROUPS_DISABLED
          bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS, 
              bankStates[i].nextRead);
          bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDS), 
              bankStates[i].nextWrite);
        }
      }

      // take note of where data is going when it arrives
      incomingWriteBank = packet->bank;
      incomingWriteRow = packet->row;
      incomingWriteColumn = packet->column;
      delete(packet);
      break;

    case WRITE_P:
      //make sure a write is allowed
      if (bankStates[packet->bank].currentBankState != RowActive || 
          currentClockCycle < bankStates[packet->bank].nextWrite || 
          packet->row != bankStates[packet->bank].openRowAddress) {
        ERROR("== Error - Rank " << id << " received a WRITE_P when not allowed");
        exit(0);
      }

      //update state table
      bankStates[packet->bank].currentBankState = Idle;
      bankStates[packet->bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
          bankStates[packet->bank].nextActivate);
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        if (BANK_GROUPS_ENABLED) {
          if (packet->bankGroup == i / NUM_BANKS_PER_BANKGROUP) {
            // Accesses within the same bank group
            bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDL), 
                bankStates[i].nextWrite);
            bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRL, 
                bankStates[i].nextRead);
          } else {
            // Accesses to different bank groups
            bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDS), 
                bankStates[i].nextWrite);
            bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS, 
                bankStates[i].nextRead);
          }
        } else { //BANK_GROUPS_DISABLED
          bankStates[i].nextWrite = max(currentClockCycle + max(BL/2, tCCDS), 
              bankStates[i].nextWrite);
          bankStates[i].nextRead = max(currentClockCycle + WL + BL/2 + tWTRS, 
              bankStates[i].nextRead);
        }
      }

      //take note of where data is going when it arrives
      incomingWriteBank = packet->bank;
      incomingWriteRow = packet->row;
      incomingWriteColumn = packet->column;
      delete(packet);
      break;

    case ACTIVATE:
      //make sure activate is allowed
      if (bankStates[packet->bank].currentBankState != Idle || 
          currentClockCycle < bankStates[packet->bank].nextActivate) {
        ERROR("== Error - Rank " << id << " received an ACT when not allowed");
        packet->print();
        bankStates[packet->bank].print();
        exit(0);
      }

      bankStates[packet->bank].currentBankState = RowActive;
      bankStates[packet->bank].nextActivate = currentClockCycle + tRC;
      bankStates[packet->bank].openRowAddress = packet->row;

      //if AL is greater than one, then posted-cas is enabled - handle accordingly
      if (AL > 0) {
        bankStates[packet->bank].nextWrite = currentClockCycle + (tRCD-AL);
        bankStates[packet->bank].nextRead = currentClockCycle + (tRCD-AL);
      } else {
        bankStates[packet->bank].nextWrite = currentClockCycle + (tRCD-AL);
        bankStates[packet->bank].nextRead = currentClockCycle + (tRCD-AL);
      }

      bankStates[packet->bank].nextPrecharge = currentClockCycle + tRAS;
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        if (BANK_GROUPS_ENABLED) {
          if (i != packet->bank) {
            if (packet->bankGroup == i / NUM_BANKS_PER_BANKGROUP)
              // Accesses within the same bank group
              bankStates[i].nextActivate = max(currentClockCycle + tRRDL, 
                  bankStates[i].nextActivate);
            else
              // Accesses to different bank groups
              bankStates[i].nextActivate = max(currentClockCycle + tRRDS, 
                  bankStates[i].nextActivate);
          }
        } else { // BANK_GROUPS_DISABLED
          if (i != packet->bank)
            bankStates[i].nextActivate = max(currentClockCycle + tRRDS, bankStates[i].nextActivate);
        }
      }
      delete(packet); 
      break;

    case PRECHARGE:
      //make sure precharge is allowed
      if (bankStates[packet->bank].currentBankState != RowActive || 
          currentClockCycle < bankStates[packet->bank].nextPrecharge) {
        ERROR("== Error - Rank " << id << " received a PRE when not allowed");
        exit(0);
      }

      bankStates[packet->bank].currentBankState = Idle;
      bankStates[packet->bank].nextActivate = max(currentClockCycle + tRP,
          bankStates[packet->bank].nextActivate);
      delete(packet); 
      break;

    case REFRESH:
      refreshWaiting = false;
      for (unsigned i = 0; i < NUM_BANKS; ++i) {
        if (bankStates[i].currentBankState != Idle) {
          ERROR("== Error - Rank " << id << " received a REF when not allowed");
          exit(0);
        }
        bankStates[i].nextActivate = currentClockCycle + tRFC;
      }
      delete(packet); 
      break;

    case DATA:
      delete(packet);
      break;

    default:
      ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
      exit(0);
      break;
  }
}

void Rank::update()
{
  // An outgoing packet is one that is currently sending on the bus
  // do the book keeping for the packet's time left on the bus
  if (outgoingDataPacket != NULL) {
    dataCyclesLeft--;
    if (dataCyclesLeft == 0) {
      //if the packet is done on the bus, call receiveFromBus and free up the bus
      memoryController->receiveFromBus(outgoingDataPacket);
      outgoingDataPacket = NULL;
    }
  }
  
  // decrement the counter for all packets waiting to be sent back
  for_each(readReturnCountdown.begin(), readReturnCountdown.end(), [](unsigned &n){ n--; });

  if (readReturnCountdown.size() > 0) {
    if (readReturnCountdown[0] == 0) {
      // RL time has passed since the read was issued; this packet is ready to go out on the bus
      outgoingDataPacket = readReturnPacket[0];
      dataCyclesLeft = BL/2;

      // remove the packet from the ranks
      readReturnPacket.erase(readReturnPacket.begin());
      readReturnCountdown.erase(readReturnCountdown.begin());
    }
  }
}

//power down the rank
void Rank::powerDown()
{
  //perform checks
  for (unsigned i = 0; i < NUM_BANKS; ++i) {
    if (bankStates[i].currentBankState != Idle) {
      ERROR("== Error - Trying to power down rank " << id << " while not all banks are idle");
      exit(0);
    }
    
    bankStates[i].nextPowerUp = currentClockCycle + tCKE;
    bankStates[i].currentBankState = PowerDown;
  }
  
  isPowerDown = true;
}

//power up the rank
void Rank::powerUp()
{
  if (!isPowerDown) {
    ERROR("== Error - Trying to power up rank " << id << " while it is not already powered down");
    exit(0);
  }
  
  isPowerDown = false;
  
  for (unsigned i = 0; i < NUM_BANKS; ++i) {
    if (bankStates[i].nextPowerUp > currentClockCycle) {
      ERROR("== Error - Trying to power up rank " << id << " before we're allowed to");
      ERROR(bankStates[i].nextPowerUp << "    " << currentClockCycle);
      exit(0);
    }
    bankStates[i].nextActivate = currentClockCycle + tXP;
    bankStates[i].currentBankState = Idle;
  }
}
