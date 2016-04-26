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
#include "SystemConfiguration.h"
#include "AddressMapping.h"

namespace DRAMSim {

void addressMapping(uint64_t physicalAddress, unsigned &newTransactionChan, unsigned &newTransactionRank, unsigned &newTransactionBank, unsigned &newTransactionRow, unsigned &newTransactionColumn)
{
  uint64_t tempA, tempB;

  unsigned transactionSize = TRANSACTION_SIZE;
  uint64_t transactionMask =  transactionSize - 1; //ex: (64 bit bus width) x (8 Burst Length) - 1 = 64 bytes - 1 = 63 = 0x3f mask

  unsigned channelBitWidth = NUM_CHANS_LOG;
  unsigned rankBitWidth = NUM_RANKS_LOG;
  unsigned bankBitWidth = NUM_BANKS_LOG;
  unsigned rowBitWidth = NUM_ROWS_LOG;
  unsigned colBitWidth = NUM_COLS_LOG;
  unsigned byteOffsetWidth = BYTE_OFFSET_WIDTH;

  // Since we're assuming that a request is for BL*BUS_WIDTH, the bottom bits
  // of this address *should* be all zeros if it's not, issue a warning
  if ((physicalAddress & transactionMask) != 0) {
    DEBUG("WARNING: address 0x"<<std::hex<<physicalAddress<<std::dec<<" is not aligned to the request size of "<<transactionSize); 
  }

  // taking into account 256-bit prefetch architecture
  physicalAddress >>= byteOffsetWidth;

  // taking into account transaction size
  physicalAddress >>= dramsim_log2(transactionSize);

  if (DEBUG_ADDR_MAP) {
    DEBUG("Bit widths: ch:"<<channelBitWidth<<" ra:"<<rankBitWidth<<" ba:"<<bankBitWidth
        <<" ro:"<<rowBitWidth<<" co:"<<colBitWidth <<" off:"<<byteOffsetWidth 
        << " Total:"<< (channelBitWidth + rankBitWidth + bankBitWidth + rowBitWidth + colBitWidth + byteOffsetWidth));
  }

  //perform various address mapping schemes
  if (addressMappingScheme == RoBaRaCoCh) {
    // row:bank:rank:col:chan
    tempA = physicalAddress;
    physicalAddress = physicalAddress >> channelBitWidth;
    tempB = physicalAddress << channelBitWidth;
    newTransactionChan = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> colBitWidth;
    tempB = physicalAddress << colBitWidth;
    newTransactionColumn = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> rankBitWidth;
    tempB = physicalAddress << rankBitWidth;
    newTransactionRank = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> bankBitWidth;
    tempB = physicalAddress << bankBitWidth;
    newTransactionBank = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> rowBitWidth;
    tempB = physicalAddress << rowBitWidth;
    newTransactionRow = tempA ^ tempB;
  }
  else if (addressMappingScheme == ChRaBaRoCo) {
    // chan:rank:bank:row:col
    tempA = physicalAddress;
    physicalAddress = physicalAddress >> colBitWidth;
    tempB = physicalAddress << colBitWidth;
    newTransactionColumn = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> rowBitWidth;
    tempB = physicalAddress << rowBitWidth;
    newTransactionRow = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> bankBitWidth;
    tempB = physicalAddress << bankBitWidth;
    newTransactionBank = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> rankBitWidth;
    tempB = physicalAddress << rankBitWidth;
    newTransactionRank = tempA ^ tempB;

    tempA = physicalAddress;
    physicalAddress = physicalAddress >> channelBitWidth;
    tempB = physicalAddress << channelBitWidth;
    newTransactionChan = tempA ^ tempB;
  }
  else {
    ERROR("== Error - Unknown Address Mapping Scheme");
    exit(-1);
  }

  if (DEBUG_ADDR_MAP) {
    DEBUG("Mapped Ch="<<newTransactionChan<<" Rank="<<newTransactionRank
        <<" Bank="<<newTransactionBank<<" Row="<<newTransactionRow
        <<" Col="<<newTransactionColumn<<"\n"); 
  }
}

};
