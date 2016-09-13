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

using namespace std;

namespace DRAMSim {

unsigned sliceLowerBits(uint64_t& addr, unsigned bits)
{
  unsigned res = addr & ((1 << bits) - 1);
  addr >>= bits;
  return res;
}

void addressMapping(uint64_t addr, unsigned &chn, unsigned &rnk, unsigned &bnk, unsigned &row, 
    unsigned &col)
{
#ifdef DEBUG_BUILD
  uint64_t addr_old = addr;
#endif

  unsigned tx_size = TRANSACTION_SIZE;
  unsigned tx_bits = log2(tx_size);

  unsigned chn_bits = NUM_CHANS_LOG;
  unsigned rnk_bits = NUM_RANKS_LOG;
  unsigned bnk_bits = NUM_BANKS_LOG;
  unsigned row_bits = NUM_ROWS_LOG;
  unsigned col_bits = NUM_COLS_LOG;

  // clear the lowest tx_bits since each transaction size is 2^tx_bits
  addr >>= tx_bits;

  // perform various address mapping schemes
  if (addressMappingScheme == RoBaRaCoCh) { // row:bank:rank:col:chan
    chn = sliceLowerBits(addr, chn_bits);
    col = sliceLowerBits(addr, col_bits);
    rnk = sliceLowerBits(addr, rnk_bits);
    bnk = sliceLowerBits(addr, bnk_bits);
    row = sliceLowerBits(addr, row_bits);
  } else if (addressMappingScheme == ChRaBaRoCo) { // chan:rank:bank:row:col
    col = sliceLowerBits(addr, col_bits);
    row = sliceLowerBits(addr, row_bits);
    bnk = sliceLowerBits(addr, bnk_bits);
    rnk = sliceLowerBits(addr, rnk_bits);
    chn = sliceLowerBits(addr, chn_bits);
  } else {
    ERROR("error - unknown address mapping scheme");
    exit(-1);
  }

  if (DEBUG_ADDR_MAP) {
    DEBUG("addr:0x" << hex << addr_old << dec << " mapped to chan:" << chn << " rank:" << rnk << 
        " bank:" << bnk << " row:" << row << " col:" << col); 
  }
}

};
