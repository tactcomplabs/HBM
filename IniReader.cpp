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

#include "IniReader.h"

using namespace std;

// these are the values that are extern'd in SystemConfig.h so that they
// have global scope even though they are set by IniReader

unsigned NUM_BANKGROUPS;
unsigned NUM_BANKS;
unsigned NUM_BANKS_LOG;
unsigned NUM_CHANS;
unsigned NUM_CHANS_LOG;
unsigned NUM_ROWS;
unsigned NUM_ROWS_LOG;
unsigned NUM_COLS;
unsigned NUM_COLS_LOG;
unsigned PREFETCH_SIZE;
unsigned PAGE_SIZE;
unsigned BYTE_OFFSET_WIDTH;
unsigned TRANSACTION_SIZE;

unsigned REFRESH_PERIOD;
float tCK;
unsigned CL;
unsigned AL;
unsigned BL;
unsigned tRAS;
unsigned tRCD;
unsigned tRRDS;
unsigned tRRDL;
unsigned tRC;
unsigned tRP;
unsigned tCCDS;
unsigned tCCDL;
unsigned tRTP;
unsigned tWTRS;
unsigned tWTRL;
unsigned tWR;
unsigned tRTRS;
unsigned tRFC;
unsigned tFAW;
unsigned tCKE;
unsigned tXP;
unsigned tCMD;

//in bytes
unsigned JEDEC_DATA_BUS_BITS;

//Memory Controller related parameters
unsigned TRANS_QUEUE_DEPTH;
unsigned CMD_QUEUE_DEPTH;

//cycles within an epoch
unsigned EPOCH_LENGTH;

//row accesses allowed before closing (open page)
unsigned TOTAL_ROW_ACCESSES;

// strings and their associated enums
string ROW_BUFFER_POLICY;
string SCHEDULING_POLICY;
string ADDRESS_MAPPING_SCHEME;
string OPERATION_MODE;
bool BANK_GROUPS_ENABLED;

bool DEBUG_TRANS_Q;
bool DEBUG_CMD_Q;
bool DEBUG_ADDR_MAP;
bool DEBUG_BANKSTATE;
bool DEBUG_BUS;
bool DEBUG_BANKS;
bool DEBUG_POWER;

bool DEBUG_INI_READER=false;

namespace DRAMSim
{
RowBufferPolicy rowBufferPolicy;
SchedulingPolicy schedulingPolicy;
AddressMappingScheme addressMappingScheme;
OperationMode operationMode;


//Map the string names to the variables they set
static ConfigMap configMap[] =
{
  //DEFINE_UINT_PARAM -- see IniReader.h
  DEFINE_UINT_PARAM(NUM_BANKGROUPS,DEV_PARAM),
  DEFINE_UINT_PARAM(NUM_BANKS,DEV_PARAM),
  DEFINE_UINT_PARAM(NUM_ROWS,DEV_PARAM),
  DEFINE_UINT_PARAM(NUM_COLS,DEV_PARAM),
  DEFINE_UINT_PARAM(PREFETCH_SIZE,DEV_PARAM),
  DEFINE_UINT_PARAM(PAGE_SIZE,DEV_PARAM),
  DEFINE_UINT_PARAM(REFRESH_PERIOD,DEV_PARAM),
  DEFINE_FLOAT_PARAM(tCK,DEV_PARAM),
  DEFINE_UINT_PARAM(CL,DEV_PARAM),
  DEFINE_UINT_PARAM(AL,DEV_PARAM),
  DEFINE_UINT_PARAM(BL,DEV_PARAM),
  DEFINE_UINT_PARAM(tRAS,DEV_PARAM),
  DEFINE_UINT_PARAM(tRCD,DEV_PARAM),
  DEFINE_UINT_PARAM(tRRDS,DEV_PARAM),
  DEFINE_UINT_PARAM(tRRDL,DEV_PARAM),
  DEFINE_UINT_PARAM(tRC,DEV_PARAM),
  DEFINE_UINT_PARAM(tRP,DEV_PARAM),
  DEFINE_UINT_PARAM(tCCDS,DEV_PARAM),
  DEFINE_UINT_PARAM(tCCDL,DEV_PARAM),
  DEFINE_UINT_PARAM(tRTP,DEV_PARAM),
  DEFINE_UINT_PARAM(tWTRS,DEV_PARAM),
  DEFINE_UINT_PARAM(tWTRL,DEV_PARAM),
  DEFINE_UINT_PARAM(tWR,DEV_PARAM),
  DEFINE_UINT_PARAM(tRTRS,DEV_PARAM),
  DEFINE_UINT_PARAM(tRFC,DEV_PARAM),
  DEFINE_UINT_PARAM(tFAW,DEV_PARAM),
  DEFINE_UINT_PARAM(tCKE,DEV_PARAM),
  DEFINE_UINT_PARAM(tXP,DEV_PARAM),
  DEFINE_UINT_PARAM(tCMD,DEV_PARAM),
  
  DEFINE_UINT_PARAM(NUM_CHANS,SYS_PARAM),
  DEFINE_UINT_PARAM(JEDEC_DATA_BUS_BITS,SYS_PARAM),

  //Memory Controller related parameters
  DEFINE_UINT_PARAM(TRANS_QUEUE_DEPTH,SYS_PARAM),
  DEFINE_UINT_PARAM(CMD_QUEUE_DEPTH,SYS_PARAM),

  DEFINE_UINT_PARAM(EPOCH_LENGTH,SYS_PARAM),
  DEFINE_UINT_PARAM(TOTAL_ROW_ACCESSES,SYS_PARAM),
  DEFINE_STRING_PARAM(ROW_BUFFER_POLICY,SYS_PARAM),
  DEFINE_STRING_PARAM(SCHEDULING_POLICY,SYS_PARAM),
  DEFINE_STRING_PARAM(ADDRESS_MAPPING_SCHEME,SYS_PARAM),
  DEFINE_STRING_PARAM(OPERATION_MODE,SYS_PARAM),
  DEFINE_BOOL_PARAM(BANK_GROUPS_ENABLED,SYS_PARAM),

  // debug flags
  DEFINE_BOOL_PARAM(DEBUG_TRANS_Q,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_CMD_Q,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_ADDR_MAP,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_BANKSTATE,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_BUS,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_BANKS,SYS_PARAM),
  DEFINE_BOOL_PARAM(DEBUG_POWER,SYS_PARAM),
  {"", NULL, UINT, SYS_PARAM, false} // tracer value to signify end of list; if you delete it, epic fail will result
};

void IniReader::SetKey(string key, string valueString, bool isSystemParam, size_t lineNumber)
{
  size_t i;
  unsigned intValue;
  uint64_t int64Value;
  float floatValue;

  for (i=0; configMap[i].variablePtr != NULL; i++)
  {
    istringstream iss(valueString);
    // match up the string in the config map with the key we parsed
    if (key.compare(configMap[i].iniKey) == 0)
    {
      switch (configMap[i].variableType)
      {
        //parse and set each type of variable
      case UINT:
        if ((iss >> dec >> intValue).fail())
        {
          ERROR("could not parse line "<<lineNumber<<" (non-numeric value '"<<valueString<<"')?");
        }
        *((unsigned *)configMap[i].variablePtr) = intValue;
        if (DEBUG_INI_READER)
        {
          DEBUG("\t - SETTING "<<configMap[i].iniKey<<"="<<intValue);
        }
        break;
      case UINT64:
        if ((iss >> dec >> int64Value).fail())
        {
          ERROR("could not parse line "<<lineNumber<<" (non-numeric value '"<<valueString<<"')?");
        }
        *((uint64_t *)configMap[i].variablePtr) = int64Value;
        if (DEBUG_INI_READER)
        {
          DEBUG("\t - SETTING "<<configMap[i].iniKey<<"="<<int64Value);
        }
        break;
      case FLOAT:
        if ((iss >> dec >> floatValue).fail())
        {
          ERROR("could not parse line "<<lineNumber<<" (non-numeric value '"<<valueString<<"')?");
        }
        *((float *)configMap[i].variablePtr) = floatValue;
        if (DEBUG_INI_READER)
        {
          DEBUG("\t - SETTING "<<configMap[i].iniKey<<"="<<floatValue);
        }
        break;
      case STRING:
        *((string *)configMap[i].variablePtr) = string(valueString);
        if (DEBUG_INI_READER)
        {
          DEBUG("\t - SETTING "<<configMap[i].iniKey<<"="<<valueString);
        }

        break;
      case BOOL:
        if (valueString == "true" || valueString == "1")
        {
          *((bool *)configMap[i].variablePtr) = true;
        }
        else
        {
          *((bool *)configMap[i].variablePtr) = false;
        }
      }
      // lineNumber == 0 implies that this is an override parameter from the command line, so don't bother doing these checks
      if (lineNumber > 0)
      {
        if (isSystemParam && configMap[i].parameterType == DEV_PARAM)
        {
          DEBUG("WARNING: Found device parameter "<<configMap[i].iniKey<<" in system config file");
        }
        else if (!isSystemParam && configMap[i].parameterType == SYS_PARAM)
        {
          DEBUG("WARNING: Found system parameter "<<configMap[i].iniKey<<" in device config file");
        }
      }
      // use the pointer stored in the config map to set the value of the variable
      // to make sure all parameters are in the ini file
      configMap[i].wasSet = true;
      break;
    }
  }

  if (configMap[i].variablePtr == NULL)
  {
    DEBUG("WARNING: UNKNOWN KEY '"<<key<<"' IN INI FILE");
  }
}

void IniReader::ReadIniFile(string filename, bool isSystemFile)
{
  ifstream iniFile;
  string line;
  string key,valueString;

  size_t commentIndex, equalsIndex;
  size_t lineNumber=0;

  iniFile.open(filename.c_str());
  if (iniFile.is_open()) {
    while (!iniFile.eof()) {
      lineNumber++;
      getline(iniFile, line);

      //this can happen if the filename is actually a directory
      if (iniFile.bad()) {
        ERROR("Cannot read ini file '"<<filename<<"'");
        exit(-1);
      }

      // skip zero-length lines
      if (line.size() == 0) {
        // DEBUG("Skipping blank line "<<lineNumber);
        continue;
      }

      //search for a comment char
      if ((commentIndex = line.find_first_of(";")) != string::npos) {
        //if the comment char is the first char, ignore the whole line
        if (commentIndex == 0) {
          //DEBUG("Skipping comment line "<<lineNumber);
          continue;
        }

        //DEBUG("Truncating line at comment"<<line[commentIndex-1]);
        //truncate the line at first comment before going on
        line = line.substr(0,commentIndex);
      }

      // trim off the end spaces that might have been between the value and comment char
      size_t whiteSpaceEndIndex;
      if ((whiteSpaceEndIndex = line.find_last_not_of(" \t")) != string::npos) {
        line = line.substr(0,whiteSpaceEndIndex+1);
      }

      // at this point line should be a valid, commentless string

      // a line has to have an equals sign
      if ((equalsIndex = line.find_first_of("=")) == string::npos) {
        ERROR("Malformed Line "<<lineNumber<<" (missing equals)");
        abort();
      }

      size_t strlen = line.size();

      // all characters before the equals are the key
      key = line.substr(0, equalsIndex);

      // all characters after the equals are the value
      valueString = line.substr(equalsIndex+1,strlen-equalsIndex);

      IniReader::SetKey(key, valueString, isSystemFile, lineNumber);
      // got to the end of the config map without finding the key
    }
  } else {
    ERROR ("Unable to load ini file "<<filename);
    abort();
  }

  /* precompute frequently used values */
  NUM_BANKS_LOG = log2(NUM_BANKS);
  NUM_CHANS_LOG = log2(NUM_CHANS);
  NUM_ROWS_LOG  = log2(NUM_ROWS);
  NUM_COLS_LOG  = log2(NUM_COLS);

  // after system.ini is read
  if (JEDEC_DATA_BUS_BITS > 0) {
    BYTE_OFFSET_WIDTH = log2(PAGE_SIZE / NUM_COLS);
    TRANSACTION_SIZE = JEDEC_DATA_BUS_BITS / 8 * BL;
    if (OPERATION_MODE == "pseudo_channel_mode")
      TRANSACTION_SIZE /= 2;
  }
}

bool IniReader::CheckIfAllSet()
{
  // check to make sure all parameters that we exepected were set
  for (size_t i = 0; configMap[i].variablePtr != NULL; ++i) {
    if (!configMap[i].wasSet) {
      DEBUG("WARNING: KEY " << configMap[i].iniKey << " NOT FOUND IN INI FILE.");
      switch (configMap[i].variableType) {
        //the string and bool values can be defaulted, but generally we need all the numeric values
        //to be set to continue
      case UINT:
      case UINT64:
      case FLOAT:
        ERROR("Cannot continue without key '"<<configMap[i].iniKey<<"' set.");
        return false;
        break;
      case BOOL:
        *((bool *)configMap[i].variablePtr) = false;
        DEBUG("\tSetting Default: "<<configMap[i].iniKey<<"=false");
        break;
      case STRING:
        break;
      }
    }
  }
  return true;
}

void IniReader::InitEnumsFromStrings()
{
  if (ADDRESS_MAPPING_SCHEME == "ChRaBaRoCo")
    addressMappingScheme = ChRaBaRoCo;
  else if (ADDRESS_MAPPING_SCHEME == "RoBaRaCoCh")
    addressMappingScheme = RoBaRaCoCh;
  else {
    cout << "WARNING: unknown address mapping scheme '" << ADDRESS_MAPPING_SCHEME << "'; valid "
      "values are 'ChRaBaRoCo' or 'RoBaRaCoCh', defaulting to RoBaRaCoCh" << endl;
    addressMappingScheme = RoBaRaCoCh;
  }

  if (ROW_BUFFER_POLICY == "open_page")
    rowBufferPolicy = OpenPage;
  else if (ROW_BUFFER_POLICY == "close_page")
    rowBufferPolicy = ClosePage;
  else {
    cout << "WARNING: unknown row buffer policy '" << ROW_BUFFER_POLICY << "'; valid values are "
      "'open_page' or 'close_page', defaulting to Close Page." << endl;
    rowBufferPolicy = ClosePage;
  }

  if (SCHEDULING_POLICY == "rank_then_bank_round_robin")
    schedulingPolicy = RankThenBankRoundRobin;
  else if (SCHEDULING_POLICY == "bank_then_rank_round_robin")
    schedulingPolicy = BankThenRankRoundRobin;
  else {
    cout << "WARNING: Unknown scheduling policy '" << SCHEDULING_POLICY << "'; valid options are "
      "'rank_then_bank_round_robin' or 'bank_then_rank_round_robin', defaulting to Bank Then Rank "
      "Round Robin" << endl;
    schedulingPolicy = BankThenRankRoundRobin;
  }

  if (OPERATION_MODE == "legacy_mode")
    operationMode = LegacyMode;
  else if (OPERATION_MODE == "pseudo_channel_mode")
    operationMode = PseudoChannelMode;
  else {
    cout << "WARNING: unknown operation mode '" << OPERATION_MODE << "'; valid values are "
      "'legacy_mode' or 'pseudo_channel_mode', defaulting to Legacy Mode." << endl;
    operationMode = LegacyMode;
  }
}

} // namespace DRAMSim
