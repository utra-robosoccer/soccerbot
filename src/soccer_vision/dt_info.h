/*
 * Copyright 1994-2002 The MathWorks, Inc.
 *
 *
 * Abstract:
 *   typedef for data type transitions vectors.  Included by MODEL.dt
 */

#ifndef _DTINFO_H_
#define _DTINFO_H_

/* Data type transition */
typedef struct DataTypeTransition_tag {
  char_T *baseAddr; /* starting address of the transition  */
  int_T  dataType;
  int_T  isComplex; /* elements of this region are complex */
  int_T  nEls;
} DataTypeTransition;

/* Data type transition table */
typedef struct DataTypeTransitionTable_tag {
  uint_T             numTransitions;     /* number of transitions in table  */
  DataTypeTransition *transitions;       /* base address of transition table*/
} DataTypeTransitionTable;

/*
 * The model's data type transition info structure is attached
 * to the SimStruct:
 *
 * DataTypeTransInfo *dtInfo = ssGetModelMappingInfo(S);
 *
 * Additional information is accessed via:
 * 
 * uint_T NumDataTypes     = dtGetNumDataTypes(dtInfo);
 * uint_T DataTypeSizes[]  = dtGetDataTypeSizes(dtInfo);
 * char_T *DataTypeNames[] = dtGetDataTypeNames(dtInfo);
 *
 * DataTypeTransTable *B     = dtGetBIODataTypeTrans(dtInfo);
 * DataTypeTransTable *P     = dtGetParamDataTypeTrans(dtInfo);
 * DataTypeTransTable *DWork = dtGetDWorkDataTypeTrans(dtInfo);
 * DataTypeTransTable *Xd    = dtGetDiscStatesDataTypeTrans(dtInfo);
 * DataTypeTransTable *U     = dtGetExternalInputsDataTypeTrans(dtInfo);
 * DataTypeTransTable *Y     = dtGetExternalOutputsDataTypeTrans(dtInfo);
 *
 * And, using the block outputs as an example
 * 
 * uint_T              NumBIOTransitions  = dtGetNumTransitions(B);
 * DataTypeTransition  *BIOTransitions    = dtGetTransitions(B);
 * char_T              *address           = dtTransGetAddress(B, idx);
 * int_T               dataType           = dtTransGetDataType(B, idx);
 * int_T               isComplex          = dtTransGetComplexFlag(B, idx);
 *
 * where, idx is the index into the block outputs transition table.
 * For example, dtTransGetComplexFlag(B, 5) indicates whether the fifth
 * transition represents a complex region of the block outputs structure
 * structure and dtTransGetAddress(B, 5) is the absolute base address of
 * the region.
 */
 
typedef struct DataTypeTransInfo_tag {
  uint_T         numDataTypes;    /* number of data types in model    */
  uint_T         *dataTypeSizes;  /* data types (bytes) of data types */
  char_T  const **dataTypeNames;  /* names of data types              */

  DataTypeTransitionTable *BTransTable;     /* block outputs */
  DataTypeTransitionTable *PTransTable;     /* parameters */
  DataTypeTransitionTable *DWorkTransTable; /* data type work vector */
  DataTypeTransitionTable *XdTransTable;    /* discrete states */
  DataTypeTransitionTable *UTransTable;     /* model inputs */
  DataTypeTransitionTable *YTransTable;     /* model outputs */
} DataTypeTransInfo;

#define dtGetNumDataTypes(dtInfo)   ((uint_T)((dtInfo)->numDataTypes))
#define dtGetDataTypeSizes(dtInfo)  ((dtInfo)->dataTypeSizes)
#define dtGetDataTypeNames(dtInfo)  ((dtInfo)->dataTypeNames)

#define dtGetBIODataTypeTrans(dtInfo)             ((dtInfo)->BTransTable)
#define dtGetParamDataTypeTrans(dtInfo)           ((dtInfo)->PTransTable)
#define dtGetDWorkDataTypeTrans(dtInfo)           ((dtInfo)->DWorkTransTable)
#define dtGetDiscStatesDataTypeTrans(dtInfo)      ((dtInfo)->XdTransTable)
#define dtGetExternalInputsDataTypeTrans(dtInfo)  ((dtInfo)->UTransTable)
#define dtGetExternalOutputsDataTypeTrans(dtInfo) ((dtInfo)->YTransTable)

#define dtGetNumTransitions(dtTable) \
    ((dtTable)->numTransitions)

#define dtGetTransitions(dtTable) \
    ((dtTable)->transitions)

#define dtTransGetAddress(dtTable, idx) \
    ((dtTable)->transitions[(idx)].baseAddr)

#define dtTransGetDataType(dtTable, idx) \
    ((dtTable)->transitions[(idx)].dataType)

#define dtTransNEls(dtTable, idx) \
    ((dtTable)->transitions[(idx)].nEls)

#define dtTransGetComplexFlag(dtTable, idx) \
    ((dtTable)->transitions[(idx)].isComplex)

#endif /* _DTINFO_ */
