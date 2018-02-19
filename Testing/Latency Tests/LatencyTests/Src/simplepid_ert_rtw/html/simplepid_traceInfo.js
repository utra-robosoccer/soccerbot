function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/Position */
	this.urlHashMap["simplepid:1"] = "ert_main.c:45&simplepid.c:39,56";
	/* <Root>/Repeating
Sequence
Stair */
	this.urlHashMap["simplepid:10"] = "msg=&block=simplepid:10";
	/* <Root>/Torque */
	this.urlHashMap["simplepid:4"] = "ert_main.c:48&simplepid.c:53";
	/* <S1>/Derivative Gain */
	this.urlHashMap["simplepid:3:4163"] = "simplepid.c:48";
	/* <S1>/Filter */
	this.urlHashMap["simplepid:3:4165"] = "simplepid.c:47,88&simplepid.h:38";
	/* <S1>/Filter Coefficient */
	this.urlHashMap["simplepid:3:4166"] = "simplepid.c:46";
	/* <S1>/Integral Gain */
	this.urlHashMap["simplepid:3:4162"] = "msg=rtwMsg_reducedBlock&block=simplepid:3:4162";
	/* <S1>/Integrator */
	this.urlHashMap["simplepid:3:4164"] = "simplepid.c:55,85&simplepid.h:37";
	/* <S1>/Proportional Gain */
	this.urlHashMap["simplepid:3:4161"] = "msg=rtwMsg_reducedBlock&block=simplepid:3:4161";
	/* <S1>/Setpoint Weighting
(Derivative) */
	this.urlHashMap["simplepid:3:4169"] = "msg=rtwMsg_reducedBlock&block=simplepid:3:4169";
	/* <S1>/Setpoint Weighting
(Proportional) */
	this.urlHashMap["simplepid:3:4168"] = "msg=rtwMsg_reducedBlock&block=simplepid:3:4168";
	/* <S1>/Sum */
	this.urlHashMap["simplepid:3:4160"] = "simplepid.c:58";
	/* <S1>/Sum1 */
	this.urlHashMap["simplepid:3:4170"] = "simplepid.c:59";
	/* <S1>/Sum2 */
	this.urlHashMap["simplepid:3:4171"] = "simplepid.c:37";
	/* <S1>/Sum3 */
	this.urlHashMap["simplepid:3:4172"] = "msg=rtwMsg_CodeGenerationReducedBlock&block=simplepid:3:4172";
	/* <S1>/SumD */
	this.urlHashMap["simplepid:3:4167"] = "simplepid.c:49";
	/* <S2>/Force to be scalar */
	this.urlHashMap["simplepid:10:1"] = "msg=&block=simplepid:10:1";
	/* <S2>/LimitedCounter */
	this.urlHashMap["simplepid:10:2"] = "msg=&block=simplepid:10:2";
	/* <S2>/Out */
	this.urlHashMap["simplepid:10:3"] = "msg=&block=simplepid:10:3";
	/* <S2>/Output */
	this.urlHashMap["simplepid:10:4"] = "simplepid.c:40,57";
	/* <S2>/Vector */
	this.urlHashMap["simplepid:10:5"] = "simplepid.c:38,54&simplepid.h:45&simplepid_data.c:30";
	/* <S2>/y */
	this.urlHashMap["simplepid:10:6"] = "msg=&block=simplepid:10:6";
	/* <S3>/Data Type
Propagation */
	this.urlHashMap["simplepid:10:2:1"] = "msg=&block=simplepid:10:2:1";
	/* <S3>/Force to be scalar */
	this.urlHashMap["simplepid:10:2:2"] = "msg=&block=simplepid:10:2:2";
	/* <S3>/Increment
Real World */
	this.urlHashMap["simplepid:10:2:3"] = "msg=&block=simplepid:10:2:3";
	/* <S3>/Output */
	this.urlHashMap["simplepid:10:2:4"] = "simplepid.c:41,60,68,74,79&simplepid.h:39";
	/* <S3>/Wrap To Zero */
	this.urlHashMap["simplepid:10:2:5"] = "msg=&block=simplepid:10:2:5";
	/* <S3>/y */
	this.urlHashMap["simplepid:10:2:6"] = "msg=&block=simplepid:10:2:6";
	/* <S4>/u */
	this.urlHashMap["simplepid:10:2:3:1"] = "msg=&block=simplepid:10:2:3:1";
	/* <S4>/FixPt
Constant */
	this.urlHashMap["simplepid:10:2:3:2"] = "simplepid.c:67";
	/* <S4>/FixPt
Data Type
Duplicate */
	this.urlHashMap["simplepid:10:2:3:3"] = "msg=&block=simplepid:10:2:3:3";
	/* <S4>/FixPt
Sum1 */
	this.urlHashMap["simplepid:10:2:3:4"] = "simplepid.c:66";
	/* <S4>/y */
	this.urlHashMap["simplepid:10:2:3:5"] = "msg=&block=simplepid:10:2:3:5";
	/* <S5>/U */
	this.urlHashMap["simplepid:10:2:5:1"] = "msg=&block=simplepid:10:2:5:1";
	/* <S5>/Constant */
	this.urlHashMap["simplepid:10:2:5:4"] = "simplepid.c:75";
	/* <S5>/FixPt
Data Type
Duplicate1 */
	this.urlHashMap["simplepid:10:2:5:2"] = "msg=&block=simplepid:10:2:5:2";
	/* <S5>/FixPt
Switch */
	this.urlHashMap["simplepid:10:2:5:3"] = "simplepid.c:72,83";
	/* <S5>/Y */
	this.urlHashMap["simplepid:10:2:5:5"] = "msg=&block=simplepid:10:2:5:5";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "simplepid"};
	this.sidHashMap["simplepid"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "simplepid:3"};
	this.sidHashMap["simplepid:3"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<S2>"] = {sid: "simplepid:10"};
	this.sidHashMap["simplepid:10"] = {rtwname: "<S2>"};
	this.rtwnameHashMap["<S3>"] = {sid: "simplepid:10:2"};
	this.sidHashMap["simplepid:10:2"] = {rtwname: "<S3>"};
	this.rtwnameHashMap["<S4>"] = {sid: "simplepid:10:2:3"};
	this.sidHashMap["simplepid:10:2:3"] = {rtwname: "<S4>"};
	this.rtwnameHashMap["<S5>"] = {sid: "simplepid:10:2:5"};
	this.sidHashMap["simplepid:10:2:5"] = {rtwname: "<S5>"};
	this.rtwnameHashMap["<Root>/Position"] = {sid: "simplepid:1"};
	this.sidHashMap["simplepid:1"] = {rtwname: "<Root>/Position"};
	this.rtwnameHashMap["<Root>/PID Controller (2DOF)"] = {sid: "simplepid:3"};
	this.sidHashMap["simplepid:3"] = {rtwname: "<Root>/PID Controller (2DOF)"};
	this.rtwnameHashMap["<Root>/Repeating Sequence Stair"] = {sid: "simplepid:10"};
	this.sidHashMap["simplepid:10"] = {rtwname: "<Root>/Repeating Sequence Stair"};
	this.rtwnameHashMap["<Root>/Torque"] = {sid: "simplepid:4"};
	this.sidHashMap["simplepid:4"] = {rtwname: "<Root>/Torque"};
	this.rtwnameHashMap["<S1>/r"] = {sid: "simplepid:3:1"};
	this.sidHashMap["simplepid:3:1"] = {rtwname: "<S1>/r"};
	this.rtwnameHashMap["<S1>/y"] = {sid: "simplepid:3:2"};
	this.sidHashMap["simplepid:3:2"] = {rtwname: "<S1>/y"};
	this.rtwnameHashMap["<S1>/Derivative Gain"] = {sid: "simplepid:3:4163"};
	this.sidHashMap["simplepid:3:4163"] = {rtwname: "<S1>/Derivative Gain"};
	this.rtwnameHashMap["<S1>/Filter"] = {sid: "simplepid:3:4165"};
	this.sidHashMap["simplepid:3:4165"] = {rtwname: "<S1>/Filter"};
	this.rtwnameHashMap["<S1>/Filter Coefficient"] = {sid: "simplepid:3:4166"};
	this.sidHashMap["simplepid:3:4166"] = {rtwname: "<S1>/Filter Coefficient"};
	this.rtwnameHashMap["<S1>/Integral Gain"] = {sid: "simplepid:3:4162"};
	this.sidHashMap["simplepid:3:4162"] = {rtwname: "<S1>/Integral Gain"};
	this.rtwnameHashMap["<S1>/Integrator"] = {sid: "simplepid:3:4164"};
	this.sidHashMap["simplepid:3:4164"] = {rtwname: "<S1>/Integrator"};
	this.rtwnameHashMap["<S1>/Proportional Gain"] = {sid: "simplepid:3:4161"};
	this.sidHashMap["simplepid:3:4161"] = {rtwname: "<S1>/Proportional Gain"};
	this.rtwnameHashMap["<S1>/Setpoint Weighting (Derivative)"] = {sid: "simplepid:3:4169"};
	this.sidHashMap["simplepid:3:4169"] = {rtwname: "<S1>/Setpoint Weighting (Derivative)"};
	this.rtwnameHashMap["<S1>/Setpoint Weighting (Proportional)"] = {sid: "simplepid:3:4168"};
	this.sidHashMap["simplepid:3:4168"] = {rtwname: "<S1>/Setpoint Weighting (Proportional)"};
	this.rtwnameHashMap["<S1>/Sum"] = {sid: "simplepid:3:4160"};
	this.sidHashMap["simplepid:3:4160"] = {rtwname: "<S1>/Sum"};
	this.rtwnameHashMap["<S1>/Sum1"] = {sid: "simplepid:3:4170"};
	this.sidHashMap["simplepid:3:4170"] = {rtwname: "<S1>/Sum1"};
	this.rtwnameHashMap["<S1>/Sum2"] = {sid: "simplepid:3:4171"};
	this.sidHashMap["simplepid:3:4171"] = {rtwname: "<S1>/Sum2"};
	this.rtwnameHashMap["<S1>/Sum3"] = {sid: "simplepid:3:4172"};
	this.sidHashMap["simplepid:3:4172"] = {rtwname: "<S1>/Sum3"};
	this.rtwnameHashMap["<S1>/SumD"] = {sid: "simplepid:3:4167"};
	this.sidHashMap["simplepid:3:4167"] = {rtwname: "<S1>/SumD"};
	this.rtwnameHashMap["<S1>/u"] = {sid: "simplepid:3:16"};
	this.sidHashMap["simplepid:3:16"] = {rtwname: "<S1>/u"};
	this.rtwnameHashMap["<S2>/Force to be scalar"] = {sid: "simplepid:10:1"};
	this.sidHashMap["simplepid:10:1"] = {rtwname: "<S2>/Force to be scalar"};
	this.rtwnameHashMap["<S2>/LimitedCounter"] = {sid: "simplepid:10:2"};
	this.sidHashMap["simplepid:10:2"] = {rtwname: "<S2>/LimitedCounter"};
	this.rtwnameHashMap["<S2>/Out"] = {sid: "simplepid:10:3"};
	this.sidHashMap["simplepid:10:3"] = {rtwname: "<S2>/Out"};
	this.rtwnameHashMap["<S2>/Output"] = {sid: "simplepid:10:4"};
	this.sidHashMap["simplepid:10:4"] = {rtwname: "<S2>/Output"};
	this.rtwnameHashMap["<S2>/Vector"] = {sid: "simplepid:10:5"};
	this.sidHashMap["simplepid:10:5"] = {rtwname: "<S2>/Vector"};
	this.rtwnameHashMap["<S2>/y"] = {sid: "simplepid:10:6"};
	this.sidHashMap["simplepid:10:6"] = {rtwname: "<S2>/y"};
	this.rtwnameHashMap["<S3>/Data Type Propagation"] = {sid: "simplepid:10:2:1"};
	this.sidHashMap["simplepid:10:2:1"] = {rtwname: "<S3>/Data Type Propagation"};
	this.rtwnameHashMap["<S3>/Force to be scalar"] = {sid: "simplepid:10:2:2"};
	this.sidHashMap["simplepid:10:2:2"] = {rtwname: "<S3>/Force to be scalar"};
	this.rtwnameHashMap["<S3>/Increment Real World"] = {sid: "simplepid:10:2:3"};
	this.sidHashMap["simplepid:10:2:3"] = {rtwname: "<S3>/Increment Real World"};
	this.rtwnameHashMap["<S3>/Output"] = {sid: "simplepid:10:2:4"};
	this.sidHashMap["simplepid:10:2:4"] = {rtwname: "<S3>/Output"};
	this.rtwnameHashMap["<S3>/Wrap To Zero"] = {sid: "simplepid:10:2:5"};
	this.sidHashMap["simplepid:10:2:5"] = {rtwname: "<S3>/Wrap To Zero"};
	this.rtwnameHashMap["<S3>/y"] = {sid: "simplepid:10:2:6"};
	this.sidHashMap["simplepid:10:2:6"] = {rtwname: "<S3>/y"};
	this.rtwnameHashMap["<S4>/u"] = {sid: "simplepid:10:2:3:1"};
	this.sidHashMap["simplepid:10:2:3:1"] = {rtwname: "<S4>/u"};
	this.rtwnameHashMap["<S4>/FixPt Constant"] = {sid: "simplepid:10:2:3:2"};
	this.sidHashMap["simplepid:10:2:3:2"] = {rtwname: "<S4>/FixPt Constant"};
	this.rtwnameHashMap["<S4>/FixPt Data Type Duplicate"] = {sid: "simplepid:10:2:3:3"};
	this.sidHashMap["simplepid:10:2:3:3"] = {rtwname: "<S4>/FixPt Data Type Duplicate"};
	this.rtwnameHashMap["<S4>/FixPt Sum1"] = {sid: "simplepid:10:2:3:4"};
	this.sidHashMap["simplepid:10:2:3:4"] = {rtwname: "<S4>/FixPt Sum1"};
	this.rtwnameHashMap["<S4>/y"] = {sid: "simplepid:10:2:3:5"};
	this.sidHashMap["simplepid:10:2:3:5"] = {rtwname: "<S4>/y"};
	this.rtwnameHashMap["<S5>/U"] = {sid: "simplepid:10:2:5:1"};
	this.sidHashMap["simplepid:10:2:5:1"] = {rtwname: "<S5>/U"};
	this.rtwnameHashMap["<S5>/Constant"] = {sid: "simplepid:10:2:5:4"};
	this.sidHashMap["simplepid:10:2:5:4"] = {rtwname: "<S5>/Constant"};
	this.rtwnameHashMap["<S5>/FixPt Data Type Duplicate1"] = {sid: "simplepid:10:2:5:2"};
	this.sidHashMap["simplepid:10:2:5:2"] = {rtwname: "<S5>/FixPt Data Type Duplicate1"};
	this.rtwnameHashMap["<S5>/FixPt Switch"] = {sid: "simplepid:10:2:5:3"};
	this.sidHashMap["simplepid:10:2:5:3"] = {rtwname: "<S5>/FixPt Switch"};
	this.rtwnameHashMap["<S5>/Y"] = {sid: "simplepid:10:2:5:5"};
	this.sidHashMap["simplepid:10:2:5:5"] = {rtwname: "<S5>/Y"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
