function CodeDefine() { 
this.def = new Array();
this.def["rt_OneStep"] = {file: "ert_main_c.html",line:41,type:"fcn"};
this.def["main"] = {file: "ert_main_c.html",line:83,type:"fcn"};
this.def["simplepid_DW"] = {file: "simplepid_c.html",line:28,type:"var"};
this.def["simplepid_step"] = {file: "simplepid_c.html",line:31,type:"fcn"};
this.def["simplepid_initialize"] = {file: "simplepid_c.html",line:93,type:"fcn"};
this.def["simplepid_terminate"] = {file: "simplepid_c.html",line:99,type:"fcn"};
this.def["DW_simplepid_T"] = {file: "simplepid_h.html",line:40,type:"type"};
this.def["ConstP_simplepid_T"] = {file: "simplepid_h.html",line:48,type:"type"};
this.def["simplepid_ConstP"] = {file: "simplepid_data_c.html",line:28,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:59,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:60,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:61,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:73,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:74,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:75,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:76,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:94,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "/";
var isPC = false;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_c.html"] = "../ert_main.c";
	this.html2Root["ert_main_c.html"] = "ert_main_c.html";
	this.html2SrcPath["simplepid_c.html"] = "../simplepid.c";
	this.html2Root["simplepid_c.html"] = "simplepid_c.html";
	this.html2SrcPath["simplepid_h.html"] = "../simplepid.h";
	this.html2Root["simplepid_h.html"] = "simplepid_h.html";
	this.html2SrcPath["simplepid_private_h.html"] = "../simplepid_private.h";
	this.html2Root["simplepid_private_h.html"] = "simplepid_private_h.html";
	this.html2SrcPath["simplepid_types_h.html"] = "../simplepid_types.h";
	this.html2Root["simplepid_types_h.html"] = "simplepid_types_h.html";
	this.html2SrcPath["simplepid_data_c.html"] = "../simplepid_data.c";
	this.html2Root["simplepid_data_c.html"] = "simplepid_data_c.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_c.html","simplepid_c.html","simplepid_h.html","simplepid_private_h.html","simplepid_types_h.html","simplepid_data_c.html","rtwtypes_h.html"];
