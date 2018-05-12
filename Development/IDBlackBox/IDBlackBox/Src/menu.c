/********************************* Includes ************************************/
#include "menu.h"

/*********************************** Types ************************************/
/* This menu struct design is from user "brewbuck" on cboard.cprogramming.com
 *
 * Each menu has a title (i.e. message associated with it, to be printed from
 * its parent.
 *
 * Each menu has some submenus. If there are none (numSubmenus == 0 is true) then
 * we know this is a leaf of the menu tree; in this case, we invoke the function
 * cmd. Otherwise, we simply navigate into the submenus by rendering them on the
 * LCD similarly (i.e. printing the titles of their children) and changing the
 * current menu pointer.
 */

#define MAX_TITLE_SIZE 16
#define MAX_NUM_SUBMENUS 4
struct menu_t{
	char* title; // Message to print to LCD
	void (*cmd) (); // Function to invoke at this state
	uint8_t numSubmenus;
	struct menu_t* submenu[MAX_NUM_SUBMENUS];
};

/******************************** Variables ************************************/
// Top-level menu
const char menuTopLevel[4][15] = {
		{"Set ID         "},
		{"Reset          "},
		{"Brdcst Revive  "},
		{"Set LED        "}
};

// Set ID menu
//TODO

// Set LED menu
//TODO

/******************************** Functions ************************************/
//TODO: Check that the appropriate conditions are checked for (might depend on
// keypad design)
int isValid(int keypress){
	// "If 3, 7, 11, 15..." (rightmost column of keys)
	if ((keypress & 0b01) != 1){
		return -1;
	}
	return 1;
}

void reset(void){
	/* The function invoked upon entering the reset menu.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	lcdClear();
	lcdHome();
	printf("Resetting...   ");

	Dynamixel_Reset(&motor);

	HAL_Delay(2000); // Wait about 2 seconds, usually enough time to reset

	lcdClear();
	lcdHome();
	printf("Reset success  ");

	lcdNewline();
	printf("Press any key  ");
	while(keypadRoutine() != -1){	continue;	} // Do nothing while no valid input
}

void revive(void){
	/* The function invoked upon entering the revive menu.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	lcdClear();
	lcdHome();
	printf("Reviving...    ");

	lcdNewline();
	printf("Resetting      ");

	Dynamixel_BroadcastRevive(&motor, DEFAULT_ID);

	lcdClear();
	lcdHome();
	printf("Reviving...    ");
	lcdNewline();
	printf("Set pos to 150 ");

	HAL_Delay(1500); // Time for actuator to move

	printf("Done           ");
	printf("Press any key  ");
	while(keypadRoutine() != -1){	continue;	} // Do nothing while no valid input
}

void menu(void){
	/* Runs the menu and handles all the function calls. Never returns.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	struct menu_t menuSetID;
	menuSetID.cmd = NULL;
	menuSetID.title = "Set ID         ";
	menuSetID.numSubmenus = 0;

	struct menu_t menuReset;
	menuReset.cmd = NULL;
	menuReset.title = "Reset          ";
	menuReset.numSubmenus = 0;

	struct menu_t menuRevive;
	menuRevive.cmd = NULL;
	menuRevive.title = "Brdcst Revive  ";
	menuRevive.numSubmenus = 0;

	struct menu_t menuSetLED;
	menuSetLED.cmd = NULL;
	menuSetLED.title = "Set LED        ";
	menuSetLED.numSubmenus = 0;

	struct menu_t menuTop;
	menuTop.cmd = NULL;
	menuTop.title = NULL;
	menuTop.numSubmenus = 4;
	menuTop.submenu[0] = &menuSetID;
	menuTop.submenu[1] = &menuReset;
	menuTop.submenu[2] = &menuRevive;
	menuTop.submenu[3] = &menuSetLED;
	while(1){

	}
}
