#pragma config(StandardModel, "EV3_REMBOT")

typedef struct
{
	bool northWall;
	bool eastWall;
	bool isValidPath; // Returns true if path is valid and false otherwise
} cell;

// Global Constants
const int GRID_ROWS = 6, GRID_COLS = 8;
const int SCREEN_WIDTH = 177, SCREEN_HEIGHT = 127;

// Starting and ending positions
const int START_ROW = 4, START_COL = 1;
const int TARGET_ROW = 1, TARGET_COL = 6;

// Global 2D Array
cell grid[GRID_ROWS][GRID_COLS];

// Global Variables
int currRow = START_ROW, currCol = START_COL;
char heading = 'N';
char travelLog[23] = 0;

// Initilizes maze and set all paths as valid
void initMazeGrid()
{
	for (int row = 1; row < GRID_ROWS-1; row++) {
		for (int col = 1; col < GRID_COLS-1; col++) {
			grid[row][col].isValidPath = true;
		}
	}
}

/*
The following function scans infront of the robot and
determines if there is a wall present
*/
bool scanFront()
{
	if (getUSDistance(sonarSensor) < 20) {
		switch (heading) {
		case 'N': grid[currRow][currCol].northWall = true; break;
		case 'E': grid[currRow][currCol].eastWall = true; break;
		case 'S': grid[currRow+1][currCol].northWall = true; break;
		case 'W': grid[currRow][currCol-1].eastWall = true; break;
		}
		return true;
	}
	return false;
}

/*
The following function sets the current box as invalid and
drives to robot forward for 730 encoder values.
*/
void forward()
{
	// Set current box robot is in as invalid
	grid[currRow][currCol].isValidPath = false;

	// Drive robot forward 730 encoder movements
	moveMotorTarget(leftMotor, 730, 50);
	moveMotorTarget(rightMotor, 730, 50);
	waitUntilMotorStop(leftMotor);
	delay(200);

	// Determines which cell the robot is in
	switch (heading) {
	case 'N': currRow--; break;
	case 'E': currCol++; break;
	case 'S': currRow++; break;
	case 'W': currCol--; break;
	}
}

/*
Description: The function is responsible for rotating 90 degrees
depending on the amount of time specified in the parameter.
*/
void rotate90Degrees(int multiplier)
{
	// Determines the encoder values to turn
	int steps = abs(multiplier);

	// 90* 2.494 is a constant that happens to represent 90 degrees
	for (int i = 0; i < steps; i++) {
		moveMotorTarget(leftMotor, multiplier/steps*90*2.494, 25);
		moveMotorTarget(rightMotor, multiplier/steps*-90*2.494, 25);
		waitUntilMotorStop(leftMotor);
		delay(200);
	}
}

// The  following function add a char to the beginning of a string
void stringPrepend(string &str, char c)
{
	reverseChars(str);
	stringConcatenate(str, c);
	reverseChars(str);
}

// insert challenge heading at correct index of headings so that direction with the highest chance
// (closest with isValidPath = true) of leading to target is at the back
void getValidHeading(string &preTargets, float &minCellDist, string &headings, char challenge)
{
	float cellAvgDist = 0;
	int cellRow, cellCol;

	// get coords of cell to test (next cell the robot would travel to)
	switch (challenge) {
	case 'N': cellRow = currRow-1; cellCol = currCol; break;
	case 'E': cellRow = currRow; cellCol = currCol+1; break;
	case 'S': cellRow = currRow+1; cellCol = currCol; break;
	case 'W': cellRow = currRow; cellCol = currCol-1; break;
	}

	// iterate through cells around target cell that could lead to the target cell,
	// add distance from test cell to cell around the target to get average distance (from test to target)
	for (int i = 0; i < strlen(preTargets); i++) {
		int toRow, toCol;
		switch (stringGetChar(preTargets, i)) {
		case 'N': toRow = TARGET_ROW-1; toCol = TARGET_COL; break;
		case 'E': toRow = TARGET_ROW; toCol = TARGET_COL+1; break;
		case 'S': toRow = TARGET_ROW+1; toCol = TARGET_COL; break;
		case 'W': toRow = TARGET_ROW; toCol = TARGET_COL-1; break;
		}
		if (currRow == toRow && currCol == toCol) {
			toRow = TARGET_ROW;
			toCol = TARGET_COL;
		}
		cellAvgDist += abs(cellRow-toRow) + abs(cellCol-toCol);
	}
	cellAvgDist /= strlen(preTargets);

	// increase distance of already travelled cells to lower priority
	cellAvgDist += !grid[cellRow][cellCol].isValidPath;

	// modify valid headings so priority goes to back of string
	// (directions that lead farther away from target pushed to front of string)
	if (cellAvgDist < minCellDist) {
		minCellDist = cellAvgDist;
		stringConcatenate(headings, challenge);
		} else {
		stringPrepend(headings, challenge);
	}
}

// Determines if path is valid
bool validatePathDecision(char newHeading)
{
	bool isValid = true;
	switch (newHeading) {
	case 'N':
		isValid = grid[currRow-1][currCol].isValidPath && !grid[currRow][currCol].northWall;
		break;
	case 'E':
		isValid = grid[currRow][currCol+1].isValidPath && !grid[currRow][currCol].eastWall;
		break;
	case 'S':
		isValid = grid[currRow+1][currCol].isValidPath && !grid[currRow+1][currCol].northWall;
		break;
	case 'W':
		isValid = grid[currRow][currCol-1].isValidPath && !grid[currRow][currCol-1].eastWall;
		break;
	}
	return isValid;
}

char getNextHeading()
{
	string preTargets = "";
	string headings = "";
	string validHeadings = "";
	float minCellDist = 9;

	// target cell walls that are unknown
	stringConcatenate(preTargets, grid[TARGET_ROW][TARGET_COL].northWall ? 0 : 'N');
	stringConcatenate(preTargets, grid[TARGET_ROW][TARGET_COL].eastWall ? 0 : 'E');
	stringConcatenate(preTargets, grid[TARGET_ROW+1][TARGET_COL].northWall ? 0 : 'S');
	stringConcatenate(preTargets, grid[TARGET_ROW][TARGET_COL-1].eastWall ? 0 : 'W');

	// add valid directions and sort so priority goes to the back
	if (strlen(preTargets) != 0) {
		if (!grid[currRow][currCol].northWall) {
			getValidHeading(preTargets, minCellDist, headings, 'N');
		}
		if (!grid[currRow][currCol].eastWall) {
			getValidHeading(preTargets, minCellDist, headings, 'E');
		}
		if (!grid[currRow+1][currCol].northWall) {
			getValidHeading(preTargets, minCellDist, headings, 'S');
		}
		if (!grid[currRow][currCol-1].eastWall) {
			getValidHeading(preTargets, minCellDist, headings, 'W');
		}
	}

	// move headings with isValidPath == true to back (higher priority)
	for (int i = 0; i < strlen(headings); i++) {
		if (validatePathDecision(stringGetChar(headings, i))) {
			stringConcatenate(validHeadings, stringGetChar(headings, i));
		}
	}

	// if no paths, back track
	if (strlen(validHeadings) == 0 && strlen(travelLog) > 0) {
		switch (travelLog[strlen(travelLog)-1]) {
		case 'N': stringConcatenate(validHeadings, 'S'); break;
		case 'E': stringConcatenate(validHeadings, 'W'); break;
		case 'S': stringConcatenate(validHeadings, 'N'); break;
		case 'W': stringConcatenate(validHeadings, 'E'); break;
		}
	}

	// return heading with highest priority (closest to target, preferably with isValidPath = true)
	return stringGetChar(validHeadings, strlen(validHeadings)-1);
}

void setHeading(char newHeading)
{
	static const string COMPASS = "NESW";

	string strNewHeading = newHeading;
	string strHeading = heading;
	int indexDiff = stringFind(COMPASS, strNewHeading) - stringFind(COMPASS, strHeading);
	switch (indexDiff) {
		case 3: indexDiff = -1; break;
		case -3: indexDiff = 1; break;
	}
	rotate90Degrees(indexDiff);
	heading = newHeading;
}

// The following function determines the shortest distance
// possible inorder to reach back to the starting position
void updateTravelLog()
{
	bool backtracking = false;
	if (strlen(travelLog) != 0) {
		char prevHeading = travelLog[strlen(travelLog)-1];
		// sets the appropriate heading to return to start
		switch (heading) {
		case 'N': backtracking = prevHeading == 'S'; break;
		case 'E': backtracking = prevHeading == 'W'; break;
		case 'S': backtracking = prevHeading == 'N'; break;
		case 'W': backtracking = prevHeading == 'E'; break;
		}
	}
	if (backtracking) {
		travelLog[strlen(travelLog)-1] = 0;
		} else {
		travelLog[strlen(travelLog)] = heading;
	}
}

// The following function determines if the maze is solvable
bool isSolvable()
{
	bool targetNorthWall = grid[TARGET_ROW][TARGET_COL].northWall;
	bool targetEastWall = grid[TARGET_ROW][TARGET_COL].eastWall;
	bool targetSouthWall = grid[TARGET_ROW+1][TARGET_COL].northWall;
	bool targetWestWall = grid[TARGET_ROW][TARGET_COL-1].eastWall;
	bool startNorthWall = grid[START_ROW][START_COL].northWall;
	bool startEastWall = grid[START_ROW][START_COL].eastWall;
	bool startSouthWall = grid[START_ROW+1][START_COL].northWall;
	bool startWestWall = grid[START_ROW][START_COL-1].eastWall;

	return !(targetNorthWall && targetEastWall && targetSouthWall && targetWestWall)
	&& !(startNorthWall && startEastWall && startSouthWall && startWestWall);
}

// The following function draws the walls the robot sees on the LCD screen
void drawCell(int row, int col)
{
	int top = (SCREEN_HEIGHT/4.0) * (GRID_ROWS-row-1);
	int right = (SCREEN_WIDTH/6.0) * (col);
	int bottom = (SCREEN_HEIGHT/4.0) * (GRID_ROWS-row-2);
	int left = (SCREEN_WIDTH/6.0) * (col-1);

	int midPtX = (right+left) / 2 - 1;
	int midPtY = (top+bottom) / 2 + 1;

	// Draws the position the robot is facing on the LCD screen
	if (row == currRow && col == currCol) {
		switch (heading) {
		case 'N': displayStringAt(midPtX, midPtY, "^"); break;
		case 'E': displayStringAt(midPtX, midPtY, ">"); break;
		case 'S': displayStringAt(midPtX, midPtY, "v"); break;
		case 'W': displayStringAt(midPtX, midPtY, "<"); break;
		}
	}
	// Displays the starting position on the LCD screen
	else if (row == START_ROW && col == START_COL)
		displayStringAt(midPtX, midPtY, "S");

	// Displays the ending positon on the LCD screen
	else if (row == TARGET_ROW && col == TARGET_COL)
		displayStringAt(midPtX, midPtY, "E");

	// Draws line (xPosition, yPosition, xPositionTo, yPositionTo)
	if (grid[row][col].northWall)
		drawLine(left, top, right, top);

	if (grid[row][col].eastWall)
		drawLine(right, bottom, right, top);

}

// The following function updates the maze on the LCD screen
void updateMazeDisplay()
{
	eraseDisplay();

	drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	for (int row = 1; row < GRID_ROWS-1; row++) {
		for (int col = 1; col < GRID_COLS-1; col++) {
			drawCell(row, col);
		}
	}
}

// The following function displays a message to the LCD screen
void displayNotification(const string &msg)
{
	for (int i = 0; i < 3; i++) {
		eraseDisplay();
		delay(100);
		displayCenteredTextLine(3, msg);
		delay(400);
	}
	delay(3000);
}

task main()
{
	initMazeGrid();

	moveMotorTarget(leftMotor, 25, 25);
	moveMotorTarget(rightMotor, 25, 25);
	moveMotorTarget(armMotor, 50, 50);
	delay(1000);

	while ((currRow != TARGET_ROW || currCol != TARGET_COL)) {
		scanFront();
		do {
			updateMazeDisplay();
			setHeading(getNextHeading());
			updateMazeDisplay();
		} while (scanFront() && isSolvable());
		if (isSolvable()) {
			forward();
			updateMazeDisplay();
			updateTravelLog();
			} else {
			break;
		}
	}
	if (isSolvable()) {
		displayNotification("TARGET CELL REACHED");
		} else {
		displayNotification("(/=_ =)/ ^ _|__|_");
	}
	for (int i = strlen(travelLog)-1; i >= 0; i--) {
		switch (travelLog[i]) {
		case 'N': setHeading('S'); break;
		case 'E': setHeading('W'); break;
		case 'S': setHeading('N'); break;
		case 'W': setHeading('E'); break;
		}
		updateMazeDisplay();
		forward();
		scanFront();
		updateMazeDisplay();
	}
	displayNotification("HOME CELL REACHED");
}
