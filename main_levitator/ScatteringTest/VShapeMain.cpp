#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverBEM.h>
#include <AsierInho_V2.h>
#include <stdio.h>
#include <conio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Helper/ReflectionSurface.h>
#include <Helper/VisualizePlane.h>
#include "CImg\CImg.h"
#include <Windows.h>

void print(const char* str) {
	printf("%s\n", str);
}

void visualize(GSPAT::Solver* solver, float A[3], float B[3], float C[3], int imageSize[2], float* hologram) {
	cimg_library::CImg<unsigned char> img = cimg_library::CImg<unsigned char>(imageSize[0], imageSize[1], 1, 3);
	//1.Initialize iterator vectors.
	float step_AB[3], step_AC[3];
	step_AB[0] = (B[0] - A[0]) / imageSize[0]; //Step vector in direction AB
	step_AB[1] = (B[1] - A[1]) / imageSize[0];
	step_AB[2] = (B[2] - A[2]) / imageSize[0];
	step_AC[0] = (C[0] - A[0]) / imageSize[1]; //Step vector in direction AC
	step_AC[1] = (C[1] - A[1]) / imageSize[1];
	step_AC[2] = (C[2] - A[2]) / imageSize[1];
	//2. Step through the pixels:
	int numFieldPoints = imageSize[0] * imageSize[1];
	float* fieldPositions = new float[4 * numFieldPoints];
	int positionIndex = 0;
	for (int px = 0; px < imageSize[0]; px++) {
		for (int py = 0; py < imageSize[1]; py++) {
			//2.1. Compute 3D position of pixel (px,py)
			fieldPositions[positionIndex++] = A[0] + px * step_AB[0] + py * step_AC[0];
			fieldPositions[positionIndex++] = A[1] + px * step_AB[1] + py * step_AC[1];
			fieldPositions[positionIndex++] = A[2] + px * step_AB[2] + py * step_AC[2];
			fieldPositions[positionIndex++] = 1;
		}
	}
	float* field = GSPAT_BEM::simulateFieldFromHologram(solver, hologram, numFieldPoints, fieldPositions);

	float* amplitudes = new float[imageSize[0] * imageSize[1]];
	float avg_Amp = 0, min_Amp = 1000000, max_Amp = 0;;
	for (int px = 0; px < imageSize[0]; px++) {
		for (int py = 0; py < imageSize[1]; py++) {
			float Re = field[2 * (px * imageSize[1] + py) + 0];
			float Im = field[2 * (px * imageSize[1] + py) + 1];
			//2.3. Map field to pixel colour: 
			float amp_R = sqrt(Re * Re + Im * Im);
			amplitudes[py * imageSize[0] + px] = amp_R;
			//Some stats:
			avg_Amp += amp_R;
			min_Amp = (amp_R < min_Amp ? amp_R : min_Amp);
			max_Amp = (amp_R > max_Amp ? amp_R : max_Amp);
		}
	}
	float ampToColour = 3 * 256 / max_Amp;
	for (int px = 0; px < imageSize[0]; px++) {
		for (int py = 0; py < imageSize[1]; py++) {

			unsigned int amp_R = (unsigned int)(ampToColour * amplitudes[py * imageSize[0] + px]);
			unsigned int R_aux = 0, G_aux = 0, B_aux = 0;
			if (amp_R < 256) R_aux = amp_R;
			else if (amp_R < 512) { R_aux = 255; G_aux = (amp_R - 256); }
			else { R_aux = 255; G_aux = 255; B_aux = amp_R - 512; }
			unsigned char R = (unsigned char)(R_aux);
			unsigned char G = (unsigned char)(G_aux);
			unsigned char B = (unsigned char)(B_aux);
			img(px, py, 0) = R; img(px, py, 1) = G; img(px, py, 2) = B;
		}
	}

	static cimg_library::CImgDisplay disp(img, "Playing with images...", false);
	disp.assign(img);
	delete[] field;
	delete[] amplitudes;
	printf("\nAVG Amp= %f;\n MIN Amp=%f;\n MAX amp=%f;\n", avg_Amp / (imageSize[0] * imageSize[1]), min_Amp, max_Amp);
}

enum Solvers { BEM_SOLVER = 0, GSPAT_SOLVER, NAIVE_SOLVER };
enum Setups { DOUBLE_SIDED = 0, V_SHAPED};
Solvers solverUsed = GSPAT_SOLVER;
Setups setupUsed = V_SHAPED;
void play() {
	const size_t numPoints = 2; //Change this if you want (but make sure to change also each point's position )
	const size_t numGeometries = 1;//Please, do not change this. Multiple update Messages require AsierInhoV2 (see 6.simpleGSPATCpp_AsierInhoV2)

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	if (solverUsed == BEM_SOLVER)  GSPAT_BEM::RegisterPrintFuncs(print, print, print);
	if (solverUsed == GSPAT_SOLVER)  GSPAT_V2::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	int boardIDs[] = { 8, 15 };
	glm::mat4 board1, board2;
	if (setupUsed == DOUBLE_SIDED) {
		// bottom board
		board1 = glm::mat4(1.f);// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, -0.12f));
		// top board
		board2 = glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.24f)) * glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.12f))* glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));
	}
	if (setupUsed == V_SHAPED) {
		glm::mat4 Rot = glm::rotate(glm::mat4(1.f), glm::radians(-90.f), glm::vec3(0, 0, 1));
		// left board
		glm::mat4 RotL = glm::rotate(glm::mat4(1.f), glm::radians(+45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransL = glm::translate(glm::mat4(1.f), glm::vec3(0, +0.06364f, +0.072125f));
		board1 = TransL * RotL * Rot;
		// right board
		glm::mat4 RotR = glm::rotate(glm::mat4(1.f), glm::radians(-45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransR = glm::translate(glm::mat4(1.f), glm::vec3(0, -0.06364f, +0.072125f));
		board2 = TransR * RotR * Rot;
	}
	float matBoardToWorld[32] = {
		board1[0][0], board1[1][0], board1[2][0], board1[3][0],
		board1[0][1], board1[1][1], board1[2][1], board1[3][1],
		board1[0][2], board1[1][2], board1[2][2], board1[3][2],
		board1[0][3], board1[1][3], board1[2][3], board1[3][3],
		board2[0][0], board2[1][0], board2[2][0], board2[3][0],
		board2[0][1], board2[1][1], board2[2][1], board2[3][1],
		board2[0][2], board2[1][2], board2[2][2], board2[3][2],
		board2[0][3], board2[1][3], board2[2][3], board2[3][3],
	};

	GSPAT::Solver* solver;
	if (solverUsed == BEM_SOLVER) solver = GSPAT_BEM::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	if (solverUsed == GSPAT_SOLVER) solver = GSPAT_V2::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	//if (!driver->connect(21, 15))	//Device IDs to connect to
	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	//Program: Create a trap and move it with the keyboard
	static float radius = 0.015f;
	float curPos[4 * numGeometries * numPoints];
	float amplitude[numGeometries * numPoints];
	for (int g = 0; g < numGeometries; g++) {
		for (int p = 0; p < numPoints; p++) {
			float angle = 2 * M_PI / numPoints;
			//curPos[4 * g*numPoints + 4*p + 0] = radius * cos(p*angle);
			//curPos[4 * g*numPoints + 4*p + 1] = radius * sin(p*angle);
			float r1 = 0.04f / 2.f;
			float r2 = 0.03f / 2.f;
			if (p == 0) {
				curPos[4 * g * numPoints + 4 * p + 0] = 0;
				curPos[4 * g * numPoints + 4 * p + 1] = +0.02f;
				curPos[4 * g * numPoints + 4 * p + 2] = +0.13f;
			}
			else if (p == 1) {
				curPos[4 * g * numPoints + 4 * p + 0] = 0;
				curPos[4 * g * numPoints + 4 * p + 1] = -0.02f;
				curPos[4 * g * numPoints + 4 * p + 2] = +0.13f;
			}
			else if (p == 2) {
				curPos[4 * g * numPoints + 4 * p + 0] = -0.02f;
				curPos[4 * g * numPoints + 4 * p + 1] = 0;
				curPos[4 * g * numPoints + 4 * p + 2] = -0.02f;
			}

			curPos[4 * g * numPoints + 4 * p + 3] = 1;
			amplitude[g * numPoints + p] = 10000;
		}
	}
	unsigned char* msg;
	float matI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float m1[numPoints * 16];
	for (int i = 0; i < numPoints; i++)
		memcpy(&(m1[i * 16]), matI, 16 * sizeof(float));
	//a. create a solution
	GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
	if(solverUsed == BEM_SOLVER) GSPAT_BEM::setAlgorithm(solution, 4, 200);
	solver->compute(solution);
	solution->finalMessages(&msg);
	for (int s = 0; s < 16; s++)//Fill FPGA buffers so update is processed directly
		driver->updateMessage(msg);

	{
		float A[] = { 0, -0.06f, +0.24f }, B[] = { 0, 0.06f, +0.24f }, C[] = { 0, -0.06f, 0 };
		int imgRes[] = { 256,512 };
		if (solverUsed == BEM_SOLVER) visualize(solver, A, B, C, imgRes, solution->finalHologramReIm());
		else VisualizePlane::visualizeFromPhases(A, B, C, imgRes, solution->finalArrayPhases(), 512, transducerPositions);
	}
	solver->releaseSolution(solution);

	//b. Main loop (finished when space bar is pressed):
	printf("\n Place a bead at (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);
	printf("Use keys A-D , W-S and Q-E to move the bead\n");
	printf("Press 'X' to destroy the solver.\n");

	bool finished = false;
	while (!finished) {
		//Update 3D position from keyboard presses
		switch (getch()) {
		case 'a':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 0] += 0.0005f; break;
		case 'd':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 0] -= 0.0005f; break;
		case 'w':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 1] += 0.0005f; break;
		case 's':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 1] -= 0.0005f; break;
		case 'q':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 2] += 0.00025f; break;
		case 'e':
			for (int g = 0; g < numGeometries; g++)
				for (int p = 0; p < numPoints; p++)
					curPos[4 * g * numPoints + 4 * p + 2] -= 0.00025f; break;
		case 'x':
		case 'X':
			finished = true;

		}
		//Create the trap and send to the board:
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
		if (solverUsed == BEM_SOLVER) GSPAT_BEM::setAlgorithm(solution, 4, 200);
		solver->compute(solution);
		solution->finalMessages(&msg);
		for (int s = 0; s < 16; s++)//Fill FPGA buffers so update is processed directly
			driver->updateMessage(msg);
		solver->releaseSolution(solution);
		printf("\n Bead location (%f,%f,%f)\n ", curPos[0], curPos[1], curPos[2]);

	}
	//Deallocate the AsierInho controller: 
	driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	delete solver;
}
float* getHologram(Solvers solverUsed, Setups setupUsed) {
	const size_t numPoints = 2; //Change this if you want (but make sure to change also each point's position )
	const size_t numGeometries = 1;//Please, do not change this. Multiple update Messages require AsierInhoV2 (see 6.simpleGSPATCpp_AsierInhoV2)

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	if (solverUsed == BEM_SOLVER)  GSPAT_BEM::RegisterPrintFuncs(print, print, print);
	if (solverUsed == GSPAT_SOLVER)  GSPAT_V2::RegisterPrintFuncs(print, print, print);
	if (solverUsed == NAIVE_SOLVER)  GSPAT_Naive::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	int boardIDs[] = { 8, 15 };
	glm::mat4 board1, board2;
	if (setupUsed == DOUBLE_SIDED) {
		// bottom board
		board1 = glm::mat4(1.f);// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, -0.12f));
		// top board
		board2 = glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.24f)) * glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.12f))* glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));
	}
	if (setupUsed == V_SHAPED) {
		glm::mat4 Rot = glm::rotate(glm::mat4(1.f), glm::radians(-90.f), glm::vec3(0, 0, 1));
		// left board
		glm::mat4 RotL = glm::rotate(glm::mat4(1.f), glm::radians(+45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransL = glm::translate(glm::mat4(1.f), glm::vec3(0, +0.06364f, +0.072125f));
		board1 = TransL * RotL * Rot;
		// right board
		glm::mat4 RotR = glm::rotate(glm::mat4(1.f), glm::radians(-45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransR = glm::translate(glm::mat4(1.f), glm::vec3(0, -0.06364f, +0.072125f));
		board2 = TransR * RotR * Rot;
	}
	float matBoardToWorld[32] = {
		board1[0][0], board1[1][0], board1[2][0], board1[3][0],
		board1[0][1], board1[1][1], board1[2][1], board1[3][1],
		board1[0][2], board1[1][2], board1[2][2], board1[3][2],
		board1[0][3], board1[1][3], board1[2][3], board1[3][3],
		board2[0][0], board2[1][0], board2[2][0], board2[3][0],
		board2[0][1], board2[1][1], board2[2][1], board2[3][1],
		board2[0][2], board2[1][2], board2[2][2], board2[3][2],
		board2[0][3], board2[1][3], board2[2][3], board2[3][3],
	};

	GSPAT::Solver* solver;
	if (solverUsed == BEM_SOLVER) solver = GSPAT_BEM::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	if (solverUsed == GSPAT_SOLVER) solver = GSPAT_V2::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	if (solverUsed == NAIVE_SOLVER) solver = GSPAT_Naive::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	//if (!driver->connect(21, 15))	//Device IDs to connect to
	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	//Program: Create a trap and move it with the keyboard
	float curPos[4 * numGeometries * numPoints];
	float amplitude[numGeometries * numPoints];
	for (int g = 0; g < numGeometries; g++) {
		for (int p = 0; p < numPoints; p++) {
			if (p == 0) {
				curPos[4 * g * numPoints + 4 * p + 0] = 0;
				curPos[4 * g * numPoints + 4 * p + 1] = +0.03f;
				curPos[4 * g * numPoints + 4 * p + 2] = +0.13f;
			}
			else if (p == 1) {
				curPos[4 * g * numPoints + 4 * p + 0] = 0;
				curPos[4 * g * numPoints + 4 * p + 1] = -0.03;
				curPos[4 * g * numPoints + 4 * p + 2] = +0.13f;
			}
			else if (p == 2) {
				curPos[4 * g * numPoints + 4 * p + 0] = -0.02f;
				curPos[4 * g * numPoints + 4 * p + 1] = 0;
				curPos[4 * g * numPoints + 4 * p + 2] = -0.02f;
			}

			curPos[4 * g * numPoints + 4 * p + 3] = 1;
			amplitude[g * numPoints + p] = 10000;
		}
	}
	float matI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float m1[numPoints * 16];
	for (int i = 0; i < numPoints; i++)
		memcpy(&(m1[i * 16]), matI, 16 * sizeof(float));
	//a. create a solution
	GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
	if (solverUsed == BEM_SOLVER) GSPAT_BEM::setAlgorithm(solution, 4, 200);
	solver->compute(solution);

	float* finalPhases = solution->finalArrayPhases();
	float* hologram = new float[2 * 256 * numBoards];
	for (int t = 0; t < 256 * numBoards; t++) {
		hologram[t * 2 + 0] = cosf(finalPhases[t]);
		hologram[t * 2 + 1] = sinf(finalPhases[t]);
	}
	solver->releaseSolution(solution);

	driver->disconnect();
	delete driver;
	delete solver;
	return hologram;
}
void simulateField(float* hologram) {
	const size_t numPoints = 1; //Change this if you want (but make sure to change also each point's position )
	const size_t numGeometries = 1;//Please, do not change this. Multiple update Messages require AsierInhoV2 (see 6.simpleGSPATCpp_AsierInhoV2)

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	GSPAT_BEM::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	int boardIDs[] = { 8, 15 };
	glm::mat4 board1, board2;
	if (setupUsed == DOUBLE_SIDED) {
		// bottom board
		board1 = glm::mat4(1.f);// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, -0.12f));
		// top board
		board2 = glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.24f)) * glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));// glm::translate(glm::mat4(1.f), glm::vec3(0, 0, +0.12f))* glm::rotate(glm::mat4(1.f), glm::radians(180.f), glm::vec3(0, 1, 0));
	}
	if (setupUsed == V_SHAPED) {
		glm::mat4 Rot = glm::rotate(glm::mat4(1.f), glm::radians(-90.f), glm::vec3(0, 0, 1));
		// left board
		glm::mat4 RotL = glm::rotate(glm::mat4(1.f), glm::radians(+45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransL = glm::translate(glm::mat4(1.f), glm::vec3(0, +0.06364f, +0.072125f));
		board1 = TransL * RotL * Rot;
		// right board
		glm::mat4 RotR = glm::rotate(glm::mat4(1.f), glm::radians(-45.f), glm::vec3(1, 0, 0));
		glm::mat4 TransR = glm::translate(glm::mat4(1.f), glm::vec3(0, -0.06364f, +0.072125f));
		board2 = TransR * RotR * Rot;
	}
	float matBoardToWorld[32] = {
		board1[0][0], board1[1][0], board1[2][0], board1[3][0],
		board1[0][1], board1[1][1], board1[2][1], board1[3][1],
		board1[0][2], board1[1][2], board1[2][2], board1[3][2],
		board1[0][3], board1[1][3], board1[2][3], board1[3][3],
		board2[0][0], board2[1][0], board2[2][0], board2[3][0],
		board2[0][1], board2[1][1], board2[2][1], board2[3][1],
		board2[0][2], board2[1][2], board2[2][2], board2[3][2],
		board2[0][3], board2[1][3], board2[2][3], board2[3][3],
	};

	GSPAT::Solver* solver;
	solver = GSPAT_BEM::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	//if (!driver->connect(21, 15))	//Device IDs to connect to
	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	{
		float A[] = { 0, -0.06f, +0.24f }, B[] = { 0, 0.06f, +0.24f }, C[] = { 0, -0.06f, 0 };
		int imgRes[] = { 256,512 };
		visualize(solver, A, B, C, imgRes, hologram);
	}

	driver->disconnect();
	delete driver;
	delete solver;
}

void main() {
	float* hologram = getHologram(solverUsed, setupUsed);
	simulateField(hologram);
	bool loop = true;
	while (loop) {
		switch (getch()) {
		case 'q': loop = false; break;
		}
	}
	//play();
}