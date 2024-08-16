#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverIBP.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverBEM.h>
#include <GSPAT_SolverTS.h>
#include <AsierInho_V2.h>
#include <stdio.h>
#include <conio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <Helper/ReflectionSurface.h>
#include <Helper/VisualizePlane.h>
#include <Helper/microTimer.h>
#include <Helper/HelperMethods.h>
#include "CImg\CImg.h"
#include <Windows.h>

enum Algorithms { NAIVE = 0, IBP, GS_PAT, WGS, TGS };
Algorithms algorithmUsed = TGS;
bool ZERO_THRESHOLD = false;

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
	float* field = GSPAT_TS::simulateFieldFromHologram(solver, hologram, numFieldPoints, fieldPositions);

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
std::vector<std::vector<glm::vec3>> readPaths(std::string fileName, int& numPoints, int& numFrames) {
	std::vector<std::vector<glm::vec3>> paths;
	std::ifstream ifs(fileName);
	if (ifs.is_open()) {
		std::string fileLine;
		bool infoRead = false;
		std::vector<glm::vec3> positions;
		while (getline(ifs, fileLine)) {
			std::stringstream toParse(fileLine);
			std::vector<std::string> lineTokens;
			std::string token;
			while (std::getline(toParse >> std::ws, token, ',')) {
				lineTokens.push_back(token);
			}
			if (!lineTokens.empty()) {
				if (!infoRead) {
					numPoints = atof(lineTokens[0].c_str());
					numFrames = atof(lineTokens[1].c_str());
					infoRead = true;
				}
				else {
					float x = atof(lineTokens[0].c_str());
					float y = atof(lineTokens[1].c_str());
					float z = atof(lineTokens[2].c_str());
					positions.push_back(glm::vec3(x, y, z));
					if (positions.size() == numPoints) {
						paths.push_back(positions);
						positions.clear();
					}
				}
			}
		}
	}
	else std::cout << "ERROR: File cannot be opened!" << std::endl;
	return paths;
}

std::vector<float> getAmplitudes(int numPoints, float*pointPositions, float* transducersPhases, int numTransducers, float* transducerPositions) {
	std::vector<float> amplitudes(numPoints);
	for (int p = 0; p < numPoints; p++) {
		//2.1. Get 3D position of point
		float pixelCoords[3];
		pixelCoords[0] = pointPositions[p * 4 + 0];
		pixelCoords[1] = pointPositions[p * 4 + 1];
		pixelCoords[2] = pointPositions[p * 4 + 2];

		float pixelField[3] = { 0,0,0.0f };
		//2.2. Add contribution from each transducer: 
		for (int t = 0; t < numTransducers; t++) {
			//a. Compute position of the transducer
			float* t_pos = &(transducerPositions[3 * t]);
			//b. Compute distance and amplitude: 
			float distance, amplitude;
			computeAmplitudeAndDistance(t_pos, pixelCoords, &amplitude, &distance);
			//c. Compute complex field: It will be the product of the transducer state and its propagated field
			//   Propagated:
			float phaseDelay = K() * distance;
			float Re_propagated = amplitude * cosf(phaseDelay);
			float Im_propagated = amplitude * sinf(phaseDelay);
			//   Transducer state: the buffer is stored as {{Re1, Im1}, {Re2, Im2}...}
			float Re_q = cosf(transducersPhases[t]);
			float Im_q = sinf(transducersPhases[t]);
			//d. Add to the pixel field (this is still NOT A COLOUR).
			pixelField[0] += Re_q * Re_propagated - Im_q * Im_propagated;
			pixelField[1] += Im_q * Re_propagated + Re_q * Im_propagated;
		}
		//2.3. Map field to pixel colour: 
		float amp_R = sqrt(pixelField[0] * pixelField[0] + pixelField[1] * pixelField[1]);
		amplitudes[p] = amp_R;
	}
	return amplitudes;
}


void simulateStaticScene() {
	const size_t numPoints = 2; //Change this if you want (but make sure to change also each point's position )
	const size_t numGeometries = 1;//Please, do not change this. Multiple update Messages require AsierInhoV2 (see 6.simpleGSPATCpp_AsierInhoV2)

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	GSPAT_TS::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	int boardIDs[] = { 21, 15 };
	float matBoardToWorld[32] = {
		/*bottom*/
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,//-0.12f,
		0, 0, 0, 1,
		/*top*/
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0,-1, 0.24f,//0.12f,
		0, 0, 0, 1,
	};


	GSPAT::Solver* solver = GSPAT_TS::createSolver(256 * numBoards);//Number of transducers used (two boards of 16x16)
	//if (!driver->connect(21, 15))	//Device IDs to connect to
	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);
	GSPAT_TS::setAlgorithm(solver, 2, M_PI / 32.f, M_PI / 32.f);

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
			//if (p == 0) {
			//	curPos[4 * g * numPoints + 4 * p + 0] = 0;
			//	curPos[4 * g * numPoints + 4 * p + 1] = 0;
			//	curPos[4 * g * numPoints + 4 * p + 2] = 0.16f;
			//}
			//else if (p == 1) {
			//	curPos[4 * g * numPoints + 4 * p + 0] = 0;
			//	curPos[4 * g * numPoints + 4 * p + 1] = +0.015f;
			//	curPos[4 * g * numPoints + 4 * p + 2] = -0.09f;
			//}
			//else if (p == 2) {
			//	curPos[4 * g * numPoints + 4 * p + 0] = 0;
			//	curPos[4 * g * numPoints + 4 * p + 1] = -0.015f;
			//	curPos[4 * g * numPoints + 4 * p + 2] = -0.09f;
			//}
			//else if (p == 3) {
			//	curPos[4 * g * numPoints + 4 * p + 0] = 0;
			//	curPos[4 * g * numPoints + 4 * p + 1] = -0.045f;
			//	curPos[4 * g * numPoints + 4 * p + 2] = -0.09f;
			//}

			curPos[4 * g * numPoints + 4 * p + 0] = 0.03;
			curPos[4 * g * numPoints + 4 * p + 1] = 0;
			curPos[4 * g * numPoints + 4 * p + 2] = 0.12f;
			curPos[4 * g * numPoints + 4 * p + 3] = 1;
			amplitude[g * numPoints + p] = 1000;// *(p + 1);
		}
	}
	unsigned char* msg;
	float matI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float m1[numPoints * 16];
	//for (int i = 0; i < numPoints; i++)
	//	memcpy(&(m1[i * 16]), matI, 16 * sizeof(float));
	for (int p = 0; p < numPoints; p++) {
		float matrix[] = { cosf((2 * M_PI * p) / numPoints), -sinf((2 * M_PI * p) / numPoints)	, 0, 0,
							sinf((2 * M_PI * p) / numPoints),  cosf((2 * M_PI * p) / numPoints)	, 0, 0,
											0,							0						, 1, 0,
											0,							0						, 0, 1 };
		memcpy(&(m1[p * 16]), matrix, 16 * sizeof(float));
	}

	//a. create a solution
	GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
	solver->compute(solution);
	solution->finalMessages(&msg);
	for (int s = 0; s < 16; s++)//Fill FPGA buffers so update is processed directly
		driver->updateMessage(msg);

	float targetPoints[2 * 16];
	solution->readTargetPointsReIm(targetPoints);
	for (int i = 0; i < numPoints; i++) {
		float re = targetPoints[i * 2 + 0];
		float im = targetPoints[i * 2 + 1];
		printf("P%d: phase = %f; amplitude = %f\n", i, atan2(im, re), sqrt(re * re + im * im));
	}
	float *finalPhases;
	finalPhases = solution->finalArrayPhases();
	for (int i = 0; i < 512; i++) {
		printf("T%d: phase = %f\n", i, finalPhases[i]);
	}
	{
		//float A[] = { -0.1f, 0, 0.2388f / 2 + 0.1f }, B[] = { 0.1f, 0, 0.2388f / 2 + 0.1f }, C[] = { -0.1f, 0, 0.2388f / 2 - 0.1f };
		//float A[] = { -0.06f, 0, +0.06f }, B[] = { 0.06f, 0, +0.06f }, C[] = { -0.06f, 0, -0.06f };
		//float A[] = { 0, -0.06f, +0.12f }, B[] = { 0, 0.06f, +0.12f }, C[] = { 0, -0.06f, -0.12f };
		//float A[] = { 0, -0.06f, 0.24f }, B[] = { 0, 0.06f, 0.24f }, C[] = { 0, -0.06f, 0 };
		float A[] = { -0.06f, 0, 0.24f }, B[] = { 0.06f, 0, 0.24f }, C[] = { -0.06f,0,  0 };
		int imgRes[] = { 256,512 };
		//visualize(solver, A, B, C, imgRes, solution->finalHologramReIm());
		//VisualizePlane::visualize(A,B,C, imgRes , solution->finalHologramReIm(), 512, transducerPositions);
		VisualizePlane::visualizeFromPhases(A,B,C, imgRes, solution->finalArrayPhases(), 256 * numBoards, transducerPositions);
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
void simulateDynamicScene() {
	const int numMaxGeometries = 1;
	const int numMaxPoints = 32;
	int numPoints, numFrames;
	std::vector<std::vector<glm::vec3>> paths = readPaths("TemporalGS/realPositions.csv", numPoints, numFrames);

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	if (algorithmUsed == NAIVE) GSPAT_Naive::RegisterPrintFuncs(print, print, print);
	if (algorithmUsed == IBP) GSPAT_IBP::RegisterPrintFuncs(print, print, print);
	if (algorithmUsed == GS_PAT) GSPAT_V2::RegisterPrintFuncs(print, print, print);
	if (algorithmUsed == WGS || algorithmUsed == TGS) GSPAT_TS::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	const int numTransducers = 2 * 256;
	int boardIDs[] = { 21, 15 };
	float matBoardToWorld[32] = {
		/*top*/
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0,-1, 0.24f,//0.12f,
		0, 0, 0, 1,
		/*bottom*/
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,//-0.12f,
		0, 0, 0, 1,
	};

	GSPAT::Solver* solver;
	if (algorithmUsed == NAIVE) solver = GSPAT_Naive::createSolver(numTransducers);
	if (algorithmUsed == IBP) solver = GSPAT_IBP::createSolver(numTransducers);
	if (algorithmUsed == GS_PAT) solver = GSPAT_V2::createSolver(numTransducers);
	if (algorithmUsed == WGS || algorithmUsed == TGS) solver = GSPAT_TS::createSolver(numTransducers);

	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	float matI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float m1[numMaxPoints * 16];
	for (int i = 0; i < numMaxPoints; i++)
		memcpy(&(m1[i * 16]), matI, 16 * sizeof(float));

	std::string fileName;
	if (algorithmUsed == NAIVE) fileName = "TemporalGS/OutputNaive.csv";
	else if (algorithmUsed == IBP) fileName = "TemporalGS/OutputIBP.csv";
	else if (algorithmUsed == GS_PAT) fileName = "TemporalGS/OutputGSPAT.csv";
	else if (algorithmUsed == WGS) fileName = "TemporalGS/OutputWGS.csv";
	else if (algorithmUsed == TGS) {
		if (ZERO_THRESHOLD) fileName = "TemporalGS/OutputTGS_zero.csv";
		else fileName = "TemporalGS/OutputTGS_pi32.csv";
	}
	else fileName = "TemporalGS/Output.csv";
	std::ofstream ofs(fileName);
	ofs << "index,phaseDiff,amplitudeDiff,amplitdueSTD," << std::endl;

	std::ofstream ofs2("TemporalGS/Tests.csv");

	int currentGeometry = 0;
	float curPos[4 * numMaxGeometries * numMaxPoints];
	float amplitude[numMaxGeometries * numMaxPoints];
	float targetPoints[2 * numMaxGeometries * numMaxPoints];
	float prevPhases[numMaxGeometries * numTransducers];
	while (currentGeometry < numFrames) {
		// fill the buffers
		int numGeometries = min(numMaxGeometries, numFrames - currentGeometry);
		for (int g = 0; g < numGeometries; g++) {
			//printf("Current geometry = %d\n", currentGeometry + g);
			for (int p = 0; p < numPoints; p++) {
				glm::vec3 point = paths[currentGeometry + g][p];
				//printf("P%d: (%f, %f, %f)\n", p, point.x, point.y, point.z);
				curPos[4 * g * numPoints + 4 * p + 0] = paths[currentGeometry + g][p].x;
				curPos[4 * g * numPoints + 4 * p + 1] = paths[currentGeometry + g][p].y;
				curPos[4 * g * numPoints + 4 * p + 2] = paths[currentGeometry + g][p].z;
				amplitude[g * numPoints + p] = 1000;
			}
		}
		
		//a. create a solution
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
		if (algorithmUsed == WGS) GSPAT_TS::setAlgorithm(solver, 100, M_PI, M_PI);
		if (algorithmUsed == TGS) {
			if (ZERO_THRESHOLD) GSPAT_TS::setAlgorithm(solver, 100, M_PI / 32.f, 0.f);
			else GSPAT_TS::setAlgorithm(solver, 100, M_PI / 32.f, M_PI / 32.f);
		}
		solver->compute(solution);

		float* currPhases = solution->finalArrayPhases();
		solution->readTargetPointsReIm(targetPoints, numGeometries);
		unsigned char* msg;
		solution->finalMessages(&msg);
		for (int g = 0; g < numGeometries; g++) {
			float phaseDiffMean = 0;
			for (int t = 0; t < numTransducers; t++) {
				if (currentGeometry + g > 0) {
					float phaseDiff = currPhases[g*numTransducers + t] - prevPhases[t];
					if (phaseDiff < -M_PI)
						phaseDiff += 2. * M_PI;
					if (phaseDiff > +M_PI)
						phaseDiff -= 2. * M_PI;
					phaseDiffMean += abs(phaseDiff);
				}
				prevPhases[t] = currPhases[g * numTransducers + t];
				ofs2 << (int)msg[g * numTransducers*2 + t] << ",";
			}
			ofs2 << std::endl;
			phaseDiffMean /= (float)(numTransducers);

			//float amplitudeMean = 0;
			//for (int p = 0; p < numPoints; p++) {
			//	float re = targetPoints[(g * numPoints + p) * 2 + 0];
			//	float im = targetPoints[(g * numPoints + p) * 2 + 1];
			//	amplitudeMean += sqrt(re * re + im * im);
			//	printf("P%d: phase = %f, amplitude = %f\n", p, atan2(im, re), sqrt(re * re + im * im));
			//}
			//amplitudeMean /= (float)(numPoints);
			//float amplitudeSTD = 0;
			//for (int p = 0; p < numPoints; p++) {
			//	float re = targetPoints[(g * numPoints + p) * 2 + 0];
			//	float im = targetPoints[(g * numPoints + p) * 2 + 1];
			//	float diff = amplitudeMean - sqrt(re * re + im * im);
			//	amplitudeSTD += diff * diff;
			//}
			//amplitudeSTD = sqrt(amplitudeSTD / (numPoints - 1.f));

			std::vector<float> focalAmplitudes = getAmplitudes(numPoints, &curPos[4 * g * numPoints], &currPhases[g * numTransducers], numTransducers, transducerPositions);
			float amplitudeMean = 0;
			for (int p = 0; p < numPoints; p++) {
				amplitudeMean += focalAmplitudes[p];
				//printf("P%d: phase = %f, amplitude = %f\n", p, 0, focalAmplitudes[p]);
			}
			amplitudeMean /= (float)(numPoints);
			float amplitudeSTD = 0;
			for (int p = 0; p < numPoints; p++) {
				float diff = amplitudeMean - focalAmplitudes[p];
				amplitudeSTD += diff * diff;
			}
			amplitudeSTD = sqrt(amplitudeSTD / (numPoints - 1.f));

			std::cout << currentGeometry + g << ", " << phaseDiffMean << ", " << amplitudeMean << ", " << amplitudeSTD << ", " << std::endl;
			ofs << currentGeometry + g << "," << phaseDiffMean << ", " << amplitudeMean << ", " << amplitudeSTD << ", " << std::endl;
		}
		solver->releaseSolution(solution);
		currentGeometry += numMaxGeometries;
	}
	ofs.close();
	ofs2.close();

	//Deallocate the AsierInho controller: 
	driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	delete solver;
}
void simulateSpeed() {
	const int numMaxGeometries = 32;
	const int numMaxPoints = 32;

	//Create driver and connect to it
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	AsierInho_V2::AsierInhoBoard_V2* driver = AsierInho_V2::createAsierInho();
	//Create solver:
	GSPAT_TS::RegisterPrintFuncs(print, print, print);

	int numBoards = 2;
	const int numTransducers = 2 * 256;
	int boardIDs[] = { 21, 15 };
	float matBoardToWorld[32] = {
		/*top*/
		-1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0,-1, 0.12f,
		0, 0, 0, 1,
		/*bottom*/
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, -0.12f,
		0, 0, 0, 1,
	};

	GSPAT::Solver* solver = GSPAT_TS::createSolver(numTransducers);//Number of transducers used (two boards of 16x16)
	if (!driver->connect(numBoards, boardIDs, matBoardToWorld))
		printf("Failed to connect to board.");
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	driver->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	float matI[] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	float m1[numMaxPoints * 16];
	for (int i = 0; i < numMaxPoints; i++)
		memcpy(&(m1[i * 16]), matI, 16 * sizeof(float));

	float curPos[4 * numMaxGeometries * numMaxPoints];
	float amplitude[numMaxGeometries * numMaxPoints];
	// fill the buffers
	int numPoints = 16;
	int numGeometries = numMaxGeometries;
	for (int g = 0; g < numGeometries; g++) {
		for (int p = 0; p < numPoints; p++) {
			curPos[4 * g * numPoints + 4 * p + 0] = 0;
			curPos[4 * g * numPoints + 4 * p + 1] = 0;
			curPos[4 * g * numPoints + 4 * p + 2] = 0.12f;
			amplitude[g * numPoints + p] = 1000;
		}
	}

	{
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
		GSPAT_TS::setAlgorithm(solver, 100, M_PI / 32.f, M_PI / 32.f);
		solver->compute(solution);
		unsigned char* msg;
		solution->finalMessages(&msg);
		solver->releaseSolution(solution);
	}


	int numTests = 1000;
	DWORD startTime = microTimer::uGetTime();
	for (int i = 0; i < numTests;i++) {
		//a. create a solution
		GSPAT::Solution* solution = solver->createSolution(numPoints, numGeometries, true, curPos, amplitude, m1, m1);
		GSPAT_TS::setAlgorithm(solver, 2, M_PI / 32.f, M_PI / 32.f);
		solver->compute(solution);

		unsigned char* msg;
		solution->finalMessages(&msg);
		solver->releaseSolution(solution);
		if(i%100==0) printf("Test %d\n", i);
	}
	DWORD endTime = microTimer::uGetTime();
	printf("It took %f sec to update %d geometries\n", (endTime - startTime) / 1000000.f, numGeometries * numTests);
	printf("Therefore, its update rate is %f ups\n", numGeometries * numTests * 1000000.f / (endTime - startTime));

	//Deallocate the AsierInho controller: 
	driver->turnTransducersOff();
	Sleep(100);
	driver->disconnect();
	delete driver;
	delete solver;
}


void main() {
	simulateStaticScene();
	//simulateDynamicScene();
	//simulateSpeed();
}