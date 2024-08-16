#include <AsierInho_V2.h>
#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverIBP.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverTS.h>

#include <Helper/microTimer.h>
#include <S2M2/cpp/PathPlanner.h>
#include <clientPrograms/Applications/LeviPropRecognizer/BeadDetector.h>
#include <GenerateSpeedTestPaths.h>

#include <deque>
#include <conio.h>

enum Algorithms { GS_PAT = 0, TEMPORAL, NAIVE };
Algorithms algorithmUsed = GS_PAT;
int numPoints = 8;
bool verticalTest = false;
//float angleDegree = -30.f;
float a_step = (numPoints <= 8 ? 0.1f : 0.01f);
float a0 = 2; //Acceleration to test in meter/second
int numIterations = 5;
float transducerThreshold = M_PI / 32.f;  //M_PI / 32.f;   M_PI; 
float pointThreshold = 0;      //0;   M_PI;  
bool initializeCamera = false;
float targetAmplitude = 1.0f;
unsigned char amplitudeDisc = (float)((64 * 2.0f / M_PI) * asin(targetAmplitude));


int failedTimes = 0;
int totalTestTimes = 0;

AsierInho_V2::AsierInhoBoard_V2* a;		// this version (V2) of AsierInho has the function to send 32 geometries at the same time
float centreHeight = 0.12f;				// ?? distance between the top and bottom boards
bool running = true;					// indicates the threads are still running
float targetUPS = 10000;				// target updates per second
const int numGeometries = 32;					// number of geometries to be sent at the same time
pthread_mutex_t mutex_solution_queue;
pthread_mutex_t solution_available;
std::deque<GSPAT::Solution*> solutions;
std::deque<bool> transducerOffSignals;
pthread_t readerThread, writerThread;

bool turnTransducersOff = false;						// transducers can be off before detecting the particle positions
bool pickingup = true;
bool startTesting = false;
bool startPlanningControl = false;
bool run4p_updown = false;

void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false, bool targetConstraint = false);
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, float*& posBuffer, float*& ampBuffer);
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);

void print(const char* str);

void* solutionReaderThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	while (running) {
		//0. Check the Queue:
		pthread_mutex_lock(&mutex_solution_queue);
		while (solutions.size() == 0) {
			//1. There is nothing to be done... wait until some solution is produced.
			pthread_mutex_unlock(&mutex_solution_queue);
			pthread_mutex_lock(&solution_available);
			pthread_mutex_lock(&mutex_solution_queue);
		}
		//1. We got access! 
		{
			//2. Let's get the first solution in the queue (and unlock, so others can keep adding/removing)
			GSPAT::Solution* curSolution = solutions[0];
			solutions.pop_front();
			bool transducerOffSignal = transducerOffSignals[0];
			transducerOffSignals.pop_front();
			pthread_mutex_unlock(&mutex_solution_queue);
			// read the final phases and discretise. 
			unsigned char* finalMessages = NULL;
			curSolution->finalMessages(&finalMessages);
			for (int g = 0; g < numGeometries; g++) {
				memset(&finalMessages[g*512+256], amplitudeDisc, 256);
				memset(&finalMessages[(g+numGeometries)*512+256], amplitudeDisc, 256);
			}
			updateGeometries(finalMessages, numGeometries, transducerOffSignal);
			solver->releaseSolution(curSolution);
		}
	}
	return NULL;
}
void* solutionWriterThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	// 1. Fill buffers: we set all the positions at (0, 0, 0.12), the centre of the working volume
	int pBufferSize, aBufferSize;
	float* posBuffer, * ampBuffer;
	fillBuffers(numPoints, pBufferSize, aBufferSize, posBuffer, ampBuffer);
	float* circlePath_data;
	size_t sizeCirclePath;
	float O[] = { 0, 0, centreHeight, 1 };
	float radius = 0.035f;
/*	if (!verticalTest) createHorizontalCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, &circlePath_data, &sizeCirclePath, 3);
	else createVerticalCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, &circlePath_data, &sizeCirclePath, 3);
	*///createArbitraryAngleCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, angleDegree, &circlePath_data, &sizeCirclePath, 3);

	// 2. Initialize the bead detector
	float stageHeight = 0.0375f; // the height of the stage (work great to set a little bit above the surface)   0.0355f;  0.031f; 
	float p1[] = { -0.084f, -0.084f, stageHeight },
		p2[] = { 0.084f, -0.084f, stageHeight },
		p3[] = { 0.084f,  0.084f, stageHeight },
		p4[] = { -0.084f,  0.084f, stageHeight };
	BeadDetector& detector = BeadDetector::instance(p1, p2, p3, p4);
	if (!initializeCamera) detector.loadBackground();
	detector.startDetection();

	// 3. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight - 0.06f);
	glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight + 0.06f);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.007, 0.007, 0.007);	// size of the agents in meter
	float velocity = 0.05f;					// maximum velocity in meter/second   0.05
	// define initial positions
	std::vector<AgentPath> paths(numPoints);
	std::vector<glm::vec3> initialPositions;
	std::vector<glm::vec3> targetPositions;

	float r1 = 0.03f;    //circle radius
	float r2 = 0.02f;    //cube radius

	/*for (int i = 0; i < numPoints; i++) {
		initialPositions.push_back(glm::vec3(circlePath_data[i * 4 + 0], circlePath_data[i * 4 + 1], circlePath_data[i * 4 + 2]));
		paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	}*/
	delete[] circlePath_data;

	 //TEST 0-2 AC-UL

	//initialPositions = {

	//	//A
	//	glm::vec3(0.040, 0.0, 0.02 + centreHeight),
	//	glm::vec3(0.020, 0.013, 0.02 + centreHeight),
	//	glm::vec3(-0.01, 0.030, 0.02 + centreHeight),
	//	glm::vec3(-0.03, 0.045, 0.02 + centreHeight),
	//	glm::vec3(0.020, -0.013, 0.02 + centreHeight),
	//	glm::vec3(-0.01, 0.0, 0.02 + centreHeight),
	//	glm::vec3(-0.01, -0.030, 0.02 + centreHeight),
	//	glm::vec3(-0.03, -0.045, 0.02 + centreHeight),

	//	//C
	//	glm::vec3(0.030, -0.035, -0.02 + centreHeight),
	//	glm::vec3(0.040, -0.010, -0.02 + centreHeight),
	//	glm::vec3(0.035, 0.020, -0.02 + centreHeight),
	//	glm::vec3(0.015, 0.035, -0.02 + centreHeight),
	//	glm::vec3(-0.015, 0.035, -0.02 + centreHeight),
	//	glm::vec3(-0.035, 0.020, -0.02 + centreHeight),
	//	glm::vec3(-0.040, -0.010, -0.02 + centreHeight),
	//	glm::vec3(-0.030, -0.035, -0.02 + centreHeight),
	//};
	//for (int i = 0; i < initialPositions.size(); i++)
	//{
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}

	//// target shape 

	//	
	//targetPositions = {
	//	//U
	//	glm::vec3(0.040, 0.035, 0.02 + centreHeight),
	//	glm::vec3(0.010, 0.035, 0.02 + centreHeight),
	//	glm::vec3(-0.020, 0.035, 0.02 + centreHeight),
	//	glm::vec3(-0.040, 0.015, 0.02 + centreHeight),
	//	glm::vec3(-0.040,-0.015, 0.02 + centreHeight),
	//	glm::vec3(-0.020,-0.035, 0.02 + centreHeight),
	//	glm::vec3(0.010,-0.035, 0.02 + centreHeight),
	//	glm::vec3(0.040,-0.035, 0.02 + centreHeight),

	//	//L
	//	glm::vec3(0.04, 0.03, -0.02 + centreHeight),
	//	glm::vec3(0.02, 0.03, -0.02 + centreHeight),
	//	glm::vec3(0.00, 0.03, -0.02 + centreHeight),
	//	glm::vec3(-0.02, 0.03, -0.02 + centreHeight),
	//	glm::vec3(-0.04, 0.03, -0.02 + centreHeight),
	//	glm::vec3(-0.04, 0.01, -0.02 + centreHeight),
	//	glm::vec3(-0.04, -0.01, -0.02 + centreHeight),
	//	glm::vec3(-0.04, -0.03, -0.02 + centreHeight),


	//	};

	////TEST 0: two circle + 2 middle points => butterfly
	////top circle
	//for (int i = 0; i < 4; i++) {
	//	initialPositions.push_back(glm::vec3(r2 * cos(i * 2* M_PI / (float)numPoints), r2 * sin(i * 2* M_PI / (float)numPoints), centreHeight-0.03f));	
	//}
	////bottom circle
	//for (int i = 0; i < 6; i++) {
	//	initialPositions.push_back(glm::vec3(r1 * cos(i * 2 * M_PI / (float)numPoints), r1 * sin(i * 2 * M_PI / (float)numPoints), centreHeight+0.03f));	
	//}
	//initialPositions.push_back(glm::vec3(0,-0.01f,centreHeight)); initialPositions.push_back(glm::vec3(0, 0.01f, centreHeight));

	//for (int i = 0; i < numPoints; i++) {
	//
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}
	//targetPositions.push_back(glm::vec3(0, 0, 0.01f + centreHeight)); targetPositions.push_back(glm::vec3(0, 0, -0.01f + centreHeight));
	//targetPositions.push_back(glm::vec3(0, 0.03f, 0.02f + centreHeight)); targetPositions.push_back(glm::vec3(0, -0.03f, 0.02f + centreHeight));
	//targetPositions.push_back(glm::vec3(0, 0.055f, 0.04f + centreHeight)); targetPositions.push_back(glm::vec3(0, -0.055f, 0.04f + centreHeight));
	//targetPositions.push_back(glm::vec3(0, 0.045f, 0.01f + centreHeight)); targetPositions.push_back(glm::vec3(0, -0.045f, 0.01f + centreHeight));
	//targetPositions.push_back(glm::vec3(0, 0.035f, -0.035f + centreHeight)); targetPositions.push_back(glm::vec3(0, 0.035f, -0.035f + centreHeight));
	//targetPositions.push_back(glm::vec3(0, 0.02f, -0.02f + centreHeight)); targetPositions.push_back(glm::vec3(0, -0.02f, -0.02f + centreHeight));

	////TEST 0-3: 4 up-down + 4 left-right
	////bottle

	//initialPositions.push_back(glm::vec3(-0.02f, -0.03f, centreHeight)); initialPositions.push_back(glm::vec3(-0.02f, -0.01f, centreHeight));
	//initialPositions.push_back(glm::vec3(-0.02f, 0.01f, centreHeight)); initialPositions.push_back(glm::vec3(-0.02f, 0.03f, centreHeight));
	//initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight - 0.045f)); initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight - 0.015f));
	//initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight + 0.015f)); initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight + 0.045f));


	////initialPositions.push_back(glm::vec3(0, 0, centreHeight - 0.02f));
	//for (int i = 0; i < numPoints; i++) {

	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}

	//targetPositions.push_back(glm::vec3(-0.02f, -0.03f, centreHeight + 0.03f)); targetPositions.push_back(glm::vec3(-0.02f, -0.01f, centreHeight - 0.03f));
	//targetPositions.push_back(glm::vec3(-0.02f, 0.01f, centreHeight + 0.03f)); targetPositions.push_back(glm::vec3(-0.02f, 0.03f, centreHeight - 0.03f));
	//targetPositions.push_back(glm::vec3(0.02f, 0.03f, centreHeight - 0.045f)); targetPositions.push_back(glm::vec3(0.02f, -0.03f, centreHeight - 0.015f));
	//targetPositions.push_back(glm::vec3(0.02f, 0.03f, centreHeight + 0.015f)); targetPositions.push_back(glm::vec3(0.02f, -0.03f, centreHeight + 0.045f));


	//TEST1: circle and cube animation (8 particles)
	for (int i = 0; i < numPoints; i++) {
		//initialPositions.push_back(glm::vec3(r1 * cos(i * 2* M_PI / (float)numPoints), r1 * sin(i * 2* M_PI / (float)numPoints), centreHeight));
		initialPositions.push_back(glm::vec3(r1 * cos(i * M_PI / 4.f), r1 * sin(i *  M_PI / 4.f), centreHeight));
		paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	}
	targetPositions.push_back(glm::vec3(+r2, +r2, +r2+centreHeight)); targetPositions.push_back(glm::vec3(+r2, +r2, -r2+centreHeight)); targetPositions.push_back(glm::vec3(+r2, -r2, +r2+centreHeight)); targetPositions.push_back(glm::vec3(+r2, -r2, -r2+centreHeight));targetPositions.push_back(glm::vec3(-r2, +r2, +r2+centreHeight)); targetPositions.push_back(glm::vec3(-r2, +r2, -r2+centreHeight)); targetPositions.push_back(glm::vec3(-r2, -r2, +r2+centreHeight)); targetPositions.push_back(glm::vec3(-r2, -r2, -r2+centreHeight));
	//targetPositions.push_back(glm::vec3(+r2+0.01f, +r2, +r2 + centreHeight)); targetPositions.push_back(glm::vec3(+r2, +r2, -r2 + centreHeight)); targetPositions.push_back(glm::vec3(+r2+0.01f, -r2, +r2 + centreHeight)); targetPositions.push_back(glm::vec3(+r2, -r2, -r2 + centreHeight)); targetPositions.push_back(glm::vec3(-r2+0.01f, +r2, +r2 + centreHeight)); targetPositions.push_back(glm::vec3(-r2, +r2, -r2 + centreHeight)); targetPositions.push_back(glm::vec3(-r2+0.01f, -r2, +r2 + centreHeight)); targetPositions.push_back(glm::vec3(-r2, -r2, -r2 + centreHeight));


	///*for (int i = 0; i < numPoints; i++)
	//{
	//	paths[i].addWayPoint(0, targetPositions[i].x, targetPositions[i].y, targetPositions[i].z);
	//}*/

	////TEST2: 4 beads going-up animation
	//for (int i = 0; i < numPoints; i++) {
	//	initialPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight - 0.03f));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}
	//for (int i = 0; i < numPoints; i++) {
	//	targetPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight + 0.03f));
	//}

	////TEST3: 2 beads going-up, 2 beads going-down  (start with the same height)
	//for (int i = 0; i < numPoints; i++) {
	//	initialPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}
	//for (int i = 0; i < numPoints; i++) {
	//	if(i%2==0) targetPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight + 0.03f));

	//	if(i%2!=0) targetPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight - 0.03f));
	//}

	////TEST 4: 2 beads going-up, 2 beads going-down (start with the same plane, different height,square shape)
	//for (int i = 0; i < numPoints; i++) {
	//	if (i == 0) initialPositions.push_back(glm::vec3(0, -0.015f, centreHeight+0.015f));
	//	if (i == 1) initialPositions.push_back(glm::vec3(0, -0.015f, centreHeight-0.015f));
	//	if (i == 2) initialPositions.push_back(glm::vec3(0, 0.015f, centreHeight+0.015f));
	//	if (i == 3) initialPositions.push_back(glm::vec3(0, 0.015f, centreHeight-0.015f));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}
	//for (int i = 0; i < numPoints; i++) {
	//	if (i == 0) targetPositions.push_back(initialPositions[i] + glm::vec3(0, 0, 0.03f));
	//	if (i == 1) targetPositions.push_back(initialPositions[i] + glm::vec3(0, 0, -0.03f));
	//	if (i == 2) targetPositions.push_back(initialPositions[i] + glm::vec3(0, 0, 0.03f));
	//	if (i == 3) targetPositions.push_back(initialPositions[i] + glm::vec3(0, 0, -0.03f));
	//}


	////TEST5: 2 beads going-right, 2 beads going-left (start with two different heights,rhombus shape)  do not use task assignment
	//for (int i = 0; i < numPoints; i++) {
	//	if (i == 0) initialPositions.push_back(glm::vec3(0, 0.03f, centreHeight - 0.015f));
	//	if (i == 1) initialPositions.push_back(glm::vec3(0, 0.01f, centreHeight - 0.015f));
	//	if (i == 2) initialPositions.push_back(glm::vec3(0, -0.01f, centreHeight + 0.015f));
	//	if (i == 3) initialPositions.push_back(glm::vec3(0, -0.03f, centreHeight + 0.015f));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}
	//for (int i = 0; i < numPoints; i++) {
	//	if (i == 0) targetPositions.push_back(initialPositions[i] + glm::vec3(0, -0.04f, 0));
	//	if (i == 1) targetPositions.push_back(initialPositions[i] + glm::vec3(0, -0.04f, 0));
	//	if (i == 2) targetPositions.push_back(initialPositions[i] + glm::vec3(0, +0.04f, 0));
	//	if (i == 3) targetPositions.push_back(initialPositions[i] + glm::vec3(0, +0.04f, 0));
	//}



////TEST 6: 4 up-down + 4 left-right
//	//bottle
//	
//    initialPositions.push_back(glm::vec3(-0.02f, -0.03f, centreHeight)); initialPositions.push_back(glm::vec3(-0.02f, -0.01f, centreHeight));
//    initialPositions.push_back(glm::vec3(-0.02f, 0.01f, centreHeight)); initialPositions.push_back(glm::vec3(-0.02f, 0.03f, centreHeight));
//    initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight - 0.045f)); initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight - 0.015f));
//    initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight + 0.015f)); initialPositions.push_back(glm::vec3(0.02f, 0, centreHeight + 0.045f));
//
//
//    for (int i = 0; i < numPoints; i++) {
//
//	    paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
//    }
//
//    targetPositions.push_back(glm::vec3(-0.02f, -0.03f, centreHeight + 0.03f)); targetPositions.push_back(glm::vec3(-0.02f, -0.01f, centreHeight - 0.03f));
//    targetPositions.push_back(glm::vec3(-0.02f, 0.01f, centreHeight + 0.03f)); targetPositions.push_back(glm::vec3(-0.02f, 0.03f, centreHeight - 0.03f));
//    targetPositions.push_back(glm::vec3(0.02f, 0.03f, centreHeight - 0.045f)); targetPositions.push_back(glm::vec3(0.02f, -0.03f, centreHeight - 0.015f));
//    targetPositions.push_back(glm::vec3(0.02f, 0.03f, centreHeight + 0.015f)); targetPositions.push_back(glm::vec3(0.02f, -0.03f, centreHeight + 0.045f));

  //    //TEST 7 4-up 4-down
	
		//initialPositions.push_back(glm::vec3(-0.015f, -0.015f, centreHeight+0.015f));
		//initialPositions.push_back(glm::vec3(-0.015f, -0.015f, centreHeight-0.015f));
		//initialPositions.push_back(glm::vec3(-0.015f, 0.015f, centreHeight+0.015f));
		//initialPositions.push_back(glm::vec3(-0.015f, 0.015f, centreHeight-0.015f));

		//initialPositions.push_back(glm::vec3(0.015f, -0.015f, centreHeight + 0.015f));
		//initialPositions.push_back(glm::vec3(0.015f, -0.015f, centreHeight - 0.015f));
		//initialPositions.push_back(glm::vec3(0.015f, 0.015f, centreHeight + 0.015f));
		//initialPositions.push_back(glm::vec3(0.015f, 0.015f, centreHeight - 0.015f));
		//
		//for (int i = 0; i < numPoints; i++)
	 //   {
		//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	 //   }

		//targetPositions.push_back(initialPositions[0] + glm::vec3(0, 0, 0.03f));
		//targetPositions.push_back(initialPositions[1] + glm::vec3(0, 0, -0.03f));
		//targetPositions.push_back(initialPositions[2] + glm::vec3(0, 0, 0.03f));
		//targetPositions.push_back(initialPositions[3] + glm::vec3(0, 0, -0.03f));

		//targetPositions.push_back(initialPositions[4] + glm::vec3(0, 0, 0.03f));
		//targetPositions.push_back(initialPositions[5] + glm::vec3(0, 0, -0.03f));
		//targetPositions.push_back(initialPositions[6] + glm::vec3(0, 0, 0.03f));
		//targetPositions.push_back(initialPositions[7] + glm::vec3(0, 0, -0.03f));


		//for circle to cube: target constraint= true; cube to circle: target constraint = false
		//for AC to UL : target constraint = false; UL to AC : target constraint = true
	std::vector<AgentPath> goPaths = setPaths(planner, initialPositions, targetPositions, size, velocity, true, true);
	std::vector<AgentPath> returnPaths = setPaths(planner, targetPositions, initialPositions, size, velocity, true, false);

	std::string TestName = "8_circle2cube";
	std::stringstream PathFile;       //store the paths for every test
	PathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo/Output/PathSaving" << "/" << TestName << "-1" << "-v-" << velocity << "-" << "pathFile.csv";
	PathPlanner::saveAgentPaths(PathFile.str(), goPaths);
	PathFile.str(""); //renew the string
	PathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo/Output/PathSaving" << "/" << TestName << "-2" << "-v-" << velocity << "-" << "pathFile.csv";
	PathPlanner::saveAgentPaths(PathFile.str(), returnPaths);

	//// combining go and return paths
	//float maxTime = 0.0f;
	//for (int i = 0; i < goPaths.size(); i++)
	//{
	//	float time = goPaths[i].getFinalTime();
	//	if (maxTime < time)
	//		maxTime = time;

	//	printf("Agent %d finalTime: %f \n", i, time);
	//	printf("Current largest time %f \n", maxTime);
	//}

	//for (int i = 0; i < returnPaths.size(); i++)
	//{
	//	for (int j = 0; j < returnPaths[i].getNumPoints(); j++)
	//	{
	//		AgentPath::WayPoint waypoint = returnPaths[i].getPoint(j);
	//		std::cout << "Point-" << i << "," << waypoint.t << "," << waypoint.x << "," << waypoint.y << "," << waypoint.z << std::endl;
	//		//returnPaths_revise[i].addWayPoint(waypoint.t+maxTime, waypoint.x,waypoint.y,waypoint.z);
	//		goPaths[i].addWayPoint(waypoint.t + maxTime, waypoint.x, waypoint.y, waypoint.z);
	//	}
	//}

	/*std::vector<AgentPath> goPaths = setPaths(planner, targetPositions,initialPositions,  size, velocity, true, true);
	std::vector<AgentPath> returnPaths = setPaths(planner,initialPositions, targetPositions,  size, velocity, true, true);*/

	// 4. Define matrices to update the positions of the traps
	float initMat[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	float mStarts[16 * 16], mEnds[16 * 16];
	static const size_t X_index = 3, Y_index = 7, Z_index = 11;
	for (int p = 0; p < numPoints; p++) {
		memcpy(&(mStarts[16 * p]), initMat, 16 * sizeof(float));
		memcpy(&(mEnds[16 * p]), initMat, 16 * sizeof(float));
		glm::vec3 pos = paths[p].getPosition(0);
		mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
		mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
		mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
	}

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	float currTime = 0;					     		// current (virtual) time 
	bool moving = false;							// indicates the particles are moving or not
	bool testMode = false;
	bool finished = false;
	bool going = true;

	int pIndex = 0;
	while (running) {
		// 5.1. Picking up from a bed
		if (pickingup) {
			if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			moving = true;
			currTime = 0;
			// detect the particle positions

			std::cout << "****Here we pick particles up.****" << std::endl;

			std::vector<glm::vec3> detectedPositions = getDetectedPositions(detector, numPoints);
			float aboveHeight = 0.061f;
			float stopDuration = 1.f;
			float initVelocity = 0.04f;
			float initialTime = (aboveHeight - stageHeight) / initVelocity + stopDuration;
			for (int i = 0; i < numPoints; i++) {
				detectedPositions[i].z = aboveHeight;
				std::cout << "Initial Pos Part." << i << ": x: " << detectedPositions[i].x << "y: " << detectedPositions[i].y << "\n";
			}
			
			// compute the paths
			std::vector<AgentPath> tmpPaths = setPaths(planner, detectedPositions, initialPositions, size, initVelocity, true, true);

			if (tmpPaths.size() == numPoints) {
				for (int i = 0; i < numPoints; i++) {
					glm::vec3 initPos = tmpPaths[i].getPosition(0);
					paths[i].clearWaypoints();
					paths[i].addWayPoint(0.f, initPos.x, initPos.y, stageHeight);
					paths[i].addWayPoint(stopDuration, initPos.x, initPos.y, stageHeight + 0.01f);
					for (int j = 0; j < tmpPaths[i].getNumPoints(); j++)
						paths[i].addWayPoint(initialTime + tmpPaths[i].getPoint(j).t, tmpPaths[i].getPoint(j).x, tmpPaths[i].getPoint(j).y, tmpPaths[i].getPoint(j).z);
				}
				// update the matrices
				for (int p = 0; p < numPoints; p++) {
					glm::vec3 pos = paths[p].getPosition(0);
					mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
					mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
					mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
				}
				turnTransducersOff = false; // turn the transducers on
				pickingup = false;
			}
			else {
				pickingup = false;
			}
		}
		// 5.2. Create circle test paths
		if (startTesting) {
			testMode = true;
			if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			if (!verticalTest) createHorizontalCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, &circlePath_data, &sizeCirclePath, 3);
			else createVerticalCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, &circlePath_data, &sizeCirclePath, 3);
			//createArbitraryAngleCircleTest(O, radius, a0, 1.0f / targetUPS, numPoints, angleDegree, &circlePath_data, &sizeCirclePath, 3);
			for (int p = 0; p < numPoints; p++) {
				memcpy(&(mStarts[16 * p]), initMat, 16 * sizeof(float));
				memcpy(&(mEnds[16 * p]), initMat, 16 * sizeof(float));
			}
			startTesting = false;
		}

		if (startPlanningControl) {
			//++totalTestTimes;
			//printf("totalTestTimes: %d \n",totalTestTimes);
			//if(totalTestTimes <= 1)    //if(totalTestTimes % 2 == 0)
				if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			  
			currTime = 0;
			moving = true;
			paths = (going ? goPaths : returnPaths);
			going = !going;
			startPlanningControl = false;		
		}
		if (run4p_updown) {
			//if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			initialPositions.clear();
			targetPositions.clear();
			//TEST2: 4 beads going-up animation
			for (int i = 0; i < numPoints; i++) {
				initialPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight));				
			}
			for (int i = 0; i < numPoints; i++) {
				if (i % 2 == 0) targetPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight + 0.03f));

				if (i % 2 != 0) targetPositions.push_back(glm::vec3(0, -0.03f + i * 0.02f, centreHeight - 0.03f));
			}
			velocity = 0.08f;
			goPaths = setPaths(planner, initialPositions, targetPositions, size, velocity, true, true);
			returnPaths = setPaths(planner, targetPositions, initialPositions, size, velocity, true, false);
			run4p_updown = false;
		}
		// 5.3. Update matrices for picking up (also for the planning control)
		if (!testMode) {
			finished = false;   //finished = false
			for (int p = 0; p < numPoints; p++) {
				glm::vec3 pos = paths[p].getPosition(currTime); // get the positions of the agents according to the current time
				mStarts[16 * p + X_index] = mEnds[16 * p + X_index];
				mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index];
				mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index];
				mEnds[16 * p + X_index] = pos.x;
				mEnds[16 * p + Y_index] = pos.y;
				mEnds[16 * p + Z_index] = pos.z;
				finished &= (paths[p].getFinalTime() < currTime); // check if every agent has finished its path
			}
			if (moving) currTime += dt; // increment the current time
		}
		// 5.4. Compute a hologram
		GSPAT::Solution* solution;
		if (testMode) solution = solver->createSolution(numPoints, numGeometries, true, &(circlePath_data[pIndex]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		else solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[0]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
		solver->compute(solution);
		// 5.5. Pass the solution to the reader thread
		pthread_mutex_lock(&mutex_solution_queue);
		solutions.push_back(solution);
		transducerOffSignals.push_back(turnTransducersOff);
		pthread_mutex_unlock(&solution_available);
		pthread_mutex_unlock(&mutex_solution_queue);
		// 5.6. Update the status
		if (testMode) {
			pIndex += numPoints * numGeometries * 4;
			if (pIndex > sizeCirclePath - numPoints * numGeometries * 4) {
				pIndex = 0;
				testMode = false;
				for (int p = 0; p < numPoints; p++) {
					glm::vec3 pos = paths[p].getPosition(paths[p].getFinalTime());
					mStarts[16 * p + X_index] = mEnds[16 * p + X_index] = pos.x;
					mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index] = pos.y;
					mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index] = pos.z;
				}
				delete[] circlePath_data;
			}
		}
		else {
			if (finished) {
				moving = false;
				finished = false;
			}
		}
	}

	delete[] posBuffer, ampBuffer;
	return NULL;
}

void main() {
	std::cout << "**FILE USED** PlanningIntegration_clean.cpp\n" << std::endl;

	//STAGE 0: Initialize
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	a = AsierInho_V2::createAsierInho();
	a->connect(3, 23);  //3,23  21,15    2,4
	a->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);
	//STAGE 1: Initialize the solver
	GSPAT::Solver* solver;
	switch (algorithmUsed) {
	case GS_PAT:
		GSPAT_V2::RegisterPrintFuncs(print, print, print);
		solver = GSPAT_V2::createSolver(512);
		break;
	case TEMPORAL:
		GSPAT_TS::RegisterPrintFuncs(print, print, print);
		solver = GSPAT_TS::createSolver(512);
		GSPAT_TS::setAlgorithm(solver, numIterations, transducerThreshold, pointThreshold);
		break;
	case NAIVE:
		GSPAT_Naive::RegisterPrintFuncs(print, print, print);
		solver = GSPAT_Naive::createSolver(512);
		break;
	}
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);
	//STAGE 2: Create the reader and writer threads and the concurrency control variables:
	pthread_mutex_init(&solution_available, NULL);
	pthread_mutex_lock(&solution_available);
	pthread_mutex_init(&mutex_solution_queue, NULL);
	pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
	pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));

	while (running) {
		if (_kbhit()) {
			switch (_getch()) {
			case 'q':
				running = false; break;
			case '0':
				pickingup = true; std::cout << "Pick Up Mode Activated." << std::endl; break;
			case '9':
				turnTransducersOff = true; break;
			/*case '1':
				startTesting = true;
				++totalTestTimes;
				printf("Current test %d\n", totalTestTimes);
				break;*/
			case '3':
				printf("Acceleration increased from %f to %f\n", a0, a0 + a_step); a0 += a_step; break;
			case '4':
				printf("Acceleration decreased from %f to %f\n", a0, a0 - a_step); a0 -= a_step; break;
			case 'f':
				++failedTimes;
				printf("Failed at test %d\n", failedTimes);
				printf("Current test %d\n", totalTestTimes);
				break;
			case 'p':
				startPlanningControl = true; break;
			case 'a':
				run4p_updown = true; break;   // To run the 4 particles all up_down animation
			default:
				break;
			}
		}
	}

	Sleep(2000);
	a->disconnect();
	delete a;
	delete solver;
}


void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff) {
	static float dynamicTargetUPS = targetUPS;
	static float timePerUpdateInMilis = (float)(numGeometries) * 1000.0f / dynamicTargetUPS; //this needs to change every time we change dynamicTargetUPS
	static int numSolutions = dynamicTargetUPS;
	static DWORD prevTime, curTime;
	static DWORD secPrevTime = microTimer::uGetTime(), secCurTime;
	static float timeSinceLastUpdate;

	static bool firstTime = true;
	if (firstTime) {
		prevTime = microTimer::uGetTime();
		firstTime = false;
	}
	//Compute discrete phases for this update:
	if (transducersOff) a->turnTransducersOff();
	else a->updateMessages(messages, numGeometries);

	//let's check the current time and wait until next update is due
	do {
		curTime = microTimer::uGetTime();
		timeSinceLastUpdate = (curTime - prevTime) / 1000.f;
	} while (timeSinceLastUpdate < timePerUpdateInMilis && timeSinceLastUpdate > 0);
	prevTime = curTime;
	//Send phases and update time

	//Plot performance (should be 1s)
	numSolutions -= numGeometries;
	if (numSolutions <= 0) {
		dynamicTargetUPS += (int)(0.1f * targetUPS);
		if (dynamicTargetUPS > targetUPS)
			dynamicTargetUPS = targetUPS;
		timePerUpdateInMilis = (float)(numGeometries) * 1000.0f / dynamicTargetUPS;
		numSolutions = dynamicTargetUPS;
		secCurTime = microTimer::uGetTime();
		printf("Time Per computation = %f; Last Update: %f, UPS: %f\n", (secCurTime - secPrevTime) / 1000000.f, timeSinceLastUpdate / (float)(numGeometries), dynamicTargetUPS);
		secPrevTime = secCurTime;
	}
}
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment, bool targetConstraint) {
	int numAgents = starts.size();
	std::vector<PathPlanner::AgentInfo> agents;
	for (int i = 0; i < numAgents; i++) {
		PathPlanner::AgentInfo agent(starts[i], targets[i], size, velocity);
		agents.push_back(agent);
	}
	planner.setAgents(agents);

	std::string fileName = "S2M2/Files/test.csv";
	std::string newFileName = "S2M2/Files/newTest.csv";
	std::string outFileName = "S2M2/Files/output.csv";
	planner.saveAgentInfo(fileName);

	bool success = true;
	if (taskAssignment) {
		success &= planner.taskAssignment(fileName, newFileName);
		success &= planner.runS2M2(newFileName, outFileName);
	}
	else {
		success &= planner.runS2M2(fileName, outFileName);
	}
	if (!success)
		return std::vector<AgentPath>(0);

	std::vector<AgentPath> paths = PathPlanner::readAgentPaths(outFileName);
	if (targetConstraint) {
		std::vector<AgentPath> constraintPaths;
		for (int j = 0; j < numAgents; j++) {
			for (int i = 0; i < numAgents; i++) {
				glm::vec3 finalPoint = paths[i].getPosition(paths[i].getFinalTime());
				if (glm::distance(finalPoint, targets[j]) < 0.001f) {
					constraintPaths.push_back(paths[i]);
				}
			}
		}
		return constraintPaths;
	}
	else
		return paths;
}
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, float*& posBuffer, float*& ampBuffer) {
	int sampleSize = numGeometries;
	pBufferSize = numPoints * sampleSize * 4;
	aBufferSize = numPoints * sampleSize * 1;
	posBuffer = new float[pBufferSize];
	ampBuffer = new float[aBufferSize];

	// Define actual content we create
	int posInd = 0, ampInd = 0, colInd = 0;
	for (int s = 0; s < sampleSize; s++) {
		for (int t = 0; t < numPoints; t++) {
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = 0;
			posBuffer[posInd++] = 1;
			ampBuffer[ampInd++] = 16000;
		}
	}
}
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints) {
	static const int minThres = 10, maxThres = 160, thresStep = 10;
	int threshold = minThres;
	bool corrected = false;

	std::vector<glm::vec3> detectedPositions(numPoints);
	while (!corrected) {
		detector.setThreshold(threshold);
		std::vector<cv::Point3d> currentBeads = detector.getCurrentBeads();
		if (currentBeads.size() == numPoints) {
			corrected = true;
			for (int p = 0; p < numPoints; p++)
				detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
		}
		if (threshold == maxThres) threshold = minThres;
		else threshold += thresStep;
	}
	return detectedPositions;
}
void print(const char* str) {
	printf("%s\n", str);
}
