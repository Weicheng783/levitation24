#include <AsierInho_V2.h>
#include <AsierInho.h>
#include <GSPAT_SolverNaive.h>
#include <GSPAT_SolverIBP.h>
#include <GSPAT_SolverV2.h>
#include <GSPAT_SolverTS.h>

#include <Helper/microTimer.h>
#include <S2M2/cpp/PathPlanner.h>
#include <clientPrograms/Applications/LeviPropRecognizer/BeadDetector.h>
#include <GenerateSpeedTestPaths.h>
//#include <OptiTrack/OptiTrack.h>

#include <random>
#include <deque>
#include <conio.h>
#include <thread>


// define meta information
// Setting Ground Plane: 0, Testing Target: 0 then 3.
// 9 Switch transducer off.
enum Algorithms { GS_PAT = 0, TEMPORAL, NAIVE, IBP };
Algorithms algorithmUsed = GS_PAT;
int numAgents = 1;
std::string numAgentss = "1";
//int roundNum = 4;
std::string roundNumm = "1";
std::string algorithmUsedd = "gs-pat";
std::string testPattern = "drawing";
std::string systemVersion = "240702";
float testing_velocity = 0.01f;					// maximum velocity in meter/second
std::string testing_velocityy = "0.01";
float settedStageHeight = 0.033f;
bool initializeCamera = true; // Whether to perform camera calibration

std::string fullPathPre = "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + numAgentss + "/" + testPattern + "/" + algorithmUsedd + "/" + testing_velocityy + "/" + roundNumm + "/";
std::string auxiliaryName = "";

std::string tranducerPhaseFile = fullPathPre + "sim_output_transducerPhase.csv";
std::ofstream PositionFile;   // to record the tracking position of the particles

// END OF META INFORMATION

int numPoints = numAgents;
bool verticalTest = true;
//float angleDegree = -30.f;
float a_step = (numPoints <= 8 ? 0.1f : 0.01f);
float a0 = 2; //Acceleration to test in m/s2
int numIterations = 5;
float transducerThreshold = M_PI / 32.f;  //M_PI / 32.f;   ; 
float pointThreshold = 0;      //0;   M_PI;  
float targetAmplitude = 1.0f;
unsigned char amplitudeDisc = (float)((64 * 2.0f / M_PI) * asin(targetAmplitude));


int failedTimes = 0;
int totalTestTimes = 0;

std::stringstream savedPathFile;
std::vector<AgentPath> existingPath;
std::stringstream existingPathFile;
std::string folderName = "2Agent";//"Recording_3D-8p-0.01v-10rounds";   //save the recordingPositions in this folder  //Recording10p-0.05v-10rounds
std::string updatedFolderName = "Recording_3D-8p-0.1v-10rounds";
std::string csvFileName = "RecordingLog_TEMPORAL_medium.csv";    // record if particles drop or not
int runningTimes = 1;   // define the index of every trail


AsierInho_V2::AsierInhoBoard_V2* a;		// this version (V2) of AsierInho has the function to send 32 geometries at the same time
float centreHeight = 0.12f;				// distance between the top and bottom boards
bool running = true;					// indicates the threads are still running
float targetUPS = 10000;				// target updates per second
const int numGeometries = 32;					// number of geometries to be sent at the same time
pthread_mutex_t mutex_solution_queue;
pthread_mutex_t solution_available;
std::deque<GSPAT::Solution*> solutions;
std::deque<bool> transducerOffSignals;
pthread_t readerThread, writerThread;

pthread_mutex_t mtx_running_from_file = PTHREAD_MUTEX_INITIALIZER;   // mutex for runningFromFiles (when we need phases by reading from external files)
pthread_cond_t controlVal_running_from_file = PTHREAD_COND_INITIALIZER;

bool simplyReadPhase = true;			// simply read the phase from the files
bool USE_AMPLITUDE = true;            // use amplitude from the file
bool runningFromFiles = false;

bool turnTransducersOff = true;						// transducers can be off before detecting the particle positions
bool pickingup = false;
bool pickingup_disturbance = false;
bool startTesting = false;
bool startPlanningControl = false;

bool goToStartPos = false;
bool goToTargetPos = false;

bool goToTestingPos = false;
bool testLowVelocity = false;
bool testMediumVelocity = false;
bool testHighVelocity = false;

bool goToTestingStart = false;
bool goToTestingEnd = false;

bool LogSuccess = false;
bool LogFailed = false;

bool revisingAmplitude = false;
bool clearRevisedAmplitude = false;
int startStep = 0;
int endStep = 9999;
int revisingAgentIndex_1 = 5;
int revisingAgentIndex_2 = 3;
int ampOffset = 0;


void updateGeometries_byReading();
void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false, bool targetConstraint = false);
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, float*& posBuffer, float*& ampBuffer);
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);
//std::vector<glm::vec3> getDetectedPositions(OptiTrack& optitrack, int numPoints);
std::vector<glm::vec3> getRandomPositions(int numPoints, float size = 0.0075f, float zSize = 0.01f, glm::vec3 boundaryMin = glm::vec3(-0.06, -0.06, -0.06 + centreHeight), glm::vec3 boundaryMax = glm::vec3(+0.06, +0.06, +0.06 + centreHeight));  // z pos + centreHeight
void readTransducerInfo(std::string fileName, int& numGeometries, int& numTransducers, float*& phases);
void discretizeInfo();

void print(const char* str);

void* solutionReaderThread(void* arg) {
	GSPAT::Solver* solver = ((GSPAT::Solver*)arg);

	while (running) {
		//0.0 Check if we are using the file to read the phases
		//pthread_mutex_lock(&mtx_running_from_file);
		//while (runningFromFiles) {
		//	//pthread_cond_wait(&controlVal_running_from_file, &mtx_running_from_file);  // wait until we are not running from files
		//}
		//pthread_mutex_unlock(&mtx_running_from_file);

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
			for (int g = 0; g < numGeometries; g++) { // only when we want to change the amplitudes of the transducers
				memset(&finalMessages[g * 512 + 256], amplitudeDisc, 256);
				memset(&finalMessages[(g + numGeometries) * 512 + 256], amplitudeDisc, 256);
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

	// 2. Initialize the bead detector
	// TODO: CHANGE THIS 0.0355
	float stageHeight = settedStageHeight; // the height of the stage (work great to set a little bit above the surface)   0.0355f; 0.031f
	float p1[] = { -0.084f, -0.084f,  stageHeight },
		p2[] = { 0.084f, -0.084f, stageHeight },
		p3[] = { 0.084f,  0.084f, stageHeight },
		p4[] = { -0.084f,  0.084f, stageHeight };
	BeadDetector& detector = BeadDetector::instance(p1, p2, p3, p4);
	if (!initializeCamera) detector.loadBackground();
	detector.startDetection(0, 1920, 1080);
	//detector.startDetection(0, 1280, 720);

	//OptiTrack& Optitrack = OptiTrack::instance();
	//if (Optitrack.InitOptiTrack() != 0) {
	//	printf("Could not connect to OptiTrack. Is it connected?\n");
	//	return 0;
	//}

	// 3. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight - 0.06f);
	glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight + 0.06f);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.007, 0.007, 0.015);	// size of the agents in meter
	float velocity = 0.01f;					// picking up maximum velocity in meter/second   0.05
	// define initial positions
	std::vector<AgentPath> paths(numPoints);
	std::vector<glm::vec3> initialPositions;
	std::vector<glm::vec3> initialTestingPositions;
	std::vector<glm::vec3> endTestingPositions;
	std::vector<glm::vec3> startPositions, targetPositions;

	float timeCoefficient = 1.0f;     // time coefficient to change the velocity for the same paths

	float r1 = 0.04f;    //circle radius
	float r2 = 0.02f;    //cube radius


	delete[] circlePath_data;

	//detected particles go to the initial positions first !

	//initialPositions.push_back(glm::vec3(0.00, -0.06, centreHeight+0.06));
	//initialPositions.push_back(glm::vec3(0.05, -0.06, centreHeight));
	//initialPositions.push_back(glm::vec3(0.05, 0.06, centreHeight));

	//initialPositions.push_back(glm::vec3(0.00, 0.00, centreHeight + 0.03));
	//initialPositions.push_back(glm::vec3(0.00, 0.00, centreHeight - 0.06));
	//initialPositions.push_back(glm::vec3(0.04, 0.00, centreHeight - 0.06));

	// Mapped coordinates for Optitrack Coordinate System (here, -x is +y, -z is +x).
	//initialPositions.push_back(glm::vec3(0.00, 0.00, centreHeight)); // origin
	//initialPositions.push_back(glm::vec3(0.00, -0.03, centreHeight)); // A, shorter leg
	//initialPositions.push_back(glm::vec3(-0.06, 0.00, centreHeight)); // B, longer leg
	////
	////// Only for initialPositions Testing
	//for (int i = 0; i < numPoints; i++) {
	//	//TODO: NEED TO BE CHANGED.
	//	//Multiple particle
	//	initialPositions.push_back(glm::vec3(r1 * cos(i * 2 * M_PI / (float)numPoints), r1 * sin(i * 2 * M_PI / (float)numPoints), centreHeight));
	//	//Single particle only
	//   //initialPositions.push_back(glm::vec3(r1 * cos(i * M_PI / 4.f), r1 * sin(i *  M_PI / 4.f), centreHeight));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}

	// TODO: CHANGE THIS TO BE YOUR PATH FILE.
	//Get the initial positions from existing paths
	existingPathFile << fullPathPre << "pathFile" + auxiliaryName + ".csv"; //<< "agent - " << numPoints << " - maxV - 0.01" << " - round - " << runningTimes << " - pathFile.csv";
	existingPath = planner.readAgentPaths(existingPathFile.str());
	initialPositions.clear();
	targetPositions.clear();
	for (int i = 0; i < existingPath.size(); i++)
	{
		initialPositions.push_back(existingPath[i].getPosition(0));
		targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
		paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
		std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
	}

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

	// These are for storing the data
	std::vector<float> times;
	std::vector<std::vector<glm::vec3>> trapPositions(numPoints); // where you created traps
	std::vector<std::vector<glm::vec3>> realPositions(numPoints); // where optiTrack detected

	std::stringstream PositionFile;

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	float currTime = 0;								// current (virtual) time 
	bool moving = false;							// indicates the particles are moving or not
	bool testMode = false;
	bool finished = false;
	bool startRecording = false;
	bool startTracking = false;
	bool going = true;

	float lastIntendedFinishTimePickUp = 0.0f;
	float pickUpRunningFlag = false;
	float afterPickUpExperiment = false;

	bool failDuringExperiment = false;
	bool failDuringInitial = false;

	std::ofstream f;

	int pIndex = 0;

	std::cout << "herererererer" << std::endl;

	while (running) {
		if (failDuringExperiment) {
			finished = true;
			if (!f.is_open()) {
				f.close();
			}

			printf("------ Failure Happened During Experiment, clear previous data and reset ------\n");
			//reset the position container
			times.clear();
			for (int i = 0; i < trapPositions.size(); i++)
			{
				trapPositions[i].clear();
				realPositions[i].clear();
			}

			startRecording = false;
			startTracking = false;

			turnTransducersOff = true;

			failDuringExperiment = false;
		}

		// 5.0. Check if we are running from files, if not, we run the normal procedure using solver
		if (pickUpRunningFlag) {
			if (currTime > lastIntendedFinishTimePickUp) {
				print("***** Pickup Initialization Finished *****\n");
				pickUpRunningFlag = false;
				afterPickUpExperiment = true;
			}
		}

		//if (afterPickUpExperiment) {
		//	// Wait 5 seconds to run the experiment automatically
		//	if (currTime > lastIntendedFinishTimePickUp + 5) {
		//		afterPickUpExperiment = false;
		//		goToTestingPos = true;
		//	}
		//}

		// 5.0. Check if we are running from files, if not, we run the normal procedure using solver
		//pthread_mutex_lock(&mtx_running_from_file);
		//while (runningFromFiles) {
		//	pthread_cond_wait(&controlVal_running_from_file, &mtx_running_from_file);
		//}

		// 5.1. Picking up from a bed
		if (pickingup) {
			if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			moving = true;
			currTime = 0;
			// detect the particle positions
			std::vector<glm::vec3> detectedPositions = getDetectedPositions(detector, numPoints);
			//std::vector<glm::vec3> detectedPositions = getDetectedPositions(Optitrack, numPoints);
			float aboveHeight = 0.062f; //0.062
			float stopDuration = 1.f;
			float initialiseVelocity = 0.02f;
			float initialTime = (aboveHeight - stageHeight) / initialiseVelocity + stopDuration;
			for (int i = 0; i < numPoints; i++)
				detectedPositions[i].z = aboveHeight;

			// compute the paths
			std::vector<AgentPath> tmpPaths = setPaths(planner, detectedPositions, initialPositions, size, initialiseVelocity, false, false);

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

				lastIntendedFinishTimePickUp = 0.0f;
				for (int p = 0; p < numPoints; p++) {
					float tempTime = paths[p].getFinalTime();
					if (tempTime > lastIntendedFinishTimePickUp) {
						lastIntendedFinishTimePickUp = tempTime;
					}
				}
				pickUpRunningFlag = false;

				turnTransducersOff = false; // turn the transducers on
				pickingup = false;
			}
			else {
				pickingup = false;
			}
		}
		// 5.2. test paths (random paths)
		if (goToTestingStart)    // 1
		{
			//Get the initial positions from existing paths
			existingPathFile.str("");
			existingPathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output" << "/" << folderName << "/" << "agent-" << numPoints << "-maxV-0.1" << "-round-" << runningTimes << "-pathFile.csv";
			existingPath = planner.readAgentPaths(existingPathFile.str());
			initialTestingPositions.clear();
			endTestingPositions.clear();
			for (int i = 0; i < existingPath.size(); i++)
			{
				initialTestingPositions.push_back(existingPath[i].getPosition(0));
				endTestingPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
				std::cout << "current path start at " << existingPath[i].getPosition(0).x << existingPath[i].getPosition(0).y << existingPath[i].getPosition(0).z << std::endl;
			}
			// compute the paths
			velocity = 0.01;
			paths = setPaths(planner, initialPositions, initialTestingPositions, size, velocity, false, true);
			currTime = 0;
			moving = true;
			goToTestingStart = false;
			//exisingPathFile.str("");
			revisingAmplitude = false;
		}
		if (goToTestingEnd)   // temporally update the tracking and recording signal here
		{
			currTime = 0;
			moving = true;
			paths = existingPath;
			//startTracking = true;
			//startRecording = true;
			goToTestingEnd = false;
			revisingAmplitude = true;
		}
		if (goToStartPos)  // 7
		{
			pickingup_disturbance = true;
			goToStartPos = false;
		}
		if (goToTargetPos)  //5 test custom transducer amplitude adjustments
		{
			//++runningTimes;
			//targetPositions = getRandomPositions(numPoints);
			//velocity = 0.01;
			//paths = setPaths(planner, startPositions, targetPositions, size, velocity, false, false);
			////initialPositions = targetPositions;
			//currTime = 0;
			moving = true;
			goToTargetPos = false;
			////save the path 
			//savedPathFile << "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/weicheng/" << "agent-" << numPoints << "-maxV-0.01" << "-round-" << runningTimes << "-pathFile.csv";
			//PathPlanner::saveAgentPaths(savedPathFile.str(), paths);
		}

		if (goToTestingPos)    // 3
		{
			// compute the paths
			//velocity = 0.01;
			//paths = setPaths(planner, initialPositions, targetPositions, size, testing_velocity, false, true);
			paths = existingPath;
			//savedPathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo/Output" << "/" << folderName << "/" << "agent-" << numPoints << "-maxV-" << velocity << "-round-" << runningTimes << "failed-pathFile.csv";
			//PathPlanner::saveAgentPaths(savedPathFile.str(), paths);
			currTime = 0;
			moving = true;
			startTracking = true;
			finished = false;
			goToTestingPos = false;
		}
		//run the existing path with velocity=0.01
		if (testLowVelocity)      //a
		{
			moving = true;
			currTime = 0;
			timeCoefficient = 1.0f;
			std::cout << "existing pathFile:" << existingPathFile.str() << std::endl;
			paths = planner.readAgentPathsRevise(existingPathFile.str(), timeCoefficient);
			existingPathFile.str("");
			testLowVelocity = false;
		}
		//run the existing path with velocity=0.05
		if (testMediumVelocity)      //s
		{
			moving = true;
			currTime = 0;
			timeCoefficient = 0.2f;
			std::cout << "existing pathFile:" << existingPathFile.str() << std::endl;
			paths = planner.readAgentPathsRevise(existingPathFile.str(), timeCoefficient);
			existingPathFile.str("");
			testMediumVelocity = false;
		}
		//run the existing path with velocity=0.1
		if (testHighVelocity)      //d
		{
			moving = true;
			currTime = 0;
			timeCoefficient = 0.1f;
			std::cout << "existing pathFile:" << existingPathFile.str() << std::endl;
			paths = planner.readAgentPathsRevise(existingPathFile.str(), timeCoefficient);
			//added later to save the revised paths
			existingPathFile.str("");
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo/Output" << "/" << updatedFolderName << "/" << "agent-" << numPoints << "-maxV-" << 0.1 << "-round-" << runningTimes << "-pathFile.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), paths);
			std::cout << "revised path saved" << std::endl;
			std::cout << savedPathFile.str() << std::endl;
			existingPathFile.str("");
			testHighVelocity = false;
		}

		// 5.3. Update matrices for picking up (also for the planning control)
		if (!testMode) {
			finished = false;   //finished = false

			float lastIntendedFinishTime = 0.0f;
			for (int p = 0; p < numPoints; p++) {
				float tempTime = paths[p].getFinalTime();
				if (tempTime > lastIntendedFinishTime) {
					lastIntendedFinishTime = tempTime;
				}
			}

			// do some change that make sure there are enough positions detected/added to the vector container
			//std::vector<glm::vec3> currentBeads = Optitrack.getDetectedPositions();

			for (int p = 0; p < numPoints; p++) {
				glm::vec3 pos = paths[p].getPosition(currTime); // get the positions of the agents according to the current time
				mStarts[16 * p + X_index] = mEnds[16 * p + X_index];
				mStarts[16 * p + Y_index] = mEnds[16 * p + Y_index];
				mStarts[16 * p + Z_index] = mEnds[16 * p + Z_index];
				mEnds[16 * p + X_index] = pos.x;
				mEnds[16 * p + Y_index] = pos.y;
				mEnds[16 * p + Z_index] = pos.z;
				finished &= (paths[p].getFinalTime() < currTime); // check if every agent has finished its path

				// record and store timing, targetTrapPos and actual capturedPos
				// add some time constraints to start recording !
				//if (!finished && moving && startTracking)
				//{
				//	//printf("Intended Finish time: %f, Current time: %f\n", lastIntendedFinishTime, currTime);

				//	if (p == 0)  		// add time once if currTime changed
				//	{
				//		times.push_back(currTime);
				//		printf("time steps: %d ", times.size());
				//	}

				//	trapPositions[p].push_back(glm::vec3(pos.x, pos.y, pos.z));  //{pos.x,pos.y,pos.z}

				//	// number checking!
				//	if (currentBeads.size() < numPoints)
				//	{
				//		printf("some beads are not detected\n");
				//		realPositions[p].push_back(glm::vec3(999.0f, 999.0f, 999.0f));
				//	}
				//	else
				//		//need to do mapping for post-process later
				//		realPositions[p].push_back(currentBeads[p]); //  the detected single position currentBeads[p] is the bead[p]'s real position (realPositions[p])            
				//	
				//	if (currTime > lastIntendedFinishTime) {
				//		finished = true;
				//		startRecording = true;
				//		//continue;
				//	}
				//}

			}

			// Amplitude Amendments
			if (currTime > startStep * dt && currTime < endStep * dt)
			{
				if (revisingAmplitude)
				{
					//std::cout << "Currently Revising Amplitudes..." << std::endl;
					for (int g = 0; g < numGeometries; g++) {
						for (int k = 0; k < numAgents; k++) {
							//ampBuffer[g * numPoints + k] = 1000 + ampOffset;   //20800 (30%)  19200 (20%)  12800(-20%)
							ampBuffer[g * numPoints + k] = 16000 + ampOffset;
							//ampBuffer[g * numPoints + 1] = 1000 + ampOffset;
						}
						//ampBuffer[g * numPoints + revisingAgentIndex_2] = 26000;
						//ampBuffer[g * numPoints + revisingAgentIndex_1] = 12800;   //20800 (30%)  19200 (20%)  12800(-20%)
						//ampBuffer[g * numPoints + revisingAgentIndex_2] = 19200;
					}
					//int startIndex1 = 2 * numGeometries;  // Starting index (inclusive)
					//int endIndex1 = 3 * numGeometries;    // Ending index (exclusive)
					//////print the pos buffer here
					//for (int i = startIndex1; i < endIndex1; i++)
					//{
					//	// Modify the element (assign a new value)
					//	ampBuffer[i] = 5000;  // newValue is the value you want to assign
					//	//std::cout << "ampBuffer[" << i << "] is " << ampBuffer[i] << std::endl;
					//}
				}

				if (clearRevisedAmplitude)
				{
					//std::cout << "Cleared Revised Amplitude..." << std::endl;
					for (int g = 0; g < numGeometries; g++) {
						ampBuffer[g * numPoints + revisingAgentIndex_1] = 16000;   //20800 (30%)  19200 (20%)  12800(-20%)
						//ampBuffer[g * numPoints + revisingAgentIndex_2] = 26000;
						//ampBuffer[g * numPoints + revisingAgentIndex_1] = 12800;   //20800 (30%)  19200 (20%)  12800(-20%)
						//ampBuffer[g * numPoints + revisingAgentIndex_2] = 19200;
					}
					//int startIndex1 = 2 * numGeometries;  // Starting index (inclusive)
					//int endIndex1 = 3 * numGeometries;    // Ending index (exclusive)
					//////print the pos buffer here
					//for (int i = startIndex1; i < endIndex1; i++)
					//{
					//	// Modify the element (assign a new value)
					//	ampBuffer[i] = 5000;  // newValue is the value you want to assign
					//	//std::cout << "ampBuffer[" << i << "] is " << ampBuffer[i] << std::endl;
					//}
				}

			}
			else
			{
				//for (int g = 0; g < numGeometries; g++) {

				//	ampBuffer[g * numPoints + revisingAgentIndex_1] = 16000;
				//	ampBuffer[g * numPoints + revisingAgentIndex_2] = 16000;

				//}

				//int startIndex1 = 2 * numGeometries;  // Starting index (inclusive)
				//int endIndex1 = 3 * numGeometries;    // Ending index (exclusive)
				//////print the pos buffer here
				//for (int i = startIndex1; i < endIndex1; i++)
				//{
				//	// Modify the element (assign a new value)
				//	ampBuffer[i] = 16000;  // newValue is the value you want to assign
				//	//std::cout << "ampBuffer[" << i << "] is " << ampBuffer[i] << std::endl;
				//}
			}


			if (moving) currTime += dt; // increment the current time

			//if (finished && startRecording)
			//{
			//	// Sleep for 1 second
			//	//std::this_thread::sleep_for(std::chrono::seconds(1));

			//	//turnTransducersOff = true;

			//	PositionFile.clear();

			//	printf("------finished status updating------\n");

			//	//use the ofstream. create file and write in and only write once!				
			//	PositionFile << fullPathPre + "trackingPositionFile.csv";

			//	// Check if the file exists and delete it
			//	if (std::ifstream(fullPathPre + "trackingPositionFile.csv")) {
			//		if (std::remove((fullPathPre + "trackingPositionFile.csv").c_str()) != 0) {
			//			std::cerr << "Error deleting file: " << fullPathPre + "trackingPositionFile.csv" << std::endl;
			//		}
			//		else {
			//			std::cout << "Deleted (Reset) file: " << fullPathPre + "trackingPositionFile.csv" << std::endl;
			//		}
			//	}

			//	f.open(fullPathPre + "trackingPositionFile.csv");
			//	if (!f.is_open()) {
			//		std::cerr << "Unable to open file: " << fullPathPre + "trackingPositionFile.csv" << "\n" << std::endl;
			//		//return;
			//	}
			//	f << "time,";
			//	//write csv header
			//	for (int i = 0; i < numPoints; i++)
			//	{
			//		if (i == (numPoints - 1))
			//		{
			//			f << "x" << i << ",y" << i << ",z" << i << "," << std::endl;
			//		}
			//		else {
			//			f << "x" << i << ",y" << i << ",z" << i << ",";
			//		}
			//	}

			//	// write positions (target trap positions and real(optiTrack) positions)
			//	// only write once, do not write previous data for current test
			//	for (int i = 0; i < times.size()-1; i++)
			//	{
			//		f << times[i] << ",";
			//		for (int n = 0; n < numPoints; n++)
			//		{
			//			if (realPositions[n][i].x > 990.0f) {
			//				f << "," << "," << ",";
			//			}
			//			else {
			//				f << realPositions[n][i].x << "," << realPositions[n][i].y << "," << realPositions[n][i].z << ",";
			//			}
			//			//f << trapPositions[n][i].x << "," << trapPositions[n][i].y << "," << trapPositions[n][i].z << ",";
			//			//printf("currTime: %f, currPos1: (%f,%f%f), currPos2: (%f, %f, %f)\n",times[i],trapPositions[0][i].x, trapPositions[0][i].y, trapPositions[0][i].z, trapPositions[1][i].x, trapPositions[1][i].y, trapPositions[1][i].z);
			//		}
			//		f << std::endl;
			//	}
			//	f.close();

			//	//printf("------Finished position writing, clear previous data------\n");

			//	printf("------Finished, clear previous data------\n");
			//	//reset the position container
			//	times.clear();
			//	for (int i = 0; i < trapPositions.size(); i++)
			//	{
			//		trapPositions[i].clear();
			//		//realPositions[i].clear();
			//	}
			//	realPositions.clear();

			//	startRecording = false;
			//	startTracking = false;
			//	continue;
			//}

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
				startTracking = false;
			}
		}
		// record the evaluation results
		if (LogSuccess)
		{
			// TODO: CHANGE THE OUTPUT LOG FILE LOCATION
			//write the recordingLog file here
			std::fstream log("C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/weicheng/3/" + csvFileName, std::fstream::in | std::fstream::out | std::fstream::app);
			if (!log) std::cout << "Unable to find file" << std::endl;
			log << "no particle dropped" << "," << timeCoefficient << "," << runningTimes << std::endl;
			log.close();
			LogSuccess = false;
			std::cout << "Logged the successful case" << std::endl;
			runningTimes += 1;
		}

		if (LogFailed)
		{
			std::fstream log("C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/weicheng/1/" + csvFileName, std::fstream::in | std::fstream::out | std::fstream::app);
			if (!log) std::cout << "Unable to find file" << std::endl;
			log << "dropped" << "," << timeCoefficient << "," << runningTimes << std::endl;
			log.close();
			LogFailed = false;
			std::cout << "Logged the dropping case" << std::endl;
			runningTimes += 1;
		}
	}

	delete[] posBuffer, ampBuffer;
	return NULL;
}

void main() {

	//only for import discretizing function from AsierInho (V1)
	AsierInho::AsierInhoBoard* disc = AsierInho::createAsierInho();
	disc->connect(AsierInho::BensDesign, 999, 1000);    //15,21   82, 80
	disc->disconnect();

	//prepare message format for reading phase from files
	unsigned char msg[1024];
	memset(msg, 0, 1024 * sizeof(unsigned char));
	msg[0] = 128; msg[25] = 128; msg[26] = 128; msg[34] = 128;
	msg[512 + 0] = 128; msg[512 + 25] = 128; msg[512 + 26] = 128; msg[512 + 34] = 128;

	float* readingPhaseBuffer, * readingAmplitudeBuffer;
	int numReadingGeometries, numTransducers;


	//STAGE 0: Initialize
	float transducerPositions[512 * 3], transducerNormals[512 * 3], amplitudeAdjust[512];
	int mappings[512], phaseDelays[512], numDiscreteLevels;
	AsierInho_V2::RegisterPrintFuncs(print, print, print);
	a = AsierInho_V2::createAsierInho();
	a->connect(999, 1000);  //3,23  21,15    2,4   82, 80    
	//a->connect(3, 23);

	for (int i = 0; i < 16; i++) a->updateMessage(msg);

	a->readParameters(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, &numDiscreteLevels);

	if (simplyReadPhase)
	{
		readTransducerInfo(tranducerPhaseFile, numReadingGeometries, numTransducers, readingPhaseBuffer);
		if (USE_AMPLITUDE) readTransducerInfo(tranducerPhaseFile, numReadingGeometries, numTransducers, readingAmplitudeBuffer);
	}


	unsigned char* messages_fromReading = new unsigned char[2 * numReadingGeometries * numTransducers];
	unsigned char phases_disc[512], amplitudes_disc[512];
	//discretize the phases and amplitudes
	printf("start discretization\n");
	for (int g = 0; g < numReadingGeometries; g++) {
		disc->discretizePhases(&(readingPhaseBuffer[g * numTransducers]), phases_disc);
		printf("discretize %d geometries \n", g + 1);
		if (USE_AMPLITUDE) {
			disc->discretizeAmplitudes(&(readingAmplitudeBuffer[g * numTransducers]), amplitudes_disc);
			disc->correctPhasesShift(phases_disc, amplitudes_disc);
		}
		else {
			// the amplitudes_disc array is filled with the value 64 for numTransducers * sizeof(unsigned char) bytes.
			memset(amplitudes_disc, 64, numTransducers * sizeof(unsigned char));
		}
		//copying the phases_disc and amplitudes_disc arrays into the messages array and modifying some specific elements:
		memcpy(&messages_fromReading[g * 2 * numTransducers + 0], &phases_disc[0], (numTransducers / 2) * sizeof(unsigned char));
		memcpy(&messages_fromReading[g * 2 * numTransducers + numTransducers / 2], &amplitudes_disc[0], (numTransducers / 2) * sizeof(unsigned char));
		messages_fromReading[g * 2 * numTransducers + 0] += 128;
		memcpy(&messages_fromReading[g * 2 * numTransducers + numTransducers], &phases_disc[numTransducers / 2], (numTransducers / 2) * sizeof(unsigned char));
		memcpy(&messages_fromReading[g * 2 * numTransducers + 3 * numTransducers / 2], &amplitudes_disc[numTransducers / 2], (numTransducers / 2) * sizeof(unsigned char));
		messages_fromReading[g * 2 * numTransducers + numTransducers] += 128;
	}
	printf("Got messages_fromReadingFiles after discretization\n");


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
	case IBP:
		GSPAT_IBP::RegisterPrintFuncs(print, print, print);
		solver = GSPAT_IBP::createSolver(512);
		break;
	}
	solver->setBoardConfig(transducerPositions, transducerNormals, mappings, phaseDelays, amplitudeAdjust, numDiscreteLevels);

	//STAGE 2: Create the reader and writer threads and the concurrency control variables:
	pthread_mutex_init(&solution_available, NULL);
	pthread_mutex_lock(&solution_available);
	pthread_mutex_init(&mutex_solution_queue, NULL);
	pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
	pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));

	//STAGE 3: Start the OptiTrack
	//OptiTrack& Optitrack = OptiTrack::instance();
	//if (Optitrack.InitOptiTrack() != 0) {
	//	printf("Could not connect to OptiTrack. Is it connected?\n");
	//	//return 0;
	//}

	while (running) {
		if (_kbhit()) {
			switch (_getch()) {
			case 'z':
				running = false; break;
				//case '8':
				//	turnTransducersOff = false; pickingup = true; break;
			case '0':
				pickingup = true; break;
			case '9':
				turnTransducersOff = true; break;
			case '1':
				printf("Current runningTimes %d\n", runningTimes);
				goToTestingStart = true;
				break;
				break;
				//case '4':
				//	runningFromFiles = true;
				//	//running = false;
				//	break;
				//case '5':
				//	//update by read info
				//	runningFromFiles = false;
				//	//updateGeometries_byReading();
				//	break;
			case '3':
				// shall I use this argument?
				//existingPathFile.str(""); //renew the string
				//++runningTimes;
				//printf("Current runningTimes %d\n", runningTimes);
				goToTestingPos = true;
				clearRevisedAmplitude = false;
				revisingAmplitude = true;
				break;
			case '4':
				// shall I use this argument?
				//existingPathFile.str(""); //renew the string
				//++runningTimes;
				//printf("Current runningTimes %d\n", runningTimes);
				goToTestingPos = true;
				revisingAmplitude = false;
				clearRevisedAmplitude = true;
				break;
			case '5':
				ampOffset += 1000;
				std::cout << "Amp Offset Added by 1000, now is " << ampOffset << ", total " << 16000 + ampOffset << std::endl;
				break;
			case '6':
				ampOffset -= 1000;
				std::cout << "Amp Offset Reduced by 1000, now is " << ampOffset << ", total " << 16000 + ampOffset << std::endl;
				break;
			case '7':
				ampOffset += 100;
				std::cout << "Amp Offset Added by 100, now is " << ampOffset << ", total " << 16000 + ampOffset << std::endl;
				break;
			case '8':
				ampOffset -= 100;
				std::cout << "Amp Offset Reduced by 100, now is " << ampOffset << ", total " << 16000 + ampOffset << std::endl;
				break;
				//case 'a':
				//	testLowVelocity = true; break;
				//case 's':
				//	testMediumVelocity = true; break;
				//case 'd':
				//	testHighVelocity = true; break;
				//case 'f':
				//	LogFailed = true;
				//	printf("Failed at test %d\n", runningTimes);
				//	break;
				//case 'b':
				//	LogSuccess = true; 
				//	printf("Succeeded at test %d\n", runningTimes);
				//	break;
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

void readTransducerInfo(std::string fileName, int& numGeometries, int& numTransducers, float*& phases) {
	std::ifstream ifs(fileName);
	if (ifs.is_open()) {
		std::string fileLine;
		bool firstLine = true;
		int currentGeometry = 0;
		bool reading = true;
		while (getline(ifs, fileLine) && reading) {
			std::stringstream toParse(fileLine);
			std::vector<std::string> lineTokens;
			std::string token;
			while (std::getline(toParse >> std::ws, token, ',')) {
				lineTokens.push_back(token);
				//printf("read tokens: %s ...\n", token.c_str());
			}
			if (!lineTokens.empty()) {
				if (firstLine) {
					numGeometries = atof(lineTokens[0].c_str());
					numTransducers = atof(lineTokens[1].c_str());
					phases = new float[numGeometries * numTransducers];
					firstLine = false;
				}
				else {
					for (int t = 0; t < numTransducers; t++) {
						float phase = atof(lineTokens[t].c_str());
						phases[currentGeometry * numTransducers + t] = phase;
					}
					currentGeometry++;
					printf("read %d of %d geometries so far...\n", currentGeometry, numGeometries);
				}
				if (currentGeometry == numGeometries)
					reading = false;
			}
		}
	}
	else printf("ERROR: File cannot be opened!");
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

void updateGeometries_byReading() {
	//// simply update the first geometry
	//for (int i = 0; i < 16; i++) {
	//	a->updateMessage(&messages_fromReading[0]);
	//}

	//printf("Press any key to move the particles\n");
	//_getch();
	//int updateRate = 100; // Hz
	//DWORD waitingPeriod = 1000000 / updateRate;
	//DWORD lastUpdate = microTimer::uGetTime();
	//DWORD currentTime = lastUpdate;
	//DWORD start = microTimer::uGetTime();
	//for (int g = 0; g < numGeometries; g++) {
	//	// wait a while 
	//	do {
	//		currentTime = microTimer::uGetTime();
	//	} while (currentTime - lastUpdate < waitingPeriod);
	//	// send messages to the boards
	//	a->updateMessage(&messages_fromReading[2 * g * numTransducers]);
	//	// get the current time
	//	lastUpdate = microTimer::uGetTime();
	//}
	//DWORD end = microTimer::uGetTime();
	//printf("Actual frame rate is %f Hz\n", numGeometries * 1000000.f / (end - start));

	//a->disconnect();
	//delete a;
	//delete[] readingPhaseBuffer, messages_fromReading;
	//if (USE_AMPLITUDE)
	//	delete[] readingAmplitudeBuffer;

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

//std::vector<glm::vec3> getDetectedPositions(OptiTrack& optitrack, int numPoints) {
//	std::vector<glm::vec3> detectedPositions(numPoints);
//
//	std::vector<glm::vec3> currentBeads = optitrack.getDetectedPositions();
//
//	for (int p = 0; p < numPoints; p++) {
//		detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
//		printf("Detected %d: pos = (%f, %f, %f)\n", p, detectedPositions[p].x, detectedPositions[p].y, detectedPositions[p].z);
//	}
//
//	return detectedPositions;
//}

std::vector<glm::vec3> getRandomPositions(int numPoints, float size, float zSize, glm::vec3 boundaryMin, glm::vec3 boundaryMax) {
	//static std::mt19937 mt;// (time(0)); // you can use the time as a seed 
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_real_distribution<float> get_rand_x(boundaryMin.x, boundaryMax.x);
	std::uniform_real_distribution<float> get_rand_y(boundaryMin.y, boundaryMax.y);
	std::uniform_real_distribution<float> get_rand_z(boundaryMin.z, boundaryMax.z);

	static const float minDistance = zSize * 2.f;
	static const float minXYDistance = size * 2.f;

	std::vector<glm::vec3> positions;
	while (positions.size() < numPoints) {
		/*float x = get_rand_x(mt);
		float y = get_rand_y(mt);
		float z = get_rand_z(mt);*/
		float x = get_rand_x(eng);
		float y = get_rand_y(eng);
		float z = get_rand_z(eng);
		glm::vec3 newPosition(x, y, z);
		// check if the new particle position is not too close to the others
		bool tooClose = false;
		for (int p = 0; p < positions.size(); p++) {
			float dist = glm::distance(newPosition, positions[p]);
			float distXY = glm::distance(glm::vec2(newPosition), glm::vec2(positions[p]));
			if (dist < minDistance || distXY < minXYDistance) {
				tooClose = true; break;
			}
		}
		if (!tooClose) positions.push_back(newPosition);
	}
	return positions;
}

void print(const char* str) {
	printf("%s\n", str);
}
