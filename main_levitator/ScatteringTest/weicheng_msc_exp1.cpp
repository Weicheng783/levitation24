//The Original Copy of Code File for EXPERIMENT 1: P4/6 random/crossing initialization test

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
#include <OptiTrack/OptiTrack.h>

#include <random>
#include <deque>
#include <conio.h>
#include <thread>
#include <filesystem>
#include <cstdlib>  // for std::system
#include <windows.h>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <future>

namespace fs = std::filesystem;

struct Waypoint {
	double time;
	double x, y, z;

	// Constructor for Waypoint
	Waypoint(double t, double xCoord, double yCoord, double zCoord)
		: time(t), x(xCoord), y(yCoord), z(zCoord) {}
};

// Define the Agent struct
struct Agent {
	int id;  // Agent ID
	std::vector<Waypoint> waypoints;  // A vector of waypoints

	// Constructor for Agent
	Agent(int agentId, const std::vector<Waypoint>& wps)
		: id(agentId), waypoints(wps) {}
};

struct WaypointAlt {
	std::string pointLabel;
	double time;
	double x, y, z;
};

struct AgentAlt {
	int id;
	std::vector<std::vector<std::string>> data; // to store raw data for easy processing
	std::vector<Waypoint> waypoints;
};

void generateWaypoints(Agent& agent, double speed, double targetZ) {
	double initialZ = agent.waypoints.front().z;
	double distance = targetZ - initialZ;
	double time = 0.0;
	double deltaTime = 1; // Adjust this value to control waypoint granularity

	agent.waypoints.push_back({ time + deltaTime, agent.waypoints.back().x, agent.waypoints.back().y, targetZ });

	// Depending on the direction, use a different stopping condition
	//while ((distance > 0 && agent.waypoints.back().z < targetZ) ||
	//	(distance < 0 && agent.waypoints.back().z > targetZ)) {
	//	time += deltaTime;
	//	double newZ = agent.waypoints.back().z + speed * deltaTime;

	//	// Ensure we don't overshoot the targetZ
	//	if ((distance > 0 && newZ >= targetZ) || (distance < 0 && newZ <= targetZ)) {
	//		newZ = targetZ;  // Correct to the exact targetZ
	//		agent.waypoints.push_back({ time, agent.waypoints.back().x, agent.waypoints.back().y, newZ });
	//		break; // Stop loop since we've reached the targetZ
	//	}

	//	agent.waypoints.push_back({ time, agent.waypoints.back().x, agent.waypoints.back().y, newZ });
	//}
}


//void generateWaypoints(Agent& agent, double speed, double targetZ) {
//	double initialZ = agent.waypoints.front().z;
//	double distance = targetZ - initialZ;
//	double time = 0.0;
//	double deltaTime = 0.1; // Adjust this value to control waypoint granularity
//
//	while (distance > 0 ? agent.waypoints.back().z < targetZ : agent.waypoints.back().z > targetZ) {
//		time += deltaTime;
//		double newZ = agent.waypoints.back().z + speed * deltaTime;
//		if ((distance > 0 && newZ > targetZ) || (distance < 0 && newZ < targetZ)) {
//			newZ = targetZ;  // Ensure we don't overshoot
//		}
//		agent.waypoints.push_back({ time, agent.waypoints.back().x, agent.waypoints.back().y, newZ });
//	}
//}

//void generateWaypoints(Agent& agent, double speed, double targetZ) {
//	double initialZ = agent.waypoints.front().z;
//	double distance = targetZ - initialZ;
//
//	// Calculate the total time needed to reach the target Z
//	double totalTime = std::abs(distance) / speed;
//
//	// Calculate the time interval between waypoints
//	double deltaTime = totalTime / 9.0; // 9 intervals for 10 waypoints
//
//	// Generate 10 waypoints
//	for (int i = 0; i < 10; ++i) {
//		double time = i * deltaTime;
//		double newZ = initialZ + distance * (time / totalTime);
//		// Ensure we don't overshoot
//		if ((distance > 0 && newZ > targetZ) || (distance < 0 && newZ < targetZ)) {
//			newZ = targetZ;
//		}
//		agent.waypoints.push_back({ time, agent.waypoints.front().x, agent.waypoints.front().y, newZ });
//	}
//}


std::vector<std::vector<std::string>> readCSVCustom(const std::string& filePath) {
	std::ifstream file(filePath);
	std::string line;
	std::vector<std::vector<std::string>> data;

	while (std::getline(file, line)) {
		std::vector<std::string> row;
		std::stringstream ss(line);
		std::string cell;

		while (std::getline(ss, cell, ',')) {
			row.push_back(cell);
		}
		data.push_back(row);
	}
	return data;
}

void mergeCSVFiles(const std::vector<std::string>& filePaths, const std::string& outputFilePath) {
	std::vector<std::vector<std::vector<std::string>>> mergedData;

	for (const auto& filePath : filePaths) {
		auto data = readCSVCustom(filePath);

		std::vector<int> agentIndices;
		for (size_t i = 0; i < data.size(); ++i) {
			if (data[i][0].find("Agent ID") != std::string::npos) {
				agentIndices.push_back(i);
			}
		}

		for (size_t idx = 0; idx < agentIndices.size(); ++idx) {
			int start = agentIndices[idx];
			int end = (idx + 1 < agentIndices.size()) ? agentIndices[idx + 1] : data.size();
			auto agentData = std::vector<std::vector<std::string>>(data.begin() + start, data.begin() + end);

			if (mergedData.size() <= idx) {
				mergedData.push_back(agentData);
			}
			else {
				auto& prevAgentData = mergedData[idx];
				auto prevLastPoint = prevAgentData.back();
				double timeOffset = std::stod(prevLastPoint[1]);

				int waypointStartIdx = 2;
				std::vector<std::vector<std::string>> newPoints;

				for (size_t i = waypointStartIdx; i < agentData.size(); ++i) {
					auto row = agentData[i];
					if (row[0].find("Point-") != std::string::npos) {
						int newPointNumber = prevAgentData.size() - waypointStartIdx + (i - waypointStartIdx);
						row[0] = "Point-" + std::to_string(newPointNumber);
						row[1] = std::to_string(std::stod(row[1]) + timeOffset);
						newPoints.push_back(row);
					}
				}

				int newWaypointsCount = prevAgentData.size() - waypointStartIdx + newPoints.size();
				prevAgentData[1][1] = std::to_string(newWaypointsCount);
				prevAgentData.insert(prevAgentData.end(), newPoints.begin(), newPoints.end());
			}
		}
	}

	std::ofstream outFile(outputFilePath);
	if (!outFile) {
		std::cerr << "Unable to open file: " << outputFilePath << std::endl;
		return;
	}

	for (const auto& agentData : mergedData) {
		for (const auto& row : agentData) {
			for (size_t i = 0; i < row.size(); ++i) {
				outFile << row[i];
				if (i < row.size() - 1) {
					outFile << ",";
				}
			}
			outFile << "\n";
		}
	}

	outFile.close();
}


void writeOutputToFile(const std::string& filePath, const std::vector<Agent>& agents) {
	std::ofstream outFile(filePath);
	if (!outFile) {
		std::cerr << "Unable to open file: " << filePath << std::endl;
		return;
	}

	for (const auto& agent : agents) {
		outFile << "Agent ID," << agent.id << std::endl;
		outFile << "Number of waypoints," << agent.waypoints.size() << std::endl;

		for (size_t i = 0; i < agent.waypoints.size(); ++i) {
			outFile << "Point-" << i << ","
				<< agent.waypoints[i].time << ","
				<< std::fixed << std::setprecision(6)
				<< agent.waypoints[i].x << ","
				<< agent.waypoints[i].y << ","
				<< agent.waypoints[i].z << std::endl;
		}
	}

	outFile.close();
}

void create_directories_if_not_exist(const std::string& path) {
	fs::path dir(path);

	if (!fs::exists(dir)) {
		if (fs::create_directories(dir)) {
			std::cout << "Directories created: " << path << std::endl;
		}
		else {
			std::cerr << "Failed to create directories: " << path << std::endl;
		}
	}
	else {
		std::cout << "Directory already exists: " << path << std::endl;
	}
}

bool exists_test(const std::string& name) {
	std::ifstream f(name.c_str());
	return f.good();
}

bool FileExists(const std::string& path) {
	DWORD fileAttr = GetFileAttributes(path.c_str());
	return (fileAttr != INVALID_FILE_ATTRIBUTES && !(fileAttr & FILE_ATTRIBUTE_DIRECTORY));
}

bool MoveFileFromAToB(const std::string& source, const std::string& destination) {
	if (FileExists(source)) {
		if (MoveFile(source.c_str(), destination.c_str())) {
			return true; // File moved successfully
		}
		else {
			std::cerr << "Failed to move the file. Error code: " << GetLastError() << std::endl;
			return false; // File move failed
		}
	}
	else {
		std::cerr << "File does not exist at the source location." << std::endl;
		return false; // File does not exist, so do not move
	}
}

bool CopyFileFromAToB(const std::string& source, const std::string& destination) {
	if (CopyFile(source.c_str(), destination.c_str(), FALSE)) {
		return true; // File copied successfully
	}
	else {
		std::cerr << "Failed to copy the file. Error code: " << GetLastError() << std::endl;
		return false; // File copy failed
	}
}

bool isFileEmpty(std::ifstream& file) {
	return file.peek() == std::ifstream::traits_type::eof();
}

void deleteFile(const std::string& filename) {
	if (std::remove(filename.c_str()) == 0) {
		std::cout << "File deleted successfully." << std::endl;
	}
	else {
		std::cerr << "Error deleting the file." << std::endl;
	}
}

// define meta information
// Setting Ground Plane: 0, Testing Target: 0 then 3.
// 9 Switch transducer off.
enum Algorithms { GS_PAT = 0, TEMPORAL, NAIVE, IBP };
Algorithms algorithmUsed = GS_PAT;
int numAgents = 6;
std::string numAgentss = "x";
int roundNum = 1;
std::string roundNumm = "1";
std::string algorithmUsedd = "gs-pat";
std::string testPattern = "random";
std::string systemVersion = "240702";
float testing_velocity = 1.00f;					// maximum velocity in meter/second
std::string testing_velocityy = "1.00";
float settedStageHeight = 0.0335f; // 0.033f
bool initializeCamera = true; // Whether to perform camera calibration (this reads from a file)
std::string nameIntegration = "";
bool manualMode = false;
bool manualInputMode = true;
bool manualDataCollectionMode = true;
bool manualSpeedReduction = false;
bool manualSpeedIncrease = false;
bool noCaptureMode = false;
bool noOptitrackMode = false;
bool path1plannerFailed = false;

int totalRecordings = 1;
float upper_limit = 0.10f; // m/s
float lower_limit = 0.01f;
//float current_speedo = lower_limit + (upper_limit - lower_limit) / 2;
float current_speedo = lower_limit;
float stepping = 0.02f;
int doing_times = 5; // each run 5 times at least
int current_counter = 0;
std::vector<float> particles_speed;
std::vector<float> particles_pickup_speed;
std::string filenamea = "";
std::ofstream outFilea;
std::vector<glm::vec3> detectedPositions;

std::string fullPathPre = "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(current_speedo) + "/" + roundNumm + "/";
std::string auxiliaryName = "";
std::string feedback_txt = "C:/Users/weicheng/Desktop/ServerSampler/bin/feedback.txt";
std::string amp_txt = "C:/Users/weicheng/Desktop/ServerSampler/bin/curr_amp.txt";
std::string velo_txt = "C:/Users/weicheng/Desktop/ServerSampler/bin/velo_amp.txt";

std::string tranducerPhaseFile = fullPathPre + "sim_output_transducerPhase.csv";
std::ofstream PositionFile;   // to record the tracking position of the particles

std::vector<glm::vec3> initialPositions, targetPositions;
std::vector<glm::vec3> initialPositions1, targetPositions1;

// END OF META INFORMATION

int numPoints = numAgents;
int numPointsSec = numAgents;
bool verticalTest = true;
//float angleDegree = -30.f;
//float a_step = (numPoints <= 8 ? 0.1f : 0.01f);
//float a0 = 2; //Acceleration to test in m/s2
int numIterations = 5;
float transducerThreshold = M_PI / 32.f;  //M_PI / 32.f;   ; 
float pointThreshold = 0;      //0;   M_PI;  
float targetAmplitude = 1.0f;
unsigned char amplitudeDisc = (float)((64 * 2.0f / M_PI) * asin(targetAmplitude));

int failedTimes = 0;
int totalTestTimes = 0;

std::vector<bool> revisingAmplitude;
std::vector<bool> clearRevisedAmplitude;
std::vector<int> startSteps;
std::vector<int> endSteps;
std::vector<int> revisingAgentIndices;
std::vector<int> ampOffsets;
std::vector<int> previousUsableSolutionAmpOffsets;
std::vector<float> previousUsableSolutionVelocities;


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

bool speed_reduce_calc = false;
bool speed_increase_calc = false;
bool simplyReadPhase = false;			// simply read the phase from the files
bool USE_AMPLITUDE = false;            // use amplitude from the file
bool runningFromFiles = false;

bool turnTransducersOff = true;						// transducers can be off before detecting the particle positions
bool pickingup = false;
bool pickingup_2d = false;
bool pickingup_disturbance = false;
bool startTesting = false;
bool startPlanningControl = false;
bool turbulance_generator = false;

bool goToStartPos = false;
bool goToTargetPos = false;

bool goToTestingPos = false;
bool goToTestingPos_2d = false;
bool testLowVelocity = false;
bool testMediumVelocity = false;
bool testHighVelocity = false;

bool goToTestingStart = false;
bool goToTestingEnd = false;
bool binarySearchLowerUpper = false;
bool binarySearchHigherLower = false;
bool writingToFile = false;

bool LogSuccess = false;
bool LogFailed = false;

bool firstTimeToLog = true;
bool bnAnalysis = false;
bool bnAnalysisFullMode = true;
bool lstmAnalysis = false;
bool simpleAnalysis = false;
bool simpleAnalysisFullMode = true;
bool mlpAnalysis = false;

bool guiManageMode = false;
bool developmentMode = false; // This disables/bypasses all functionality of the levitator for testing GUI

bool usePath1 = true; // Switch between path planners (true use S2M2, false use CBS).

std::string currentTime = "";
std::string integratedDir = "";

std::ofstream logFile;

void updateGeometries_byReading();
void updateGeometries(unsigned char* messages, int numGeometries, bool transducersOff = false);
std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, std::vector<float> velocities, bool taskAssignment = false, bool targetConstraint = false);
void fillBuffers(int numPoints, int& pBufferSize, int& aBufferSize, float*& posBuffer, float*& ampBuffer);
std::vector<glm::vec3> getDetectedPositions(BeadDetector& detector, int numPoints);
std::vector<glm::vec3> getDetectedPositions(OptiTrack& optitrack, int numPoints);
std::vector<glm::vec3> getRandomPositions(int numPoints, float size = 0.0075f, float zSize = 0.01f, glm::vec3 boundaryMin = glm::vec3(-0.06, -0.06, -0.06 + centreHeight), glm::vec3 boundaryMax = glm::vec3(+0.06, +0.06, +0.06 + centreHeight));  // z pos + centreHeight
void readTransducerInfo(std::string fileName, int& numGeometries, int& numTransducers, float*& phases);
void discretizeInfo();
void getStartEndPos_Crossing_four();
void getStartEndPos_StableLev_62();
void getStartEndPos_Crossing_six();
void getStartEndPos_Crossing_eight();
void getStartEndPos_Pushing_inside();
void getStartEndPos_StableLev_62_plain_s1();
void KillProcessByName(const std::string& exeName);
void StartProcess(const std::string& exePath);
std::string getCurrentTimeString();
bool FileExists(const std::string& path);
bool MoveFileFromAToB(const std::string& source, const std::string& destination);
bool CopyFileFromAToB(const std::string& source, const std::string& destination);
void checkAndReset();
void getStartEndPos_Random(int num_Agents);
bool exists_test(const std::string& name);
bool isFileEmpty(std::ifstream& file);
void deleteFile(const std::string& filename);
void initializeCameraSetting();
void writeSystemInfoFile();
void* performSystemInfoWriting(void* arg);
void generateWaypoints(Agent& agent, double speed, double targetZ);
void writeOutputToFile(const std::string& filePath, const std::vector<Agent>& agents);
std::vector<std::vector<std::string>> readCSVCustom(const std::string& filePath);
void mergeCSVFiles(const std::vector<std::string>& filePaths, const std::string& outputFilePath);

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
			if (!developmentMode) {
				curSolution->finalMessages(&finalMessages);
				for (int g = 0; g < numGeometries; g++) { // only when we want to change the amplitudes of the transducers
					memset(&finalMessages[g * 512 + 256], amplitudeDisc, 256);
					memset(&finalMessages[(g + numGeometries) * 512 + 256], amplitudeDisc, 256);
				}
				updateGeometries(finalMessages, numGeometries, transducerOffSignal);
				solver->releaseSolution(curSolution);
			}
		}
	}
	return NULL;
}

void initializeCameraSetting() {
	std::ifstream file("C:/Users/weicheng/Desktop/ServerSampler/bin/cameraInitialize.txt");

	if (file.is_open()) {
		std::string firstLine;
		if (std::getline(file, firstLine)) {
			if (firstLine == "true") {
				initializeCamera = true;
			}
			else if (firstLine == "false") {
				initializeCamera = false;
			}
			else {
				initializeCamera = false; // Invalid content, set to false
			}
		}
		else {
			initializeCamera = false; // No content in the file, set to false
		}
		file.close();
	}
	else {
		initializeCamera = false; // File doesn't exist, set to false
	}
}

void writeSystemInfoFile()
{
	filenamea = "C:/Users/weicheng/Desktop/ServerSampler/bin/system_info.txt";
	outFilea.open(filenamea);
	if (outFilea.is_open()) {
		char buffer[16];
		std::time_t now = std::time(nullptr);
		std::strftime(buffer, sizeof(buffer), "%y%m%d_%H%M%S", std::localtime(&now));
		outFilea << buffer << std::endl;
		outFilea << "guiManageMode:" << guiManageMode << std::endl;
		outFilea << "noOptitrackMode:" << noOptitrackMode << std::endl;
		outFilea << "noCaptureMode:" << noCaptureMode << std::endl;
		outFilea << "manualDataCollectionMode:" << manualDataCollectionMode << std::endl;
		outFilea << "manualInputMode:" << manualInputMode << std::endl;
		outFilea << "manualMode:" << manualMode << std::endl;
		outFilea << "current_velocity:" << current_speedo << std::endl;
		outFilea << "current_amplitude:" << 16000 + ampOffsets[0] << std::endl;
		outFilea << "numAgents:" << numAgents << std::endl;
		outFilea << "testPattern:" << testPattern << std::endl;
		outFilea << "round:" << roundNumm << std::endl;
		outFilea << "integratedDir:" << integratedDir << std::endl;
		outFilea << "fullPathPre:" << fullPathPre << std::endl;
		outFilea << "systemVersion:" << systemVersion << std::endl;
		outFilea << "solver_algorithm:" << algorithmUsedd << std::endl;
		outFilea << "developmentMode:" << developmentMode << std::endl;
		outFilea << "totalRecordings:" << totalRecordings << std::endl;
		outFilea << "lower_bound:" << lower_limit << std::endl;
		outFilea << "upper_bound:" << upper_limit << std::endl;
		outFilea.close();
		//std::cout << "System state file written successfully." << std::endl;
		std::ofstream(feedback_txt) << "0";
	}
	else {
		std::ofstream(feedback_txt) << "2";
		std::cerr << "Error: Could not open the file for writing." << std::endl;
	}
	outFilea.close();
}

void getStartEndPos_Crossing_four()
{
	initialPositions.clear();
	targetPositions.clear();

	// 4 particles crossing each other from a height
	initialPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
	initialPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
	initialPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
	initialPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));

	targetPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
}

void getStartEndPos_Random(int num_Agents)
{
	initialPositions.clear();
	targetPositions.clear();

	auto a = getRandomPositions(num_Agents);
	// n particles crossing each other from a height
	for (int i = 0; i < num_Agents; i++) {
		initialPositions.push_back(a[i]);
	}

	a = getRandomPositions(num_Agents);
	for (int i = 0; i < num_Agents; i++) {
		targetPositions.push_back(a[i]);
	}
}

void getStartEndPos_Crossing_six()
{
	initialPositions.clear();
	targetPositions.clear();

	// 6 particles crossing each other from a height
	initialPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
	initialPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
	initialPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
	initialPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D
	initialPositions.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // E
	initialPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // F

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));

	targetPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // E
	targetPositions.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // F
}

void getStartEndPos_Pushing_inside()
{
	initialPositions1.clear();
	targetPositions1.clear();

	if (numAgents == 4) {
		initialPositions1.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
		initialPositions1.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
		initialPositions1.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
		initialPositions1.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D

		targetPositions1.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(-0.04f, -0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
	}
	else {
		// 8 particles crossing each other from a height
		initialPositions1.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
		initialPositions1.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
		initialPositions1.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
		initialPositions1.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D
		initialPositions1.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // E
		initialPositions1.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // F
		initialPositions1.push_back(glm::vec3(0.00f, 0.04f, 0.12f)); // G
		initialPositions1.push_back(glm::vec3(0.00f, -0.04f, 0.12f)); // H

		targetPositions1.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(-0.04f, -0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
		targetPositions1.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // E
		targetPositions1.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // F
		targetPositions1.push_back(glm::vec3(0.00f, -0.04f, 0.12f)); // G
		targetPositions1.push_back(glm::vec3(0.00f, 0.04f, 0.12f)); // H
	}

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));


}

void getStartEndPos_Crossing_eight()
{
	initialPositions.clear();
	targetPositions.clear();

	// 8 particles crossing each other from a height
	initialPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
	initialPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
	initialPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
	initialPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D
	initialPositions.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // E
	initialPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // F
	initialPositions.push_back(glm::vec3(0.00f, 0.04f, 0.12f)); // G
	initialPositions.push_back(glm::vec3(0.00f, -0.04f, 0.12f)); // H

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));

	targetPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // E
	targetPositions.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // F
	targetPositions.push_back(glm::vec3(0.00f, -0.04f, 0.12f)); // G
	targetPositions.push_back(glm::vec3(0.00f, 0.04f, 0.12f)); // H
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

	initializeCameraSetting();

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

	if (!noOptitrackMode) {
		OptiTrack& Optitrack = OptiTrack::instance();
		if (Optitrack.InitOptiTrack() != 0) {
			printf("Could not connect to OptiTrack. Is it connected?\n");
			return 0;
		}
	}

	// 3. Set up our path planner
	PathPlanner planner("PathPlanner");
	glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight - 0.06f);
	glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight + 0.06f);
	planner.setBoundary(boundaryMin, boundaryMax);
	glm::vec3 size(0.009, 0.009, 0.9);	// size of the agents in meter
	//glm::vec3 size(0.007, 0.007, 0.015);
	float velocity = 0.01f;					// picking up maximum velocity in meter/second   0.05
	// define initial positions
	std::vector<AgentPath> paths(numPoints);
	std::vector<glm::vec3> startPositions;
	std::vector<glm::vec3> initialTestingPositions;
	std::vector<glm::vec3> endTestingPositions;

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
	//initialPositions.push_back(glm::vec3(0.00, 0.00, centreHeight - 0.06)); // origin
	//initialPositions.push_back(glm::vec3(0.00, -0.03, centreHeight - 0.06)); // A, shorter leg
	//initialPositions.push_back(glm::vec3(-0.06, 0.00, centreHeight - 0.06)); // B, longer leg
	//
	//// Only for initialPositions Testing
	//for (int i = 0; i < numPoints; i++) {
	//	 //TODO: NEED TO BE CHANGED.
	//	 //Multiple particle
	//	initialPositions.push_back(glm::vec3(r1 * cos(i * 2* M_PI / (float)numPoints), r1 * sin(i * 2* M_PI / (float)numPoints), centreHeight));
	//	 //Single particle only
	//	//initialPositions.push_back(glm::vec3(r1 * cos(i * M_PI / 4.f), r1 * sin(i *  M_PI / 4.f), centreHeight));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//}

	// TODO: CHANGE THIS TO BE YOUR PATH FILE.
	//Get the initial positions from existing paths
	//existingPathFile << fullPathPre << "pathFile" + auxiliaryName + ".csv"; //<< "agent - " << numPoints << " - maxV - 0.01" << " - round - " << runningTimes << " - pathFile.csv";
	//existingPath = planner.readAgentPaths(existingPathFile.str());
	//initialPositions.clear();
	//targetPositions.clear();
	//for (int i = 0; i < existingPath.size(); i++)
	//{
	//	initialPositions.push_back(existingPath[i].getPosition(0));
	//	targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
	//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
	//	std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
	//}

	if (particles_speed.size() == 0) {
		for (int i = 0; i < numAgents; i++) {
			particles_speed.push_back(current_speedo);
		}
	}

	// Instead, we use our test positions
	//getStartEndPos_Crossing_four();
	//getStartEndPos_Pushing_inside();
	//getStartEndPos_StableLev_62();
	//getStartEndPos_Crossing_six();
	//getStartEndPos_Crossing_eight();
	//getStartEndPos_StableLev_62_plain_s1();

	if (testPattern == "random" || !(testPattern == "crossing" || testPattern == "stablelev" || testPattern == "stacking")) {
		std::string filename = fullPathPre + "pathFile_exp.csv";
		std::cout << "****** FILE EXISTENCE: " << filename << " " << FileExists(filename) << " ******" << std::endl;
		if (FileExists(filename)) {
			std::cout << "Path file exists, do not overwrite and use this file." << std::endl;
			//Get the initial positions from existing paths
			existingPathFile << fullPathPre << "pathFile_exp.csv";
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
		}
		else {
			while (existingPath.empty()) {
				std::cout << "Path is still empty (caused by no plan or infeasible), regenerating random points, please wait patiently..." << std::endl;

				getStartEndPos_Random(numAgents);
				std::cout << "Path file does not exist, generate new path now..." << std::endl;
				existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
				initialPositions.clear();
				targetPositions.clear();
				for (int i = 0; i < existingPath.size(); i++)
				{
					initialPositions.push_back(existingPath[i].getPosition(0));
					targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
					paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
					std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
				}
				create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				std::cout << "Current Paths Saved to exp csv." << std::endl;
			}
		}
	}
	else if (testPattern == "crossing" && numAgents == 4) {
		getStartEndPos_Crossing_four();
	}
	else if (testPattern == "crossing" && numAgents == 6) {
		getStartEndPos_Crossing_six();
	}
	else if (testPattern == "crossing" && numAgents == 8) {
		getStartEndPos_Crossing_eight();
	}
	else if (testPattern == "stablelev" && numAgents == 6) {
		getStartEndPos_StableLev_62();
	}

	if (testPattern != "random") {
		existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
		initialPositions.clear();
		targetPositions.clear();
		for (int i = 0; i < existingPath.size(); i++)
		{
			initialPositions.push_back(existingPath[i].getPosition(0));
			targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
			paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
			std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
		}
		create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
		savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
		PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
		std::cout << "Current Paths Saved to exp csv." << std::endl;
	}

	std::cout << "Now we are finding: the maximum speed the system is able to handle." << std::endl;

	std::cout << "Current Speeds at: \n";
	for (int i = 0; i < numAgents; i++) {
		std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
	}
	std::cout << std::endl;

	//std::filesystem::path filePath("example.txt");

	//if (std::filesystem::exists(filePath)) {
	//	std::cout << "File exists.\n";
	//}
	//else {
	//	std::cout << "File does not exist.\n";
	//}

	const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
	std::string argument1 = std::to_string(numAgents);  // Replace with actual arguments
	std::string argument2 = roundNumm;
	std::string argument3 = testPattern;
	std::string argument4 = std::to_string(current_speedo);
	std::string argument5 = "full";

	// Construct the full command
	std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

	// Call the executable and wait for it to finish
	int result = std::system(command.c_str());
	// Check the result
	if (result == 0) {
		std::cout << "Executable finished successfully." << std::endl;
	}
	else {
		std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
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
	std::vector<std::vector<glm::vec3>> realPositions(99); // where optiTrack detected
	std::stringstream PositionFile;

	// 5. Run the loop for the writing thread 
	float dt = (float)numGeometries / targetUPS;	// time interval between the matrices updates
	//float dt = 0.01f;
	float currTime = 0;								// current (virtual) time 
	bool moving = false;							// indicates the particles are moving or not
	bool testMode = false;
	bool finished = false;
	bool startRecording = false;
	bool startAnalysing = false;

	bool startTracking = false;
	bool going = true;

	float lastIntendedFinishTimePickUp = 0.0f;
	float pickUpRunningFlag = false;
	float afterPickUpExperiment = false;

	bool failDuringExperiment = false;
	bool failDuringInitial = false;

	std::vector<bool> particleDropStatus;

	std::ofstream f;

	int pIndex = 0;

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

		auto startTurbulance = std::chrono::system_clock::now();
		auto endTurbulance = std::chrono::system_clock::now();

		//if (afterPickUpExperiment) {
			//currTime = 0.0f;

			//afterPickUpExperiment = false;
			// Wait 5 seconds to run the experiment automatically
			//if (currTime > lastIntendedFinishTimePickUp + 5) {
			//	afterPickUpExperiment = false;
			//	goToTestingPos = true;
			//}
		//}

		// 5.1. Picking up from a bed
		if (pickingup || pickingup_2d) {
			if (algorithmUsed == TEMPORAL) GSPAT_TS::resetInitialPhases(solver);
			moving = true;
			currTime = 0;
			// detect the particle positions
			detectedPositions = getDetectedPositions(detector, numPoints);
			//std::vector<glm::vec3> detectedPositions = getDetectedPositions(Optitrack, numPoints);
			float aboveHeight = 0.062f; //0.062
			float stopDuration = 1.f;
			float initialiseVelocity = 0.01f;
			float initialTime = (aboveHeight - stageHeight) / initialiseVelocity + stopDuration;
			for (int i = 0; i < numPoints; i++)
				detectedPositions[i].z = aboveHeight;

			std::vector<AgentPath> tmpPaths;

			// compute the paths
			if (pickingup) {
				tmpPaths = setPaths(planner, detectedPositions, initialPositions, size, particles_pickup_speed, false, true);
			}
			else {
				std::vector<Agent> agents;
				//			agents = {
				//{0, {{0, -0.04, -0.04, 0.0}}},  // Agent 0 starting at z=0.0
				//{1, {{0, 0.04, 0.04, 0.0}}}     // Agent 1 starting at z=0.0
				//			};

				std::string filePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/pickup_transition.csv"; // Specify your file path here

				std::vector<double> targetZs;
				std::vector<Agent> agents1, agents2;
				targetZs = {};  // to all midpoint

				for (int i = 0; i < numAgents; i++) {
					// Create the initial waypoint for this agent
					Waypoint initialWaypoint(0, detectedPositions[i][0], detectedPositions[i][1], detectedPositions[i][2]);

					// Initialize the Agent with the initial waypoint
					agents.push_back(Agent(i, { initialWaypoint }));
					targetZs.push_back(0.12f);
				}

				for (int i = 0; i < numAgents; ++i) {
					generateWaypoints(agents[i], particles_pickup_speed[0], targetZs[i]);
				}

				writeOutputToFile(filePath, agents);
				std::cout << "Intermediate Output written to " << filePath << std::endl;

				//tmpPaths = PathPlanner::readAgentPaths(filePath);

				// Another transform (movement) phase
				path1plannerFailed = false;
				// 1st part
				glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight);
				glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight);
				planner.setBoundary(boundaryMin, boundaryMax);
				glm::vec3 size(0.01, 0.01, 0.012);
				std::vector<std::string> filePaths = {
					filePath,
					"C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths1.csv",
					"C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths2.csv"
				};

				std::string txtFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_goals.txt";
				std::string csvFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_goals_planned.csv";

				std::ofstream fio(txtFilePath);

				std::string mergeFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths_merge.csv";

				std::vector<glm::vec3> detectedPositionsRevisited;
				std::vector<glm::vec3> initialPositionsRevisited;
				for (int i = 0;i < detectedPositions.size(); i++) {
					glm::vec3 temp_particle_d = detectedPositions[i];
					glm::vec3 temp_particle_t = initialPositions[i];
					temp_particle_d[2] = centreHeight;
					temp_particle_t[2] = centreHeight;
					detectedPositionsRevisited.push_back(temp_particle_d);
					initialPositionsRevisited.push_back(temp_particle_t);
				}

				fio << "start" << std::endl;
				for (int i = 0;i < detectedPositions.size(); i++) {
					fio << detectedPositionsRevisited[i][0] << "," << detectedPositionsRevisited[i][1] << "," << detectedPositionsRevisited[i][2] << std::endl;
				}

				fio << "end" << std::endl;
				for (int i = 0;i < detectedPositions.size(); i++) {
					fio << initialPositionsRevisited[i][0] << "," << initialPositionsRevisited[i][1] << "," << initialPositionsRevisited[i][2] << std::endl;
				}

				fio << "speed" << std::endl;
				if (particles_pickup_speed[0] >= 0.25f) {
					fio << "0.25" << std::endl;
				}
				else {
					fio << particles_pickup_speed[0] << std::endl;
				}

				fio.close();

				std::vector<AgentPath> paths1;
				std::vector<AgentPath> paths1_alt;
				if (usePath1) {
					paths1 = setPaths(planner, detectedPositionsRevisited, initialPositionsRevisited, size, particles_pickup_speed, false, false);
				}
				else {
					// Weicheng Path Planner
					const char* executablePath = "C:/Python311/python.exe ";
					std::string argument1 = "c:/Users/weicheng/Desktop/formal_dataset/pathplan_v2.py";

					// Construct the full command
					std::string command = executablePath + argument1;

					// Call the executable and wait for it to finish
					int result = std::system(command.c_str());
					// Check the result
					if (result == 0) {
						std::cout << "Alt Path Planning Executable finished successfully." << std::endl;
					}
					else {
						std::cerr << "Alt Path Planning Executable finished with errors. Return code: " << result << std::endl;
						logFile << "Alt Path Planning Executable finished with errors. Return code: " << result << "\n";
						path1plannerFailed = true;
					}

					paths1_alt = PathPlanner::readAgentPaths(csvFilePath);
				}

				if (paths1.size() > 0 && usePath1 || paths1_alt.size() > 0 && !usePath1) {
					if (usePath1) {
						PathPlanner::saveAgentPaths(filePaths[1], paths1);
					}
					else {
						PathPlanner::saveAgentPaths(filePaths[1], paths1_alt);
					}
					std::cout << "Path 1 intermediate Part Saved." << std::endl;

					// 2nd Part
					std::vector<Agent> agents;
					//			agents = {
					//{0, {{0, -0.04, -0.04, 0.0}}},  // Agent 0 starting at z=0.0
					//{1, {{0, 0.04, 0.04, 0.0}}}     // Agent 1 starting at z=0.0
					//			};

					std::vector<double> targetZs;
					std::vector<Agent> agents1, agents2;

					//std::string outputFilePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/testMerge.csv";
					targetZs = {};  // to all midpoint

					for (int i = 0; i < numAgents; i++) {
						// Create the initial waypoint for this agent
						Waypoint initialWaypoint(0, initialPositions[i][0], initialPositions[i][1], centreHeight);

						// Initialize the Agent with the initial waypoint
						agents.push_back(Agent(i, { initialWaypoint }));
						targetZs.push_back(initialPositions[i][2]);
					}

					for (int i = 0; i < numAgents; ++i) {
						generateWaypoints(agents[i], particles_pickup_speed[0], targetZs[i]);
					}

					writeOutputToFile(filePaths[2], agents);
					std::cout << "Second Intermediate Output written to " << filePaths[2] << std::endl;

					// Merge
					mergeCSVFiles(filePaths, mergeFilePath);
					std::cout << "Merged output written to " << mergeFilePath << std::endl;

					// Final Read & Confirm Path
					tmpPaths = PathPlanner::readAgentPaths(mergeFilePath);
					//tmmPaths = PathPlanner::readAgentPaths(filePath);
				}
				else {
					std::cout << "Initialization: Part 1 selected path planner failed, period. Maybe try the other planner?" << std::endl;
					path1plannerFailed = true;
				}

				//setPaths(planner, detectedPositions, initialPositions, size, particles_pickup_speed, false, true);
				//mergeCSVFiles(filePaths, outputFilePath);
				//std::cout << "Merged output written to " << outputFilePath << std::endl;
			}

			//float time_offset = 0.0f; // this let particles picked up one by one

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

				if (!path1plannerFailed) {
					turnTransducersOff = false; // turn the transducers on
					pickUpRunningFlag = true;
				}

				pickingup = false;
				pickingup_2d = false;
			}
			else {
				pickingup = false;
				pickingup_2d = false;
			}
			glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight - 0.06f);
			glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight + 0.06f);
			planner.setBoundary(boundaryMin, boundaryMax);
			glm::vec3 size(0.009, 0.009, 0.9);
		}

		if (speed_reduce_calc || speed_increase_calc) {
			int mode_select = 0;
			if (speed_reduce_calc) {
				mode_select = 0;
			}
			else {
				mode_select = 1;
			}
			speed_reduce_calc = false;
			speed_increase_calc = false;
			std::vector<float> temp_speeds;

			if (mode_select == 0) {
				std::cout << "** Speed Reduction Calculation **" << std::endl;
			}
			else {
				std::cout << "** Speed Increment Calculation **" << std::endl;
			}

			std::cout << "Current Speeds at: \n";
			for (int i = 0; i < numAgents; i++) {
				temp_speeds.push_back(particles_speed[i]);
				std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
			}
			std::cout << std::endl;

			//initialPositions.clear();
			//targetPositions.clear();
			//for (int i = 0; i < existingPath.size(); i++)
			//{
			//	initialPositions.push_back(existingPath[i].getPosition(0));
			//	targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
			//	paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
			//	std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
			//}
			std::vector<std::vector<AgentPath>> allAvailableSol;
			int shortest_index;
			float shortest_final_time = 999.0f; // global
			for (int i = 0; i < numAgents; i++) {
				float local_swap = temp_speeds[i];
				if (mode_select == 0) {
					temp_speeds[i] = temp_speeds[i] - stepping;
				}
				else {
					temp_speeds[i] = temp_speeds[i] + stepping;
				}
				if (temp_speeds[i] < 0.01f) {
					temp_speeds[i] = 0.01f;
				}
				paths = setPaths(planner, initialPositions, targetPositions, size, temp_speeds, false, true);
				allAvailableSol.push_back(paths);
				std::cout << "When Particle " << i << ", reduced/increased to " << temp_speeds[i] << " m/s: \n";

				std::cout << "All particles speeds for calculation: ";
				for (int j = 0;j < numAgents; j++) {
					std::cout << temp_speeds[j] << "  ";
				}
				std::cout << std::endl;

				float longest_final_time = 0.0f; // local

				for (int j = 0; j < paths.size(); j++) {
					float temp_time = paths[i].getFinalTime();
					if (temp_time > longest_final_time) {
						longest_final_time = temp_time;
					}
				}

				if (longest_final_time < shortest_final_time) {
					shortest_final_time = longest_final_time;
					shortest_index = i;
				}

				std::cout << "The longest finish time reported when reducing/increasing particle " << i << ", is " << longest_final_time << std::endl;

				// back to original value
				temp_speeds[i] = local_swap;
			}

			std::cout << "The shortest finish time between all longest local finish times reported is: " << shortest_final_time << std::endl;
			std::cout << "Thus, the reduction selection is done by reducing/increasing the speed of particle #[" << shortest_index << "]\n" << std::endl;

			existingPath = allAvailableSol[shortest_index];
			if (mode_select == 0) {
				particles_speed[shortest_index] = particles_speed[shortest_index] - stepping;
			}
			else {
				particles_speed[shortest_index] = particles_speed[shortest_index] + stepping;
			}

			std::cout << "Reduced/Increased the speed of particle #" << shortest_index << ", to " << particles_speed[shortest_index] << "\n" << std::endl;

			std::cout << "Now, all particles speed: ";
			for (int j = 0; j < numAgents; j++) {
				std::cout << particles_speed[j] << "  ";
			}
			std::cout << std::endl;
		}

		if (binarySearchHigherLower) {
			upper_limit = current_speedo;
			current_speedo = lower_limit + (upper_limit - lower_limit) / 2;
			particles_speed.clear();
			for (int i = 0; i < numAgents; i++) {
				particles_speed.push_back(current_speedo);
			}

			existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
			initialPositions.clear();
			targetPositions.clear();
			for (int i = 0; i < existingPath.size(); i++)
			{
				initialPositions.push_back(existingPath[i].getPosition(0));
				targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
				paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
				std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
			}
			std::cout << "** Binary Searching the minimum condition for all particles dropping **\n" << "Now we are finding: the maximum speed the system is able to handle. \nLowering the upper limit" << std::endl;

			std::cout << "Current Speeds at: \n";
			for (int i = 0; i < numAgents; i++) {
				std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
			}
			std::cout << std::endl;

			create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			std::cout << "Current Paths Saved to exp csv." << std::endl;

			std::cout << "Current Lower Limit: " << lower_limit << ", Current Upper Limit: " << upper_limit << std::endl;

			const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
			std::string argument1 = std::to_string(numAgents);  // Replace with actual arguments
			std::string argument2 = roundNumm;
			std::string argument3 = testPattern;
			std::string argument4 = std::to_string(particles_speed[0]);
			std::string argument5 = "full";

			// Construct the full command
			std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

			// Call the executable and wait for it to finish
			int result = std::system(command.c_str());
			// Check the result
			if (result == 0) {
				std::cout << "Executable finished successfully." << std::endl;
			}
			else {
				std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
			}

			binarySearchHigherLower = false;
		}

		if (binarySearchLowerUpper) {
			lower_limit = current_speedo;
			current_speedo = lower_limit + (upper_limit - lower_limit) / 2;
			particles_speed.clear();
			for (int i = 0; i < numAgents; i++) {
				particles_speed.push_back(current_speedo);
			}

			existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
			initialPositions.clear();
			targetPositions.clear();
			for (int i = 0; i < existingPath.size(); i++)
			{
				initialPositions.push_back(existingPath[i].getPosition(0));
				targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
				paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
				std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
			}
			std::cout << "** Binary Searching the minimum condition for all particles dropping **\n" << "Now we are finding: the maximum speed the system is able to handle. \nPushing the lower limit higher" << std::endl;

			std::cout << "Current Speeds at: \n";
			for (int i = 0; i < numAgents; i++) {
				std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
			}
			std::cout << std::endl;

			create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			std::cout << "Current Paths Saved to exp csv." << std::endl;

			const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
			std::string argument1 = std::to_string(numAgents);  // Replace with actual arguments
			std::string argument2 = roundNumm;
			std::string argument3 = testPattern;
			std::string argument4 = std::to_string(particles_speed[0]);
			std::string argument5 = "full";

			// Construct the full command
			std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

			// Call the executable and wait for it to finish
			int result = std::system(command.c_str());
			// Check the result
			if (result == 0) {
				std::cout << "Executable finished successfully." << std::endl;
			}
			else {
				std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
			}

			std::cout << "Current Lower Limit: " << lower_limit << ", Current Upper Limit: " << upper_limit << std::endl;

			binarySearchLowerUpper = false;
		}

		if (mlpAnalysis) {
			mlpAnalysis = false;
			const char* executablePath2 = "python ";

			argument1 = "c:/Users/weicheng/Desktop/formal_dataset/mlp_backward.py";
			argument2 = "--numParticles " + std::to_string(numAgents);
			argument3 = "--data_file \"C:/Users/weicheng/Desktop/formal_dataset/training_data_alt_" + std::to_string(numAgents) + "_particles.txt\"";
			argument4 = "--particle_no 0";
			std::string argument5 = "--predict_mode 2";
			std::string argument6 = "--path_id " + std::to_string(numAgents) + "_" + algorithmUsedd + "_" + testPattern + "_" + roundNumm + "_" + systemVersion;
			std::string argument7 = "predict";
			command = executablePath2 + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5 + " " + argument6 + " " + argument7;

			result = std::system(command.c_str());
			bool result_success_flag = false;
			// Check the result
			if (result == 0) {
				std::cout << "Inference finished successfully." << std::endl;
				result_success_flag = true;
			}
			else {
				std::cerr << "Inference finished with errors. Return code: " << result << std::endl;
				logFile << "Inference finished with errors. Return code: " << result << "\n";
			}

			float tempFloat;
			int intValue;
			float floatValue;

			if (result_success_flag) {
				result_success_flag = false;
				const std::string filePath = "C:\\Users\\weicheng\\Desktop\\ServerSampler\\bin\\mlpTrainerInference.txt";
				int maxRetries = 15;
				bool fileFound = false;

				while (maxRetries--) {
					if (std::filesystem::exists(filePath)) {
						fileFound = true;
						break;
					}
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}

				if (fileFound) {
					std::ifstream file(filePath);
					if (file.is_open()) {
						std::string line;
						if (std::getline(file, line)) {
							std::istringstream iss(line);

							iss >> tempFloat >> floatValue;
							intValue = static_cast<int>(tempFloat);

							std::cout << "Integer: " << intValue << ", Float: " << floatValue << std::endl;
							result_success_flag = true;
						}
						file.close();

						// Delete the file
						if (std::filesystem::remove(filePath)) {
							std::cout << "File deleted successfully." << std::endl;
						}
						else {
							std::cerr << "Failed to delete the file." << std::endl;
						}
					}
					else {
						std::cerr << "Failed to open the file." << std::endl;
					}
				}
				else {
					std::cerr << "File not found after 15 retries." << std::endl;
				}
			}

			if (result_success_flag) {
				result_success_flag = false;
				current_speedo = floatValue;
				// Amplitude Adjustments
				for (int i = 0; i < numAgents; i++) {
					for (int g = 0; g < numGeometries; g++) {
						ampBuffer[g * numPoints + i] = intValue;
					}
					ampOffsets[i] = intValue - 16000;
				}
				std::cout << "Set the current amplitude to " << intValue << ", and velocity to " << floatValue << "." << std::endl;


				particles_speed.clear();
				for (int i = 0; i < numAgents; i++) {
					particles_speed.push_back(current_speedo);
				}

				existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
				initialPositions.clear();
				targetPositions.clear();
				for (int i = 0; i < existingPath.size(); i++)
				{
					initialPositions.push_back(existingPath[i].getPosition(0));
					targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
					paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
					std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
				}

				std::cout << "Current Speeds at: \n";
				for (int i = 0; i < numAgents; i++) {
					std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
				}
				std::cout << std::endl;

				create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				std::cout << "Current Paths Saved to exp csv." << std::endl;

				const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
				std::string argument1 = std::to_string(numAgents);
				std::string argument2 = roundNumm;
				std::string argument3 = testPattern;
				std::string argument4 = std::to_string(particles_speed[0]);
				std::string argument5 = "full";

				// Construct the full command
				std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

				// Call the executable and wait for it to finish
				int result = std::system(command.c_str());
				// Check the result
				if (result == 0) {
					std::cout << "Executable finished successfully." << std::endl;
				}
				else {
					std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
				}

			}
		}

		if (simpleAnalysis) {
			simpleAnalysis = false;

			const char* executablePath1 = "python ";

			argument1 = "C:/Users/weicheng/Desktop/formal_dataset/probability_pair.py";
			argument2 = std::to_string(numAgents);
			if (simpleAnalysisFullMode) {
				argument3 = std::to_string(numAgents) + "_" + algorithmUsedd + "_" + testPattern + "_" + roundNumm + "_" + systemVersion;
			}
			else {
				argument3 = "";
			}

			command = executablePath1 + argument1 + " " + argument2 + " " + argument3;

			result = std::system(command.c_str());
			bool result_success_flag = false;
			// Check the result
			if (result == 0) {
				std::cout << "Inference finished successfully." << std::endl;
				result_success_flag = true;
			}
			else {
				std::cerr << "Inference finished with errors. Return code: " << result << std::endl;
				logFile << "Inference finished with errors. Return code: " << result << "\n";
			}

			float tempFloat;
			int intValue;
			float floatValue;

			if (result_success_flag) {
				result_success_flag = false;
				const std::string filePath = "C:\\Users\\weicheng\\Desktop\\ServerSampler\\bin\\bnTrainerInference.txt";
				int maxRetries = 15;
				bool fileFound = false;

				while (maxRetries--) {
					if (std::filesystem::exists(filePath)) {
						fileFound = true;
						break;
					}
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}

				if (fileFound) {
					std::ifstream file(filePath);
					if (file.is_open()) {
						std::string line;
						if (std::getline(file, line)) {
							std::istringstream iss(line);

							iss >> tempFloat >> floatValue;
							intValue = static_cast<int>(tempFloat);

							std::cout << "Integer: " << intValue << ", Float: " << floatValue << std::endl;
							result_success_flag = true;
						}
						file.close();

						// Delete the file
						if (std::filesystem::remove(filePath)) {
							std::cout << "File deleted successfully." << std::endl;
						}
						else {
							std::cerr << "Failed to delete the file." << std::endl;
						}
					}
					else {
						std::cerr << "Failed to open the file." << std::endl;
					}
				}
				else {
					std::cerr << "File not found after 15 retries." << std::endl;
				}
			}

			if (result_success_flag) {
				result_success_flag = false;
				current_speedo = floatValue;
				// Amplitude Adjustments
				for (int i = 0; i < numAgents; i++) {
					for (int g = 0; g < numGeometries; g++) {
						ampBuffer[g * numPoints + i] = intValue;
					}
					ampOffsets[i] = intValue - 16000;
				}
				std::cout << "Set the current amplitude to " << intValue << ", and velocity to " << floatValue << "." << std::endl;


				particles_speed.clear();
				for (int i = 0; i < numAgents; i++) {
					particles_speed.push_back(current_speedo);
				}

				existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
				initialPositions.clear();
				targetPositions.clear();
				for (int i = 0; i < existingPath.size(); i++)
				{
					initialPositions.push_back(existingPath[i].getPosition(0));
					targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
					paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
					std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
				}

				std::cout << "Current Speeds at: \n";
				for (int i = 0; i < numAgents; i++) {
					std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
				}
				std::cout << std::endl;

				create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				std::cout << "Current Paths Saved to exp csv." << std::endl;

				const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
				std::string argument1 = std::to_string(numAgents);
				std::string argument2 = roundNumm;
				std::string argument3 = testPattern;
				std::string argument4 = std::to_string(particles_speed[0]);
				std::string argument5 = "full";

				// Construct the full command
				std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

				// Call the executable and wait for it to finish
				int result = std::system(command.c_str());
				// Check the result
				if (result == 0) {
					std::cout << "Executable finished successfully." << std::endl;
				}
				else {
					std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
				}

			}

		}

		if (bnAnalysis) {
			bnAnalysis = false;

			const char* executablePath1 = "python ";
			if (bnAnalysisFullMode) {
				argument1 = "C:/Users/weicheng/Desktop/formal_dataset/bn_inference_v2.py";
			}
			else {
				argument1 = "C:/Users/weicheng/Desktop/formal_dataset/bn_inference_v2_alt.py";
			}

			argument2 = "--num_particles " + std::to_string(numAgents);
			argument3 = "--mode predict";
			argument4 = "--observation 0 \"" + std::to_string(numAgents) + "_" + algorithmUsedd + "_" + testPattern + "_" + roundNumm + "_" + systemVersion + "\"";
			command = executablePath1 + argument1 + " " + argument2 + " " + argument3 + " " + argument4;

			result = std::system(command.c_str());
			bool result_success_flag = false;
			// Check the result
			if (result == 0) {
				std::cout << "Inference finished successfully." << std::endl;
				result_success_flag = true;
			}
			else {
				std::cerr << "Inference finished with errors. Return code: " << result << std::endl;
				logFile << "Inference finished with errors. Return code: " << result << "\n";
			}

			float tempFloat;
			int intValue;
			float floatValue;

			if (result_success_flag) {
				result_success_flag = false;
				const std::string filePath = "C:\\Users\\weicheng\\Desktop\\ServerSampler\\bin\\bnTrainerInference.txt";
				int maxRetries = 15;
				bool fileFound = false;

				while (maxRetries--) {
					if (std::filesystem::exists(filePath)) {
						fileFound = true;
						break;
					}
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}

				if (fileFound) {
					std::ifstream file(filePath);
					if (file.is_open()) {
						std::string line;
						if (std::getline(file, line)) {
							std::istringstream iss(line);

							iss >> tempFloat >> floatValue;
							intValue = static_cast<int>(tempFloat);

							std::cout << "Integer: " << intValue << ", Float: " << floatValue << std::endl;
							result_success_flag = true;
						}
						file.close();

						// Delete the file
						if (std::filesystem::remove(filePath)) {
							std::cout << "File deleted successfully." << std::endl;
						}
						else {
							std::cerr << "Failed to delete the file." << std::endl;
						}
					}
					else {
						std::cerr << "Failed to open the file." << std::endl;
					}
				}
				else {
					std::cerr << "File not found after 15 retries." << std::endl;
				}
			}

			if (result_success_flag) {
				result_success_flag = false;
				current_speedo = floatValue;
				// Amplitude Adjustments
				for (int i = 0; i < numAgents; i++) {
					for (int g = 0; g < numGeometries; g++) {
						ampBuffer[g * numPoints + i] = intValue;
					}
					ampOffsets[i] = intValue - 16000;
				}
				std::cout << "Set the current amplitude to " << intValue << ", and velocity to " << floatValue << "." << std::endl;


				particles_speed.clear();
				for (int i = 0; i < numAgents; i++) {
					particles_speed.push_back(current_speedo);
				}

				existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
				initialPositions.clear();
				targetPositions.clear();
				for (int i = 0; i < existingPath.size(); i++)
				{
					initialPositions.push_back(existingPath[i].getPosition(0));
					targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
					paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
					std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
				}

				std::cout << "Current Speeds at: \n";
				for (int i = 0; i < numAgents; i++) {
					std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
				}
				std::cout << std::endl;

				create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				savedPathFile.str("");
				savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile.csv";
				PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
				std::cout << "Current Paths Saved to exp csv." << std::endl;

				const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
				std::string argument1 = std::to_string(numAgents);
				std::string argument2 = roundNumm;
				std::string argument3 = testPattern;
				std::string argument4 = std::to_string(particles_speed[0]);
				std::string argument5 = "full";

				// Construct the full command
				std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

				// Call the executable and wait for it to finish
				int result = std::system(command.c_str());
				// Check the result
				if (result == 0) {
					std::cout << "Executable finished successfully." << std::endl;
				}
				else {
					std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
				}

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
			paths = setPaths(planner, initialPositions, initialTestingPositions, size, particles_speed, false, true);
			currTime = 0;
			moving = true;
			goToTestingStart = false;
			//exisingPathFile.str("");
		}
		if (goToTestingEnd)   // temporally update the tracking and recording signal here
		{
			currTime = 0;
			moving = true;
			paths = existingPath;
			//startTracking = true;
			//startRecording = true;
			goToTestingEnd = false;
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

		if (goToTestingPos || goToTestingPos_2d)    // 3
		{
			// compute the paths
			//velocity = 0.01;
			if (goToTestingPos_2d) {

				// Pre-movement Part: Lowering/Highering to the same height (centre height).
				std::vector<Agent> agents;
				//			agents = {
				//{0, {{0, -0.04, -0.04, 0.0}}},  // Agent 0 starting at z=0.0
				//{1, {{0, 0.04, 0.04, 0.0}}}     // Agent 1 starting at z=0.0
				//			};

				std::string filePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/during_transition.csv"; // Specify your file path here

				std::vector<double> targetZs;
				std::vector<Agent> agents1, agents2;
				targetZs = {};  // to all midpoint

				for (int i = 0; i < numAgents; i++) {
					// Create the initial waypoint for this agent
					Waypoint initialWaypoint(0, initialPositions[i][0], initialPositions[i][1], initialPositions[i][2]);

					// Initialize the Agent with the initial waypoint
					agents.push_back(Agent(i, { initialWaypoint }));
					targetZs.push_back(0.12f);
				}

				for (int i = 0; i < numAgents; ++i) {
					generateWaypoints(agents[i], particles_speed[0], targetZs[i]);
				}

				writeOutputToFile(filePath, agents);
				std::cout << "Intermediate Output written to " << filePath << std::endl;


				path1plannerFailed = false;
				// 1st part
				glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight);
				glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight);
				planner.setBoundary(boundaryMin, boundaryMax);
				glm::vec3 size(0.01, 0.01, 0.012);
				std::vector<std::string> filePaths = {
					filePath,
					"C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths1.csv",
					"C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths2.csv"
				};

				std::string txtFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_goals.txt";
				std::string csvFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_goals_planned.csv";

				std::ofstream fio(txtFilePath);

				std::string mergeFilePath = "C:/Users/weicheng/Desktop/ServerSampler/bin/2d_paths_merge.csv";
				if (developmentMode) {
					detectedPositions.clear();
					detectedPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
					detectedPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
					detectedPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
					detectedPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D
					//detectedPositions.push_back(glm::vec3(0.04f, 0.00f, 0.12f)); // E
					//detectedPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // F
					//detectedPositions.push_back(glm::vec3(0.00f, 0.04f, 0.12f)); // G
					//detectedPositions.push_back(glm::vec3(0.00f, -0.04f, 0.12f)); // H

					targetPositions.clear();
					targetPositions.push_back(glm::vec3(0.04f, -0.04f, 0.16f)); // A
					targetPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.05f)); // B
					targetPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.06f)); // C
					targetPositions.push_back(glm::vec3(0.04f, 0.04f, 0.11f)); // D
				}

				std::vector<glm::vec3> initialPositionsRevisited;
				std::vector<glm::vec3> targetPositionsRevisited;
				for (int i = 0;i < initialPositions.size(); i++) {
					glm::vec3 temp_particle_d = initialPositions[i];
					glm::vec3 temp_particle_t = targetPositions[i];
					temp_particle_d[2] = centreHeight;
					temp_particle_t[2] = centreHeight;
					initialPositionsRevisited.push_back(temp_particle_d);
					targetPositionsRevisited.push_back(temp_particle_t);
				}

				fio << "start" << std::endl;
				for (int i = 0;i < initialPositions.size(); i++) {
					fio << initialPositionsRevisited[i][0] << "," << initialPositionsRevisited[i][1] << "," << initialPositionsRevisited[i][2] << std::endl;
				}

				fio << "end" << std::endl;
				for (int i = 0;i < detectedPositions.size(); i++) {
					fio << targetPositionsRevisited[i][0] << "," << targetPositionsRevisited[i][1] << "," << targetPositionsRevisited[i][2] << std::endl;
				}

				fio << "speed" << std::endl;
				if (current_speedo >= 0.08f) {
					fio << "0.08" << std::endl;
				}
				else {
					fio << current_speedo << std::endl;
				}

				fio.close();

				std::vector<float> particle_speed_revisited;
				for (int i = 0; i < numAgents; i++) {
					if (current_speedo >= 0.05f) {
						particle_speed_revisited.push_back(0.05f);
					}
					else {
						particle_speed_revisited.push_back(current_speedo);
					}
				}

				std::vector<AgentPath> paths1;
				std::vector<AgentPath> paths1_alt;
				if (usePath1) {
					paths1 = setPaths(planner, initialPositionsRevisited, targetPositionsRevisited, size, particle_speed_revisited, false, false);
				}
				else {
					// Weicheng Path Planner
					const char* executablePath = "C:/Python311/python.exe ";
					std::string argument1 = "c:/Users/weicheng/Desktop/formal_dataset/pathplan_v2.py";

					// Construct the full command
					std::string command = executablePath + argument1;

					// Call the executable and wait for it to finish
					int result = std::system(command.c_str());
					// Check the result
					if (result == 0) {
						std::cout << "Alt Path Planning Executable finished successfully." << std::endl;
					}
					else {
						std::cerr << "Alt Path Planning Executable finished with errors. Return code: " << result << std::endl;
						logFile << "Alt Path Planning Executable finished with errors. Return code: " << result << "\n";
						path1plannerFailed = true;
					}

					paths1_alt = PathPlanner::readAgentPaths(csvFilePath);
				}

				if (paths1.size() > 0 && usePath1 || paths1_alt.size() > 0 && !usePath1) {
					if (usePath1) {
						PathPlanner::saveAgentPaths(filePaths[1], paths1);
					}
					else {
						PathPlanner::saveAgentPaths(filePaths[1], paths1_alt);
					}
					std::cout << "Path 1 intermediate Part Saved." << std::endl;

					// 2nd Part
					std::vector<Agent> agents;
					//			agents = {
					//{0, {{0, -0.04, -0.04, 0.0}}},  // Agent 0 starting at z=0.0
					//{1, {{0, 0.04, 0.04, 0.0}}}     // Agent 1 starting at z=0.0
					//			};

					std::vector<double> targetZs;
					std::vector<Agent> agents1, agents2;

					//std::string outputFilePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/testMerge.csv";
					targetZs = {};  // to all midpoint

					for (int i = 0; i < numAgents; i++) {
						// Create the initial waypoint for this agent
						Waypoint initialWaypoint(0, targetPositions[i][0], targetPositions[i][1], centreHeight);

						// Initialize the Agent with the initial waypoint
						agents.push_back(Agent(i, { initialWaypoint }));
						targetZs.push_back(targetPositions[i][2]);
					}

					for (int i = 0; i < numAgents; ++i) {
						generateWaypoints(agents[i], current_speedo, targetZs[i]);
					}

					writeOutputToFile(filePaths[2], agents);
					std::cout << "Second Intermediate Output written to " << filePaths[2] << std::endl;

					// Merge
					mergeCSVFiles(filePaths, mergeFilePath);
					std::cout << "Merged output written to " << mergeFilePath << std::endl;

					// Final Read & Confirm Path
					paths = PathPlanner::readAgentPaths(mergeFilePath);
					//if (usePath1) {
					//	paths = paths1;
					//}
					//else {
					//	paths = paths1_alt;
					//}

					// Copy one to the current folder and generate other features
					CopyFileFromAToB(mergeFilePath, integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_path_file_alt_plan.csv");
					CopyFileFromAToB(mergeFilePath, "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/pathFile_alt_plan.csv");

					const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
					std::string argument1 = std::to_string(numAgents);  // Replace with actual arguments
					std::string argument2 = roundNumm;
					std::string argument3 = testPattern;
					std::string argument4 = std::to_string(current_speedo);
					std::string argument5 = "alt";

					// Construct the full command
					std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

					// Call the executable and wait for it to finish
					int result = std::system(command.c_str());
					// Check the result
					if (result == 0) {
						std::cout << "Executable finished successfully." << std::endl;
					}
					else {
						std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
					}
				}
				else {
					std::cout << "Part 1 selected path planner failed, period. Maybe try the other planner?" << std::endl;
					path1plannerFailed = true;
				}
			}
			else {
				paths = existingPath;
			}

			//paths = setPaths(planner, initialPositions, targetPositions, size, testing_velocity, false, true);
			std::cout << "All particles speeds for running this experiment: ";
			for (int j = 0;j < numAgents; j++) {
				std::cout << particles_speed[j] << "  ";
			}
			std::cout << std::endl;

			std::cout << "All particles amplitudes for running this experiment: ";
			for (int j = 0; j < numAgents; j++) {
				std::cout << 16000 + ampOffsets[j] << "  ";
			}
			std::cout << std::endl;

			std::ofstream ampFile(integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_amplitudeFile.txt");
			if (!ampFile.is_open()) {
				std::cerr << "Failed to open the file for writing amplitudes log!" << std::endl;
			}
			else {
				ampFile << numAgents << "_" << algorithmUsedd << "_" << testPattern << "_" << roundNumm << "_" << systemVersion << "\n";
				for (int j = 0; j < numAgents; j++) {
					ampFile << j << " " << std::to_string(16000 + ampOffsets[j]) + "\n";
				}
				ampFile.close();
			}

			//savedPathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo/Output" << "/" << folderName << "/" << "agent-" << numPoints << "-maxV-" << velocity << "-round-" << runningTimes << "failed-pathFile.csv";
			//PathPlanner::saveAgentPaths(savedPathFile.str(), paths);

			if (!manualMode && !noCaptureMode) {
				// Create an ofstream object to open a file for writing
				std::ofstream file("C:/Users/weicheng/Desktop/ServerSampler/bin/example.txt");

				// Check if the file is opened successfully
				if (!file.is_open()) {
					std::cerr << "Failed to open the file!" << std::endl;
				}
				else {
					// Write some lines to the file
					file << "0\n";
					file << std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "\n";
					file << "C:/Users/weicheng/Desktop/formal_dataset/test_auto/" << std::endl;

					logFile << std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "\n";
					for (int i = 0; i < ampOffsets.size(); i++) {
						logFile << i << " " << ampOffsets[i] << "\n";
					}
					logFile << "\n";

					// Close the file
					file.close();

					std::cout << "Start request written to the file successfully." << std::endl;
				}

				std::this_thread::sleep_for(std::chrono::seconds(3));
			}

			currTime = 0;
			if (goToTestingPos_2d && path1plannerFailed) {
				std::cout << "Path Planning Failed, Try again by using different location." << std::endl;
				turnTransducersOff = true;
				path1plannerFailed = false;
			}
			else {
				moving = true;
				startTracking = true;
				finished = false;
				afterPickUpExperiment = false;
			}

			goToTestingPos = false;
			if (goToTestingPos_2d) {
				glm::vec3 boundaryMin(-0.06f, -0.06f, centreHeight - 0.06f);
				glm::vec3 boundaryMax(+0.06f, +0.06f, centreHeight + 0.06f);
				planner.setBoundary(boundaryMin, boundaryMax);
				glm::vec3 size(0.009, 0.009, 0.9);
				goToTestingPos_2d = false;
			}
		}

		if (writingToFile) {
			savedPathFile << "C:/Users/weicheng/Desktop/OpenMPD_Demo1/weicheng_results/test.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			std::cout << "Current Paths Saved." << std::endl;
			writingToFile = false;
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

		if (manualSpeedIncrease) {
			manualSpeedIncrease = false;

			std::cout << "Manual Velocity Increment by 0.01" << std::endl;
			current_speedo += 0.01f;

			particles_speed.clear();
			for (int i = 0; i < numAgents; i++) {
				particles_speed.push_back(current_speedo);
			}

			existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
			initialPositions.clear();
			targetPositions.clear();
			for (int i = 0; i < existingPath.size(); i++)
			{
				initialPositions.push_back(existingPath[i].getPosition(0));
				targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
				paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
				std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
			}

			std::cout << "Current Speeds at: \n";
			for (int i = 0; i < numAgents; i++) {
				std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
			}
			std::cout << std::endl;

			create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			std::cout << "Current Paths Saved to exp csv." << std::endl;

			const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
			std::string argument1 = std::to_string(numAgents);
			std::string argument2 = roundNumm;
			std::string argument3 = testPattern;
			std::string argument4 = std::to_string(particles_speed[0]);
			std::string argument5 = "full";

			// Construct the full command
			std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

			// Call the executable and wait for it to finish
			int result = std::system(command.c_str());
			// Check the result
			if (result == 0) {
				std::cout << "Executable finished successfully." << std::endl;
			}
			else {
				std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
			}
		}

		if (manualSpeedReduction) {
			manualSpeedReduction = false;

			std::cout << "Manual Velocity Reduction by 0.01" << std::endl;
			current_speedo -= 0.01f;

			particles_speed.clear();
			for (int i = 0; i < numAgents; i++) {
				particles_speed.push_back(current_speedo);
			}

			existingPath = setPaths(planner, initialPositions, targetPositions, size, particles_speed, false, true);
			initialPositions.clear();
			targetPositions.clear();
			for (int i = 0; i < existingPath.size(); i++)
			{
				initialPositions.push_back(existingPath[i].getPosition(0));
				targetPositions.push_back(existingPath[i].getPosition(existingPath[i].getFinalTime()));
				paths[i].addWayPoint(0, initialPositions[i].x, initialPositions[i].y, initialPositions[i].z);
				std::cout << existingPath[i].getPosition(0).x << ", " << existingPath[i].getPosition(0).y << ", " << existingPath[i].getPosition(0).z << std::endl;
			}

			std::cout << "Current Speeds at: \n";
			for (int i = 0; i < numAgents; i++) {
				std::cout << "Particle " << i << ", " << particles_speed[i] << " m/s\n";
			}
			std::cout << std::endl;

			create_directories_if_not_exist("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile_exp.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			savedPathFile.str("");
			savedPathFile << "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "pathFile.csv";
			PathPlanner::saveAgentPaths(savedPathFile.str(), existingPath);
			std::cout << "Current Paths Saved to exp csv." << std::endl;

			const char* executablePath = "C:/Users/weicheng/Desktop/AcousticPathPlanning/AcousticPathPlanning/x64/Debug/AcousticLevitation.exe ";
			std::string argument1 = std::to_string(numAgents);
			std::string argument2 = roundNumm;
			std::string argument3 = testPattern;
			std::string argument4 = std::to_string(particles_speed[0]);
			std::string argument5 = "full";

			// Construct the full command
			std::string command = executablePath + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

			// Call the executable and wait for it to finish
			int result = std::system(command.c_str());
			// Check the result
			if (result == 0) {
				std::cout << "Executable finished successfully." << std::endl;
			}
			else {
				std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
			}
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
			//float prev_time = -0.5f;

			int remainingParticles = 0;

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
				if (!finished && moving && startTracking)
				{
					//printf("Intended Finish time: %f, Current time: %f\n", lastIntendedFinishTime, currTime);

					if (p == 0)  		// add time once if currTime changed
					{
						times.push_back(currTime);
						//printf("time steps: %d ", times.size());
					}

					//trapPositions[p].push_back(glm::vec3(pos.x, pos.y, pos.z));  //{pos.x,pos.y,pos.z}

					//std::cout << "Current Beads Size: " << currentBeads.size() << std::endl;

					// number checking!
					//if (currentBeads.size() < numPoints)
					//{
					//	printf("some beads are not detected\n");
					//	realPositions[p].push_back(glm::vec3(999.0f, 999.0f, 999.0f));
					//}
					//else {
					//	//need to do mapping for post-process later
					//	realPositions[p].push_back(currentBeads[p]); //  the detected single position currentBeads[p] is the bead[p]'s real position (realPositions[p])  
					//	remainingParticles = currentBeads.size() - numPoints;
					//	//std::cout << "Remaining Particles Count: " << remainingParticles << std::endl;
					//	if (remainingParticles > 0) {
					//		for (int j = 0; j < remainingParticles; j++) {
					//			//std::cout << "Registering " << j << " Particle" << std::endl;
					//			try {
					//				realPositions[numPoints + j].push_back(currentBeads[numPoints + j]);
					//			}
					//			catch (const std::out_of_range& e) {
					//				std::cerr << "Out of range error: " << e.what() << std::endl;
					//				// Handle the error, possibly with a fallback mechanism or error logging
					//			}
					//			catch (const std::exception& e) {
					//				std::cerr << "An error occurred: " << e.what() << std::endl;
					//				// Handle the error
					//			}
					//			catch (...) {
					//				std::cerr << "An unknown error occurred." << std::endl;
					//				// Handle the unknown error
					//			}
					//		}
					//	}
					//}

					if (currTime > lastIntendedFinishTime) {
						numPointsSec = numAgents + remainingParticles;
						finished = true;
						startRecording = true;
						printf("------finished stat updating------\n");

						continue;
					}
				}

				// record and store timing, targetTrapPos and actual capturedPos
				// add some time constraints to start recording !
				//if (!finished && moving && startTracking)
				//{
				//	printf("Intended Finish time: %f, Current time: %f\n", lastIntendedFinishTime, currTime);

				//	if (p == 0)  		// add time once if currTime changed
				//	{
				//		times.push_back(currTime);
				//		printf("time steps: %d ", times.size());
				//	}

					//trapPositions[p].push_back(glm::vec3(pos.x, pos.y, pos.z));  //{pos.x,pos.y,pos.z}


					//if (prev_time < currTime) {
				//	prev_time = currTime;
				//}

				//if (currTime > lastIntendedFinishTime) {
				//	finished = true;
				//	firstTimeToLog = true;
				//	//startRecording = true;
				//	//continue;
				//}

				// number checking!

				//if (!finished && currTime != 0.0f && !pickUpRunningFlag && (!afterPickUpExperiment || (currTime <= lastIntendedFinishTime && prev_time != currTime))) {
				//	if (firstTimeToLog) {
				//		realPositions.clear();
				//		firstTimeToLog = false;
				//	}

				//	
				//	
				//	//if (currentBeads.size() < numPoints)
				//	//{
				//	//	printf("some beads are not detected\n");
				//	//}

				//}

				//}

			}
			if (moving) currTime += dt; // increment the current time

			//if (finished && startRecording) {
			//	// Sleep for 1 second
			//	std::this_thread::sleep_for(std::chrono::seconds(1));

			//	turnTransducersOff = true;

			//	std::ofstream outFile("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + numAgentss + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(current_speedo) + "/" + roundNumm + "/actual_pos.csv");
			//	// Check if the file is open
			//	if (!outFile) {
			//		std::cerr << "Failed to open the file." << std::endl;
			//	}

			//	outFile << "time,";
			//	for (int i = 0; i < currentBeads.size(); i++) {
			//		outFile << "x" << i << ",y" << i << ",z" << i;
			//	}
			//	outFile << std::endl;

			//	// Write the current time to the file
			//	outFile << currTime << ",";

			//	// Write the positions of the beads to the file
			//	for (size_t i = 0; i < currentBeads.size(); i++) {
			//		outFile << currentBeads[i].x << "," << currentBeads[i].y << "," << currentBeads[i].z << std::endl;
			//	}

			//	// Close the file
			//	outFile.close();
			//	startRecording = false;

			//	startAnalysing = true;
			//}

			if (finished && startRecording)
			{
				f.clear();

				// Sleep for 1 second
				std::this_thread::sleep_for(std::chrono::seconds(1));

				if (!guiManageMode) {
					turnTransducersOff = true;
				}

				PositionFile.clear();

				printf("------finished status updating------\n");

				if (!manualMode && !noCaptureMode) {
					// Create an ofstream object to open a file for writing
					std::ofstream file("C:/Users/weicheng/Desktop/ServerSampler/bin/example.txt");

					// Check if the file is opened successfully
					if (!file.is_open()) {
						std::cerr << "Failed to open the file!" << std::endl;
					}
					else {
						// Write some lines to the file
						file << "1\n";
						file << std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "\n";
						file << "C:/Users/weicheng/Desktop/formal_dataset/test_auto/" << std::endl;

						// Close the file
						file.close();

						std::cout << "Finish request written to the file successfully." << std::endl;
						printf("------ Wait the recording to end ------\n");
						std::this_thread::sleep_for(std::chrono::seconds(5));

						MoveFileFromAToB("C:/Users/weicheng/Desktop/formal_dataset/test_auto/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + ".tak", integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + ".tak");

						printf("------ Wait the data processing step to finish ------\n");

						std::string exeName = "Motive.exe"; // Replace with your program's exe name
						//std::string exePath = "C:/Program Files/OptiTrack/Motive/Motive.exe"; // Replace with the full path to the exe
						std::string exePath = "C:/Users/weicheng/Desktop/formal_dataset/240702_cal/2408191147.cal";

						if (!manualInputMode && !noCaptureMode) {
							// Kill the process
							KillProcessByName(exeName);

							const char* executablePath = "ipy ";
							std::string argument1 = "C:/Users/weicheng/Downloads/motivebatch-master/motivebatch-master/run.py";  // Replace with actual arguments
							std::string argument2 = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + ".tak";
							std::string argument3 = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + ".csv";
							//std::string argument4 = std::to_string(current_speedo);

							// Construct the full command
							std::string command = executablePath + argument1 + " " + argument2 + " " + argument3;

							// Call the executable and wait for it to finish
							int result = std::system(command.c_str());
							// Check the result
							if (result == 0) {
								std::cout << "Executable finished successfully." << std::endl;
							}
							else {
								std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
								logFile << "Executable finished with errors. Return code: " << result << "\n";
							}

							// Start the process again
							StartProcess(exePath);
						}

						//std::string argument4 = std::to_string(current_speedo);
						CopyFileFromAToB("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/" + "sim_output_TargetPosition.csv", integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_target_pos.csv");
						CopyFileFromAToB("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/pathFile.csv", integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_path_file.csv");
						CopyFileFromAToB("C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(particles_speed[0]) + "/" + roundNumm + "/pathFile_exp.csv", integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_path_file_exp.csv");

						if (!manualInputMode && !noCaptureMode) {
							const char* executablePath1 = "python ";
							argument1 = "c:/Users/weicheng/Desktop/formal_dataset/parsing_v2.py";  // Replace with actual arguments
							argument2 = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + ".csv";
							argument3 = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_target_pos.csv";
							argument4 = std::to_string(numAgents);
							std::string argument5 = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo);
							command = executablePath1 + argument1 + " " + argument2 + " " + argument3 + " " + argument4 + " " + argument5;

							result = std::system(command.c_str());
							// Check the result
							if (result == 0) {
								std::cout << "Executable finished successfully." << std::endl;
							}
							else {
								std::cerr << "Executable finished with errors. Return code: " << result << std::endl;
								logFile << "Executable finished with errors. Return code: " << result << "\n";
							}

							printf("------ Wait particles dropping result back ------\n");
							// TODO: dropping processing

							std::string filename = integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_c_pre_process_drop.txt";
							std::ifstream file(filename);


							// Check if file exists
							if (!file.is_open()) {
								std::cout << "File not found: " << filename << std::endl;

								logFile << "Dropping File Not Exist. (DFNE)\n";
							}
							else {
								std::string line;
								int particleNumber;
								char status;

								std::cout << "Particle dropping status:" << std::endl;

								// Loop through each line in the file
								while (std::getline(file, line)) {
									std::istringstream iss(line);

									// Parse the line (expecting format like "0 n", "1 y", etc.)
									if (!(iss >> particleNumber >> status)) {
										std::cout << "File parsing issue in: " << line << std::endl;
										logFile << "File parsing issue in: " << line << "\n";
									}
									else {
										// Output the particle number and its dropping status
										particleDropStatus.push_back(status == 'y' ? true : false);
										std::cout << "Particle " << particleNumber << " drops: " << (status == 'y' ? "Yes" : "No") << std::endl;
										logFile << particleNumber << " " << (status == 'y' ? "y" : "n") << "\n";
									}
								}
								file.close();
							}
						}
					}
				}

				// Correct amplitude values according to the particle dropping status
				bool noDropFlag = true;
				bool endSearchFlag = false;

				if (particleDropStatus.size() == numAgents) {
					if (!manualDataCollectionMode) {
						for (int i = 0; i < particleDropStatus.size(); i++) {
							if (particleDropStatus[i] == true) {
								noDropFlag = false;
								// if amplitude amendment is too large
								// we shift it to negative
								if (ampOffsets[i] >= 7000) {
									ampOffsets[i] = -500;
									std::cout << i << " amplitude directly assigned and amended to " << ampOffsets[i] << std::endl;
								}
								else if (ampOffsets[i] <= -1000) {
									std::cout << "Offsets value threshold reached, Rolling back to previous solution and return." << std::endl;
									logFile << i << ": amp " << ampOffsets[i] << ": " << "Offsets value threshold reached, Rolling back to previous solution and return." << "\n";

									endSearchFlag = true;
									break;
								}
								else if (ampOffsets[i] >= 0) {
									ampOffsets[i] += 1000;
									std::cout << i << " amplitude +1000 and amended to " << ampOffsets[i] << std::endl;
								}
								else if (ampOffsets[i] < 0) {
									ampOffsets[i] -= 500;
									std::cout << i << " amplitude -500 and amended to " << ampOffsets[i] << std::endl;
								}
							}

							// Amplitude Adjustments
							for (int g = 0; g < numGeometries; g++) {
								ampBuffer[g * numPoints + i] = 16000 + ampOffsets[i];   //20800 (30%)  19200 (20%)  12800(-20%)
							}

						}

						if (noDropFlag) {
							for (int i = 0; i < particleDropStatus.size(); i++) {
								previousUsableSolutionAmpOffsets.push_back(ampOffsets[i]);
								previousUsableSolutionVelocities.push_back(current_speedo);
								ampOffsets[i] = 0;
							}
							// If all not dropping, increase its velocity
							binarySearchLowerUpper = true;
							std::cout << "No Dropping Happened, We increase its velocity." << std::endl;
							logFile << "No Dropping Happened, We increase its velocity." << "\n";
						}

						if (endSearchFlag) {
							for (int i = 0; i < particleDropStatus.size(); i++) {
								previousUsableSolutionAmpOffsets.push_back(ampOffsets[i]);
								previousUsableSolutionVelocities.push_back(current_speedo);
								ampOffsets[i] = 0;
							}
							std::cout << "Search Ended. And Trying to Lower Velocities..." << std::endl;
							logFile << "Search Ended. And Trying to Lower Velocities..." << "\n";
							binarySearchHigherLower = true;
						}
					}
				}
				else {
					if (!manualMode && !guiManageMode) {
						std::cout << "*** Data Incomplete or in Manual Mode ***" << std::endl;
						std::cout << "Particle Drop Vector Size not correct due to incomplete data processing, please recollect the data." << std::endl;
						logFile << "DATA INCOMPLETE/Manual Mode: Particle Drop Vector Size not correct due to incomplete data processing, please recollect the data. [DI]" << "\n";
						std::string x;
						std::cout << "Please Manually Write the result, which particle(s) is/are dropped? (white space to separate), no drop please write 'no', otherwise will be deemed as skip.\n";

						// Get the input string from the user
						std::cout << "Enter the input string: ";
						std::getline(std::cin, x);

						// Check if the input is empty
						if (x.empty()) {
							std::cout << "Empty input, skipping processing." << std::endl;
							//return 0;
						}

						// Check if the input contains "no"
						else if (x.find("no") != std::string::npos) {
							std::cout << "No particle dropped declared." << std::endl;

							if (manualDataCollectionMode) {
								std::ofstream outFile;  // Create an ofstream object

								// Open the file for writing
								outFile.open(integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_c_pre_process_drop.txt");

								// Check if the file was opened successfully
								if (!outFile) {
									std::cerr << "Error opening file for writing." << std::endl;
								}
								else {
									// Write data to the file
									for (int l = 0; l < numAgents; l++) {
										outFile << l << " n" << std::endl;
									}
									outFile.close();
								}
							}

							if (!manualDataCollectionMode) {
								for (int i = 0; i < particleDropStatus.size(); i++) {
									previousUsableSolutionAmpOffsets.push_back(ampOffsets[i]);
									previousUsableSolutionVelocities.push_back(current_speedo);
									ampOffsets[i] = 0;
									std::cout << i << " amplitude's offset reset to " << ampOffsets[i] << std::endl;
								}
								// If all not dropping, increase its velocity
								binarySearchLowerUpper = true;
								std::cout << "No Dropping Happened, We increase its velocity." << std::endl;
								logFile << "No Dropping Happened, We increase its velocity." << "\n";
							}
							//return 0;
						}

						else {
							std::stringstream ss(x);
							std::string token;
							std::vector<int> numbers;

							while (ss >> token) {
								try {
									int num = std::stoi(token);  // Convert string token to int
									numbers.push_back(num);
								}
								catch (const std::invalid_argument& e) {
									std::cerr << "Invalid input: " << token << " is not a number." << std::endl;
								}
								catch (const std::out_of_range& e) {
									std::cerr << "Input out of range: " << token << std::endl;
								}
							}

							if (manualDataCollectionMode) {
								std::ofstream outFile;  // Create an ofstream object

								// Open the file for writing
								outFile.open(integratedDir + "/" + std::to_string(totalRecordings) + "_" + std::to_string(current_speedo) + "_c_pre_process_drop.txt");

								// Check if the file was opened successfully
								if (!outFile) {
									std::cerr << "Error opening file for writing." << std::endl;
								}
								else {
									// Write data to the file
									for (int l = 0; l < numAgents; l++) {
										bool captured = false;
										for (int kk = 0; kk < numbers.size(); kk++) {
											if (l == numbers[kk]) {
												captured = true;
												break;
											}
										}

										if (!captured) {
											outFile << l << " n" << std::endl;
										}
										else {
											outFile << l << " y" << std::endl;
										}
									}
									outFile.close();
								}
							}

							if (!manualDataCollectionMode) {
								for (int k = 0; k < numbers.size(); k++) {
									if (ampOffsets[numbers[k]] >= 7000) {
										ampOffsets[numbers[k]] = -500;
										std::cout << numbers[k] << " amplitude directly assigned and amended to " << ampOffsets[numbers[k]] << std::endl;
									}
									else if (ampOffsets[numbers[k]] <= -1000) {
										std::cout << "Offsets value threshold reached, Rolling back to previous solution and return." << std::endl;
										logFile << numbers[k] << ": amp " << ampOffsets[numbers[k]] << ": " << "Offsets value threshold reached, Rolling back to previous solution and return." << "\n";

										std::cout << "Search Ended. And Trying to Lower Velocities..." << std::endl;
										logFile << "Search Ended. And Trying to Lower Velocities..." << "\n";

										binarySearchHigherLower = true;
										break;
									}
									else if (ampOffsets[numbers[k]] >= 0) {
										ampOffsets[numbers[k]] += 1000;
										std::cout << numbers[k] << " amplitude +1000 and amended to " << ampOffsets[numbers[k]] << std::endl;
									}
									else if (ampOffsets[numbers[k]] < 0) {
										ampOffsets[numbers[k]] -= 500;
										std::cout << numbers[k] << " amplitude -500 and amended to " << ampOffsets[numbers[k]] << std::endl;
									}
								}
							}
						}
					}
				}

				//std::string tempFilePath = "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + std::to_string(numAgents) + "/" + testPattern + "/" + algorithmUsedd + "/" + std::to_string(current_speedo) + "/" + roundNumm;

				////use the ofstream. create file and write in and only write once!				
				//PositionFile << tempFilePath + "/trackingPositionFile.csv";

				//// Check if the file exists and delete it
				//if (std::ifstream(tempFilePath + "/trackingPositionFile.csv")) {
				//	if (std::remove((tempFilePath + "/trackingPositionFile.csv").c_str()) != 0) {
				//		std::cerr << "Error deleting file: " << tempFilePath + "/trackingPositionFile.csv" << std::endl;
				//	}
				//	else {
				//		std::cout << "Deleted (Reset) file: " << tempFilePath + "/trackingPositionFile.csv" << std::endl;
				//	}
				//}

				//f.open(tempFilePath + "/trackingPositionFile.csv");
				//if (!f.is_open()) {
				//	std::cerr << "Unable to open file: " << tempFilePath + "/trackingPositionFile.csv" << "\n" << std::endl;
				//	//return;
				//}
				//f << "time,";
				////write csv header
				//for (int i = 0; i < numPointsSec; i++)
				//{
				//	if (i == (numPointsSec - 1))
				//	{
				//		f << "x" << i << ",y" << i << ",z" << i << "," << std::endl;
				//	}
				//	else {
				//		f << "x" << i << ",y" << i << ",z" << i << ",";
				//	}
				//}

				//// write positions (target trap positions and real(optiTrack) positions)
				//// only write once, do not write previous data for current test
				//for (int i = 0; i < times.size()-1; i++)
				//{
				//	f << times[i] << ",";
				//	for (int n = 0; n < numPointsSec; n++)
				//	{
				//		if (realPositions[n][i].x > 990.0f) {
				//			f << "," << "," << ",";
				//		}
				//		else {
				//			f << realPositions[n][i].x << "," << realPositions[n][i].y << "," << realPositions[n][i].z << ",";
				//		}
				//		//f << trapPositions[n][i].x << "," << trapPositions[n][i].y << "," << trapPositions[n][i].z << ",";
				//		//printf("currTime: %f, currPos1: (%f,%f,%f), currPos2: (%f, %f, %f)\n",times[i],trapPositions[0][i].x, trapPositions[0][i].y, trapPositions[0][i].z, trapPositions[1][i].x, trapPositions[1][i].y, trapPositions[1][i].z);
				//	}
				//	f << std::endl;
				//}
				//f.close();

				//printf("------Finished position writing, clear previous data------\n");

				printf("------Finished, clear previous data------\n");
				//reset the position container
				times.clear();
				particleDropStatus.clear();
				//for (int i = 0; i < trapPositions.size(); i++)
				//{
				//	trapPositions[i].clear();
				//}
				//
				//for (int i = 0; i < realPositions.size(); i++) {
				//	realPositions[i].clear();
				//}

				startRecording = false;
				startTracking = false;
				totalRecordings++;
				continue;
			}

		}

		GSPAT::Solution* solution;
		if (!developmentMode) {
			// 5.4. Compute a hologram
			if (testMode) solution = solver->createSolution(numPoints, numGeometries, true, &(circlePath_data[pIndex]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
			else solution = solver->createSolution(numPoints, numGeometries, true, &(posBuffer[0]), &(ampBuffer[0]), mStarts, mEnds, GSPAT::RowMajorAlignment);
			solver->compute(solution);
		}
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
	}

	delete[] posBuffer, ampBuffer;
	return NULL;
}

std::string getCurrentTimeString() {
	// Get the current time
	std::time_t t = std::time(nullptr);
	std::tm tm = *std::localtime(&t);

	// Create a stringstream to format the time
	std::stringstream ss;
	ss << std::put_time(&tm, "%y%m%d_%H%M%S");

	// Return the formatted string
	return ss.str();
}

bool execute_with_timeout(std::function<bool()> func, int timeout_seconds) {
	auto future = std::async(std::launch::async, func);

	if (future.wait_for(std::chrono::seconds(timeout_seconds)) == std::future_status::timeout) {
		std::cout << "Task timed out, retrying..." << std::endl;
		return false;
	}
	else {
		return future.get();
	}
}

// Function to kill a process by its name
void KillProcessByName(const std::string& exeName) {
	std::string command = "taskkill /IM " + exeName + " /F";
	system(command.c_str());
}

// Function to start a process
void StartProcess(const std::string& exePath) {
	// Use ShellExecute to start the program
	ShellExecute(NULL, "open", exePath.c_str(), NULL, NULL, SW_SHOWNORMAL);
}

void* performSystemInfoWriting(void* arg) {
	while (true) {
		writeSystemInfoFile();
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}
}

void main() {
	GSPAT::Solver* solver;
	if (!developmentMode) {
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
		while (1) {
			try {
				a->connect(999, 1000);  //3,23  21,15    2,4   82, 80  
				break;
			}
			catch (...) {
				printf("Boards initialization failed, please retry.\n");
				std::this_thread::sleep_for(std::chrono::seconds(3));
			}
		}

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
		//printf("start discretization\n");
		for (int g = 0; g < numReadingGeometries; g++) {
			disc->discretizePhases(&(readingPhaseBuffer[g * numTransducers]), phases_disc);
			//printf("discretize %d geometries \n", g + 1);
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
		//printf("Got messages_fromReadingFiles after discretization\n");


		//STAGE 1: Initialize the solver
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
	}
	else {
		pthread_mutex_init(&solution_available, NULL);
		pthread_mutex_lock(&solution_available);
		pthread_mutex_init(&mutex_solution_queue, NULL);
		pthread_create(&readerThread, NULL, &solutionReaderThread, ((void*)solver));
		pthread_create(&writerThread, NULL, &solutionWriterThread, ((void*)solver));
	}

	if (developmentMode) {
		noOptitrackMode = true;
		std::cout << "***** DEVELOPMENT MODE ACTIVATED *****\n*** Under this mode, all Optitrack modules and PAT low level controls will be disabled ***" << std::endl;
	}

	//STAGE 3: Start the OptiTrack
	if (!noOptitrackMode) {
		while (1) {
			OptiTrack& Optitrack = OptiTrack::instance();
			if (Optitrack.InitOptiTrack() != 0) {
				printf("\nCould not connect to OptiTrack. Is it connected?\n");
			}
			else {
				break;
			}
		}
	}
	else {
		noCaptureMode = true;
		//manualMode = true;
		manualDataCollectionMode = true;
		manualInputMode = true;
		std::cout << "No Optitrack Mode Activated." << std::endl;
	}

	nameIntegration = std::to_string(numAgents) + "_" + testPattern + "_" + algorithmUsedd + "_" + std::to_string(current_speedo) + "_" + roundNumm + "_";
	currentTime = getCurrentTimeString();
	integratedDir = "C:/Users/weicheng/Desktop/formal_dataset/weicheng_0809/" + nameIntegration + currentTime;
	create_directories_if_not_exist(integratedDir);
	std::cout << nameIntegration + currentTime << std::endl;

	logFile.open(integratedDir + "/log.txt");

	// Initialise particles for the upper limit speed for finding
	particles_speed.clear();
	particles_pickup_speed.clear();
	for (int i = 0; i < numAgents; i++) {
		particles_speed.push_back(current_speedo);
		particles_pickup_speed.push_back(current_speedo);
		startSteps.push_back(0);
		endSteps.push_back(29999);
		ampOffsets.push_back(0);
		previousUsableSolutionVelocities.push_back(current_speedo);
		previousUsableSolutionAmpOffsets.push_back(0);
	}

	bool gothroughBNTraining = false;
	bool gothroughMLPTraining = false;

	while (running) {
		std::string instruction_txt = "C:/Users/weicheng/Desktop/ServerSampler/bin/instruction.txt";
		int instruction_number = 0;
		bool execute_instruction = false;
		bool empty_flag = false;

		if (exists_test(instruction_txt)) {
			std::ifstream file(instruction_txt);

			if (isFileEmpty(file)) {
				empty_flag = true;
			}
			else {
				empty_flag = false;
				std::string firstLine;
				std::getline(file, firstLine);
				std::istringstream iss(firstLine);
				int value;
				if (iss >> value) {
					//std::cout << "The first line is an integer: " << value << std::endl;
					if (value == 0) {
						guiManageMode = true;
						std::cout << "Entering GUI Remote Management Mode..." << std::endl;
						std::ofstream(feedback_txt) << "0";
						value = 23; // Write system info
					}
					else if (value == 1) {
						guiManageMode = false;
						std::cout << "Exited GUI Remote Management Mode..." << std::endl;
						std::ofstream(feedback_txt) << "0";
						deleteFile("C:/Users/weicheng/Desktop/ServerSampler/bin/system_info.txt");
					}
					instruction_number = value;
					execute_instruction = true;
				}
				else {
					std::cout << "Instruction Malformed, message ignored." << std::endl;
					std::ofstream(feedback_txt) << "2";
				}
			}

			file.close();

			if (!empty_flag) {
				deleteFile(instruction_txt);
			}
		}

		switch (instruction_number) {
		case 2:
			running = false; std::ofstream(feedback_txt) << "0"; break;
		case 22:
			running = false; gothroughBNTraining = true; std::ofstream(feedback_txt) << "0"; break;
		case 222:
			running = false; gothroughBNTraining = true; gothroughMLPTraining = true; std::ofstream(feedback_txt) << "0"; break;
		case 3:
			pickingup = true; std::ofstream(feedback_txt) << "0"; break;
		case 4:
			turnTransducersOff = true; std::ofstream(feedback_txt) << "0"; break;
		case 5:
			goToTestingPos = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 6:
			printf("*** Weicheng Simple Probability Analysis [FULL] ***\n");
			simpleAnalysis = true;
			simpleAnalysisFullMode = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 7:
			printf("*** Weicheng Simple Probability Analysis [FOR UNKNOWN DATA] ***\n");
			simpleAnalysis = true;
			simpleAnalysisFullMode = false;
			std::ofstream(feedback_txt) << "0";
			break;
		case 8:
			printf("*** Bayesian Standard Network Finder ***\n");
			printf("--- WAIT ---\n");
			bnAnalysis = true;
			bnAnalysisFullMode = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 9:
			printf("*** Bayesian Simpler Network Finder ***\n");
			printf("--- WAIT ---\n");
			bnAnalysis = true;
			bnAnalysisFullMode = false;
			std::ofstream(feedback_txt) << "0";
			break;
		case 10:
			printf("*** MLP Finder ***\n");
			mlpAnalysis = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 11:
			if (manualMode) {
				binarySearchHigherLower = true;
				std::ofstream(feedback_txt) << "0";
			}
			else {
				std::ofstream(feedback_txt) << "2";
			}
			break;
		case 12:
			if (manualMode) {
				binarySearchLowerUpper = true;
				std::ofstream(feedback_txt) << "0";
			}
			else {
				std::ofstream(feedback_txt) << "2";
			}
			break;
		case 13:
			if (!noOptitrackMode) {
				manualInputMode = !manualInputMode;
				std::cout << "Manual Input Mode now is: " << manualInputMode << std::endl;
				std::ofstream(feedback_txt) << "0";
			}
			else {
				std::ofstream(feedback_txt) << "2";
			}
			break;
		case 14:
			if (!noOptitrackMode) {
				manualDataCollectionMode = !manualDataCollectionMode;
				manualInputMode = !manualInputMode;
				std::cout << "Manual Data Collection Mode now is: " << manualDataCollectionMode << std::endl;
				std::cout << "Manual Input Mode now is: " << manualInputMode << std::endl;
				std::ofstream(feedback_txt) << "0";
			}
			else {
				std::ofstream(feedback_txt) << "2";
			}
			break;
		case 15:
			for (int i = 0; i < numAgents; i++) {
				ampOffsets[i] += 1000;
			}
			std::ofstream(amp_txt) << 16000 + ampOffsets[0];
			std::cout << "Amp Offset Added by 1000, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
			std::ofstream(feedback_txt) << "0";
			break;
		case 16:
			for (int i = 0; i < numAgents; i++) {
				ampOffsets[i] -= 1000;
			}
			std::ofstream(amp_txt) << 16000 + ampOffsets[0];
			std::cout << "Amp Offset Reduced by 1000, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
			std::ofstream(feedback_txt) << "0";
			break;
		case 17:
			for (int i = 0; i < numAgents; i++) {
				ampOffsets[i] += 100;
			}
			std::ofstream(amp_txt) << 16000 + ampOffsets[0];
			std::cout << "Amp Offset Added by 100, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
			std::ofstream(feedback_txt) << "0";
			break;
		case 18:
			for (int i = 0; i < numAgents; i++) {
				ampOffsets[i] -= 100;
			}
			std::ofstream(amp_txt) << 16000 + ampOffsets[0];
			std::cout << "Amp Offset Reduced by 100, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
			std::ofstream(feedback_txt) << "0";
			break;
		case 19:
			manualSpeedReduction = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 20:
			manualSpeedIncrease = true;
			std::ofstream(feedback_txt) << "0";
			break;
		case 21:
			if (!noOptitrackMode) {
				noCaptureMode = !noCaptureMode;
				std::cout << "No Capture Mode Switch, if enabled, observe the particle movements manually without cameras." << " Status: " << noCaptureMode << std::endl;
				std::ofstream(feedback_txt) << "0";
			}
			else {
				std::ofstream(feedback_txt) << "2";
			}
			break;
		case 23:
			// System state reporting request
			writeSystemInfoFile();
			break;
		default:
			break;
		}

		if (instruction_number != 0) {
			instruction_number = 0;
		}

		writeSystemInfoFile();

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
				//case '1': // Calculate which particle's speed should be reduced
				//	speed_reduce_calc = true; break;
				//case '2': // Calculate which particle's speed should be increased
				//	speed_increase_calc = true; break;
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
				goToTestingPos = true;
				break;
			case 'k':
				printf("*** Weicheng Simple Probability Analysis [FULL] ***\n");
				simpleAnalysis = true;
				simpleAnalysisFullMode = true;
				break;
			case 'j':
				printf("*** Weicheng Simple Probability Analysis [FOR UNKNOWN DATA] ***\n");
				simpleAnalysis = true;
				simpleAnalysisFullMode = false;
				break;
			case 'b':
				printf("*** Bayesian Standard Network Finder ***\n");
				printf("--- WAIT ---\n");
				bnAnalysis = true;
				bnAnalysisFullMode = true;
				break;
			case 'n':
				printf("*** Bayesian Simpler Network Finder ***\n");
				printf("--- WAIT ---\n");
				bnAnalysis = true;
				bnAnalysisFullMode = false;
				break;
			case 'q':
				printf("*** MLP Finder ***\n");
				mlpAnalysis = true;
				break;
			case 'l':
				if (manualMode) {
					binarySearchHigherLower = true;
				}
				break;
			case 'h':
				if (manualMode) {
					binarySearchLowerUpper = true;
				}
				break;
			case 'i':
				if (!noOptitrackMode) {
					manualInputMode = !manualInputMode;
					std::cout << "Manual Input Mode now is: " << manualInputMode << std::endl;
				}
				break;
			case 'm':
				if (!noOptitrackMode) {
					manualDataCollectionMode = !manualDataCollectionMode;
					manualInputMode = !manualInputMode;
					std::cout << "Manual Data Collection Mode now is: " << manualDataCollectionMode << std::endl;
					std::cout << "Manual Input Mode now is: " << manualInputMode << std::endl;
				}
				break;
			case '5':
				for (int i = 0; i < numAgents; i++) {
					ampOffsets[i] += 1000;
				}
				std::cout << "Amp Offset Added by 1000, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
				break;
			case '6':
				for (int i = 0; i < numAgents; i++) {
					ampOffsets[i] -= 1000;
				}
				std::cout << "Amp Offset Reduced by 1000, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
				break;
			case '7':
				for (int i = 0; i < numAgents; i++) {
					ampOffsets[i] += 100;
				}
				std::cout << "Amp Offset Added by 100, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
				break;
			case '8':
				for (int i = 0; i < numAgents; i++) {
					ampOffsets[i] -= 100;
				}
				std::cout << "Amp Offset Reduced by 100, now is " << ampOffsets[0] << ", total " << 16000 + ampOffsets[0] << std::endl;
				break;
			case 'r':
				manualSpeedReduction = true;
				break;
			case 'a':
				manualSpeedIncrease = true;
				break;
			case 'c':
				if (!noOptitrackMode) {
					noCaptureMode = !noCaptureMode;
					std::cout << "No Capture Mode Switch, if enabled, observe the particle movements manually without cameras." << " Status: " << noCaptureMode << std::endl;
				}
				break;
			case 'g':
				std::cout << "Path Planning This Path in 2D..." << std::endl;
				pickingup_2d = true;
				break;
			case 'f':
				goToTestingPos_2d = true;
				break;
			case 't':
				usePath1 = !usePath1;
				std::cout << "Switched Path Planner Mode to: " << usePath1 << ". (1 uses S2M2, 0 uses CBS)" << std::endl;
				break;
			default:
				break;
			}
		}

	}

	std::string addResultToBN = "";
	if (!guiManageMode) {
		std::cout << "Do you want to add this batch of result as evidence for training the Bayesian Network? Type 'y' or 'yes' to add.\n";
		std::cin >> addResultToBN;
	}
	else {
		if (gothroughBNTraining) {
			addResultToBN = "y";
		}
	}

	bool BNTrained = false;

	if (addResultToBN == "y" || addResultToBN == "yes") {
		// Open file in append mode
		std::ofstream bnTFile("C:/Users/weicheng/Desktop/ServerSampler/bin/bnTrainerFolders.txt", std::ios_base::app);

		if (!bnTFile.is_open()) {
			std::cerr << "Failed to open the file for appending training request!" << std::endl;
		}
		else {
			// Write to the file
			bnTFile << "\n" << integratedDir + "/"; // Replace with your actual directory variable if needed
			bnTFile.close();

			std::cout << "Please wait while new evidence is adding and model retraining...\n";

			const char* executablePath1 = "python ";
			std::string argument1 = "c:/Users/weicheng/Desktop/formal_dataset/bn_evidence_adder.py";
			std::string argument2 = "--num_particles " + std::to_string(numAgents);
			std::string command = executablePath1 + argument1 + " " + argument2;

			bool result = std::system(command.c_str());
			// Check the result
			if (result == 0) {
				std::cout << "BN model evidence added successfully." << std::endl;

				const char* executablePath1 = "python ";
				argument1 = "c:/Users/weicheng/Desktop/formal_dataset/bn_inference_v2.py";
				argument2 = "--num_particles " + std::to_string(numAgents);
				std::string argument3 = "--mode train";
				std::string argument4 = "--data_file \"C:/Users/weicheng/Desktop/formal_dataset/training_data_" + std::to_string(numAgents) + "_particles.txt\" --force_train";
				command = executablePath1 + argument1 + " " + argument2 + " " + argument3 + " " + argument4;

				result = std::system(command.c_str());
				// Check the result
				if (result == 0) {
					std::cout << "BN model retrain finished successfully." << std::endl;
				}
				else {
					std::cerr << "BN model retrain finished with errors. Return code: " << result << std::endl;
					logFile << "BN model retrain finished with errors. Return code: " << result << "\n";
				}

				std::cout << "Training BN Alt Model (Simple Drop Status)" << std::endl;
				const char* executablePath2 = "python ";
				argument1 = "c:/Users/weicheng/Desktop/formal_dataset/bn_inference_v2_alt.py";
				argument2 = "--num_particles " + std::to_string(numAgents);
				argument3 = "--mode train";
				argument4 = "--data_file \"C:/Users/weicheng/Desktop/formal_dataset/training_data_alt_" + std::to_string(numAgents) + "_particles.txt\" --force_train";
				command = executablePath2 + argument1 + " " + argument2 + " " + argument3 + " " + argument4;

				result = std::system(command.c_str());
				// Check the result
				if (result == 0) {
					std::cout << "BN simple model retrain finished successfully." << std::endl;
					BNTrained = true;
				}
				else {
					std::cerr << "BN simple model retrain finished with errors. Return code: " << result << std::endl;
					logFile << "BN simple model retrain finished with errors. Return code: " << result << "\n";
				}

			}
			else {
				std::cerr << "BN model evidence adding finished with errors. Return code: " << result << std::endl;
				logFile << "BN model evidence adding finished with errors. Return code: " << result << "\n";
			}
		}
	}

	if (BNTrained) {
		std::string addResultToMLP = "";
		if (!guiManageMode) {
			std::cout << "Do you also want to retrain the Multi-Layer Perceptron (MLP) model (Dataset already updated by using previous BN evidence adder)? Type 'y' or 'yes' to add.\n";
			std::cin >> addResultToMLP;
		}
		else {
			if (gothroughMLPTraining) {
				addResultToMLP = "y";
			}
		}

		if (addResultToMLP == "y" || addResultToMLP == "yes") {
			std::cout << "Please wait while model retraining...\n";

			const char* executablePath3 = "python ";

			std::string argument1 = "c:/Users/weicheng/Desktop/formal_dataset/mlp_backward.py";
			std::string argument2 = "--numParticles " + std::to_string(numAgents);
			std::string argument3 = "--data_file \"C:/Users/weicheng/Desktop/formal_dataset/training_data_alt_" + std::to_string(numAgents) + "_particles.txt\"";
			std::string argument7 = "train";
			std::string command = executablePath3 + argument1 + " " + argument2 + " " + argument3 + " " + argument7;

			bool result = std::system(command.c_str());
			bool result_success_flag = false;
			// Check the result
			if (result == 0) {
				std::cout << "MLP Training finished successfully." << std::endl;
				result_success_flag = true;
			}
			else {
				std::cerr << "MLP Training finished with errors. Return code: " << result << std::endl;
				logFile << "MLP Training finished with errors. Return code: " << result << "\n";
			}

		}
	}

	logFile.close();
	Sleep(2000);
	if (!developmentMode) {
		a->disconnect();
		delete a;
		delete solver;
	}
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
	if (!developmentMode) {
		if (transducersOff) a->turnTransducersOff();
		else a->updateMessages(messages, numGeometries);
	}

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

std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, std::vector<float> velocities, bool taskAssignment, bool targetConstraint) {
	int numAgents = starts.size();
	std::vector<PathPlanner::AgentInfo> agents;
	for (int i = 0; i < numAgents; i++) {
		PathPlanner::AgentInfo agent(starts[i], targets[i], size, velocities[i]);
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
	static const int minThres = 0, maxThres = 180, thresStep = 1;
	int threshold = minThres;
	bool corrected = false;
	//int maxTry = 999;
	//int currentTry = 0;

	std::vector<glm::vec3> detectedPositions(numPoints);
	while (!corrected) {
		detector.setThreshold(threshold);
		std::vector<cv::Point3d> currentBeads = detector.getCurrentBeads();
		if (currentBeads.size() == numPoints) {
			corrected = true;
			for (int p = 0; p < numPoints; p++)
				detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
		}
		if (threshold == maxThres) {
			//if (currentTry >= maxTry) {
			//	std::cout << "*** Try finding particles reached the max threshold ***" << std::endl;
			//	break;
			//}
			//else {
			threshold = minThres;
			//currentTry++;
		//}
		}
		else {
			threshold += thresStep;
		}
	}
	return detectedPositions;
}

std::vector<glm::vec3> getDetectedPositions(OptiTrack& optitrack, int numPoints) {
	std::vector<glm::vec3> detectedPositions(numPoints);

	std::vector<glm::vec3> currentBeads = optitrack.getDetectedPositions();

	for (int p = 0; p < numPoints; p++) {
		detectedPositions[p] = glm::vec3(currentBeads[p].x, currentBeads[p].y, currentBeads[p].z); // in S2M2, we consider z=0 at our centre height (12cm above the bottom board)
		printf("Detected %d: pos = (%f, %f, %f)\n", p, detectedPositions[p].x, detectedPositions[p].y, detectedPositions[p].z);
	}

	return detectedPositions;
}

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

void getStartEndPos_StableLev_62()
{
	initialPositions.clear();
	targetPositions.clear();

	initialPositions.push_back(glm::vec3(-0.00327422, -0.0505429, 0.0654366));
	initialPositions.push_back(glm::vec3(0.0493298, 0.0125002, 0.138103));
	initialPositions.push_back(glm::vec3(0.0270057, 0.0256784, 0.100192));
	initialPositions.push_back(glm::vec3(0.00719867, -0.0346278, 0.0910358));
	initialPositions.push_back(glm::vec3(-0.0113074, -0.0197914, 0.152751));
	initialPositions.push_back(glm::vec3(-0.051507, 0.0415672, 0.107347));


	targetPositions.push_back(glm::vec3(-0.042831, -0.0398422, 0.171935));
	targetPositions.push_back(glm::vec3(0.021193, -0.00851853, 0.129375));
	targetPositions.push_back(glm::vec3(0.0115901, 0.0393897, 0.117195));
	targetPositions.push_back(glm::vec3(-0.0390032, 0.0429634, 0.155577));
	targetPositions.push_back(glm::vec3(-0.0221276, -0.0096436, 0.0656258));
	targetPositions.push_back(glm::vec3(0.0225266, -0.0314456, 0.160557));
}


void getStartEndPos_StableLev_62_plain_s1()
{
	initialPositions.clear();
	targetPositions.clear();

	initialPositions.push_back(glm::vec3(-0.00327422, -0.0505429, 0.09));
	initialPositions.push_back(glm::vec3(0.0493298, 0.0125002, 0.09));
	initialPositions.push_back(glm::vec3(0.0270057, 0.0256784, 0.09));
	initialPositions.push_back(glm::vec3(0.00719867, -0.0346278, 0.09));
	initialPositions.push_back(glm::vec3(-0.0113074, -0.0197914, 0.09));
	initialPositions.push_back(glm::vec3(-0.051507, 0.0415672, 0.09));

	targetPositions.push_back(glm::vec3(-0.00327422, -0.0505429, 0.0654366));
	targetPositions.push_back(glm::vec3(0.0493298, 0.0125002, 0.138103));
	targetPositions.push_back(glm::vec3(0.0270057, 0.0256784, 0.100192));
	targetPositions.push_back(glm::vec3(0.00719867, -0.0346278, 0.0910358));
	targetPositions.push_back(glm::vec3(-0.0113074, -0.0197914, 0.152751));
	targetPositions.push_back(glm::vec3(-0.051507, 0.0415672, 0.107347));

	//targetPositions.push_back(glm::vec3(-0.042831, -0.0398422, 0.171935));
	//targetPositions.push_back(glm::vec3(0.021193, -0.00851853, 0.129375));
	//targetPositions.push_back(glm::vec3(0.0115901, 0.0393897, 0.117195));
	//targetPositions.push_back(glm::vec3(-0.0390032, 0.0429634, 0.155577));
	//targetPositions.push_back(glm::vec3(-0.0221276, -0.0096436, 0.0656258));
	//targetPositions.push_back(glm::vec3(0.0225266, -0.0314456, 0.160557));
}