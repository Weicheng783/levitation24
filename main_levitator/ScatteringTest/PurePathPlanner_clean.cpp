#include <S2M2/cpp/PathPlanner.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <cmath>
#include <conio.h>
#include <map>

float centreHeight = 0.12f;

std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment = false, bool targetConstraint = false);
std::vector<glm::vec3> getRandomZPositions(int numPoints, std::vector<glm::vec3> previousPositions, float zMin, float zMax);

std::vector<glm::vec3> getEqualHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float height);
std::vector<glm::vec3> getHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName);
std::vector<glm::vec3> updateHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName);
std::vector<glm::vec3> getFilteredPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset, float zPosOthers);
std::vector<glm::vec3> getFilteredAndSortedPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset);
std::vector<glm::vec3> readMultiPos(std::string fileName, int numPos);
std::vector<glm::vec3> getRandomPositions(int numPoints, float size = 0.0075f, float zSize = 0.015f, glm::vec3 boundaryMin = glm::vec3(-0.06, -0.06, -0.06 + centreHeight), glm::vec3 boundaryMax = glm::vec3(+0.06, +0.06, +0.06 + centreHeight));  // z pos + centreHeight
void otherPaths();
void getStartEndPos_Giorgos();
void getStartEndPos_Weicheng();
void getStartEndPos_One();
void getStartEndPos_Crossing_four();
void getStartEndPos_Stacking_four();
void getStartEndPos_Crossing_three();
void getStartEndPos_Crossing_six();

glm::vec3 boundaryMin(-0.06, -0.06, -0.06+centreHeight);
glm::vec3 boundaryMax(+0.06, +0.06, +0.06+centreHeight);
glm::vec3 size(0.007, 0.007, 0.015);	// size of the agents in meter

// define meta information
int numAgents = 6;
std::string numAgentss = "6";
//int roundNum = 3;
std::string roundNumm = "1";
std::string algorithmUsed = "gs-pat";
std::string testPattern = "crossing";
std::string systemVersion = "240702";
float velocity = 0.01f;					// maximum velocity in meter/second
float velocity_low = 0.01f;					// minimum testing velocity in meter/second
float velocity_high = 0.10f;					// maximum testing velocity in meter/second

std::vector<glm::vec3> initialPositions, targetPositions;
std::string folderName = "weicheng_exp";
std::string TestName = "";
std::stringstream PathFile;       //store the paths for every test
std::stringstream TrajectoryFile;  //store the trajectories for every test

std::string fullPathPre = "C:/Users/weicheng/Desktop/formal_dataset/" + systemVersion + "/" + numAgentss + "/" + testPattern + "/" + algorithmUsed + "/";

// END OF META INFORMATION

std::map<int, std::string> enumMap = {
	{1, "0.01"},
	{2, "0.02"},
	{3, "0.03"},
	{4, "0.04"},
	{5, "0.05"},
	{6, "0.06"},
	{7, "0.07"},
	{8, "0.08"},
	{9, "0.09"},
	{10, "0.10"},
	{50, "0.50"},
	{100, "1.00"}
};

//float r1 = 0.03f;    //circle radius
//float r2 = 0.02f;    //cube radius

void main()
{
	PathPlanner planner("PathPlanner");
	planner.setBoundary(boundaryMin, boundaryMax);
	std::vector<AgentPath> paths;

	for (int i = velocity_low * 100; i <= velocity_high * 100; i++) {
		PathFile.str("");
		initialPositions.clear();
		targetPositions.clear();

		velocity = i * 0.01;

		getStartEndPos_Crossing_six();
		//getStartEndPos_One();
		std::vector<AgentPath> goPaths = setPaths(planner, initialPositions, targetPositions, size, velocity, false, true);
		//std::vector<AgentPath> returnPaths = setPaths(planner, targetPositions, initialPositions, size, velocity, false, false);

		//save path
		PathFile << fullPathPre << enumMap[i] << "/" << roundNumm << "/pathFile.csv";
		std::cout << PathFile.str() << std::endl;
		PathPlanner::saveAgentPaths(PathFile.str(), goPaths);

		std::cout << "\nSaved to: " << fullPathPre << enumMap[i] << "/" << roundNumm << "/pathFile.csv";
	}

	//PathFile.str(""); //renew the string
	//PathFile << fullPathPre << "/" << TestName << "-2" << "-v-" << velocity << "-" << "pathFile.csv";
	//PathPlanner::saveAgentPaths(PathFile.str(), returnPaths);

	/*TrajectoryFile << "E:/PythonProjects/BasicTest/LevitationStability/Files/6Agent/NAIVE/" << std::to_string(numAgents) << "/" << algorithmUsed << "_" << std::to_string(numAgents) << "-maxV-" << velocity << "-round-" << roundNum << "-TargetPosition.csv";
	paths=planner.readAgentTrajectories(TrajectoryFile.str(),numAgents);*/

	//for (int i = 91; i < 101; i++) {

	//	initialPositions = getRandomPositions(numAgents);
	//	targetPositions = getRandomPositions(numAgents);

	//	std::vector<AgentPath> goPaths = setPaths(planner, initialPositions, targetPositions, size, velocity, true, false);
	//	//std::vector<AgentPath> returnPaths = setPaths(planner, targetPositions, initialPositions, size, velocity, false, false); "agent-6-maxV-0.1-round-" + roundNum + "-pathFile.csv";

	//	//PathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output/PathSaving" << "/" << TestName << "-1" << "-v-" << velocity << "-" << "pathFile.csv";
	//	PathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output/PathSaving/" << std::to_string(numAgents)<< "Agents/agent-" << std::to_string(numAgents) << "-maxV-" << velocity << "-round-" << i << "-pathFile.csv";
	//	PathPlanner::saveAgentPaths(PathFile.str(), goPaths);
	//	PathFile.str(""); //renew the string
	//	
	//}
	//initialPositions = getRandomPositions(numAgents);
	//targetPositions = getRandomPositions(numAgents);

	//std::vector<AgentPath> goPaths = setPaths(planner, initialPositions, targetPositions, size, velocity, true, false);
	////std::vector<AgentPath> returnPaths = setPaths(planner, targetPositions, initialPositions, size, velocity, false, false); "agent-6-maxV-0.1-round-" + roundNum + "-pathFile.csv";

	////PathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output/PathSaving" << "/" << TestName << "-1" << "-v-" << velocity << "-" << "pathFile.csv";
	//PathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output/PathSaving" << "/agent-" << std::to_string(numAgents) << "-maxV-" << velocity << "-round-"<<roundNum <<"-pathFile.csv";
	//PathPlanner::saveAgentPaths(PathFile.str(), goPaths);
	//PathFile.str(""); //renew the string
	//PathFile << "D:/UCLProjects/PhDProjects/GiorgosProject/SourceCode/OpenMPD_Demo/Output/PathSaving" << "/" << TestName << "-2" << "-v-" << velocity << "-" << "pathFile.csv";
	//PathPlanner::saveAgentPaths(PathFile.str(), returnPaths);

	//planner.readAgentPaths(PathFile.str());

	std::cin.get();
}

void getStartEndPos_Giorgos()
{
	initialPositions.push_back(glm::vec3(0.04, 0, 0.12));                       //1
	initialPositions.push_back(glm::vec3(0.03236068, 0.02351141, 0.12));        //2
	initialPositions.push_back(glm::vec3(0.01236068, 0.038042261, 0.12));       //3
	initialPositions.push_back(glm::vec3(-0.01236068, 0.038042261, 0.12));      //4
	initialPositions.push_back(glm::vec3(-0.03236068, 0.02351141, 0.12));       //5
	initialPositions.push_back(glm::vec3(-0.04, 0, 0.12));                      //6
	initialPositions.push_back(glm::vec3(-0.03236068, -0.02351141, 0.12));      //7
	initialPositions.push_back(glm::vec3(-0.01236068, -0.038042261, 0.12));     //8
	initialPositions.push_back(glm::vec3(0.01236068, -0.038042261, 0.12));   //9
	initialPositions.push_back(glm::vec3(0.03236068, -0.02351141, 0.12));    //10
	
	targetPositions.push_back(glm::vec3(-0.009669303, -0.034582384, 0.178223064));  //1
	targetPositions.push_back(glm::vec3(-0.013558558, -0.038258866, 0.133402889));  //2
	targetPositions.push_back(glm::vec3(-0.047038032, -0.035064122, 0.144888774));  //3
	targetPositions.push_back(glm::vec3(-0.052029108, -0.032233512, 0.118381098));  //4
	targetPositions.push_back(glm::vec3(-0.02396594, -0.037971268, 0.093706592));  //5
	targetPositions.push_back(glm::vec3(-0.022966307, -0.036602049, 0.060704586));  //6
	targetPositions.push_back(glm::vec3(0.018534741, -0.040741054, 0.145650211));  //7
	targetPositions.push_back(glm::vec3(0.030871533, -0.040119852, 0.118334904));  //8
	targetPositions.push_back(glm::vec3(0.005221928, -0.034211613, 0.096596081));  //9
	targetPositions.push_back(glm::vec3(0.008916856, -0.036588336, 0.063191506));  //10
}

void getStartEndPos_Weicheng()
{
	// 2 particles cross each other
	// 1 goes left  1 goes right
	initialPositions.push_back(glm::vec3(0, 0.02f, 0.14f)); initialPositions.push_back(glm::vec3(-0.02f, 0.02f, 0.17f));

	targetPositions.push_back(glm::vec3(0, -0.02f, 0.17f)); targetPositions.push_back(glm::vec3(-0.02f, -0.02f, 0.14f));

}

void getStartEndPos_One()
{
	// individual particle experiment
	// Goto random position
	initialPositions = getRandomPositions(numAgents);
	targetPositions = getRandomPositions(numAgents);
}

void getStartEndPos_Crossing_four()
{
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

void getStartEndPos_Crossing_three()
{
	// 3 particles crossing each other from a height
	initialPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f)); // A
	initialPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
	//initialPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
	initialPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));

	targetPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.00f, 0.12f));
	//targetPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f));
	targetPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f));
}

void getStartEndPos_Stacking_four()
{
	// 4 particles crossing each other from a height
	initialPositions.push_back(glm::vec3(-0.04f, -0.04f, 0.12f)); // A
	initialPositions.push_back(glm::vec3(0.04f, 0.04f, 0.12f)); // B
	initialPositions.push_back(glm::vec3(-0.04f, 0.04f, 0.12f)); // C
	initialPositions.push_back(glm::vec3(0.04f, -0.04f, 0.12f)); // D

	//targetPositions.push_back(glm::vec3(0.05f, -0.05f, 0.06f));

	targetPositions.push_back(glm::vec3(0.01f, 0.00f, 0.06f));
	targetPositions.push_back(glm::vec3(-0.01f, -0.00f, 0.10f));
	targetPositions.push_back(glm::vec3(0.01f, -0.00f, 0.14f));
	targetPositions.push_back(glm::vec3(-0.01f, 0.00f, 0.17f));
}

std::vector<AgentPath> setPaths(PathPlanner& planner, std::vector<glm::vec3> starts, std::vector<glm::vec3> targets, glm::vec3 size, float velocity, bool taskAssignment, bool targetConstraint) {
	int numAgents = starts.size();
	std::vector<PathPlanner::AgentInfo> agents;

	std::vector<float> agent_speed = {velocity, velocity, velocity, velocity, velocity, velocity};

	for (int i = 0; i < numAgents; i++) {
		PathPlanner::AgentInfo agent(starts[i], targets[i], size, agent_speed[i]);
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
std::vector<glm::vec3> getRandomZPositions(int numPoints, std::vector<glm::vec3> previousPositions, float zMin, float zMax)
{
	std::random_device rd;
	std::default_random_engine eng(rd());
	//std::uniform_real_distribution<float> get_rand_x(boundaryMin.x, boundaryMax.x);
	//std::uniform_real_distribution<float> get_rand_y(boundaryMin.y, boundaryMax.y);
	std::uniform_real_distribution<float> get_rand_z(zMin, zMax);


	std::vector<glm::vec3> positions;


	for (int i = 0; i < previousPositions.size(); i++)
	{
		float x = previousPositions[i].x;
		float y = previousPositions[i].y;
		float z = get_rand_z(eng);
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		//std::cout << "get equal height positions:" << i << " x：" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;
}

std::vector<glm::vec3> getHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName)
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> readPos = readMultiPos(fileName, numPoints);

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = readPos[i].z;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "get positions:" << i << " x：" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}


std::vector<glm::vec3> getEqualHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float height)
{
	std::vector<glm::vec3> positions;

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = height;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "get equal height positions:" << i << " x：" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}

std::vector<glm::vec3> updateHeightPositions(int numPoints, std::vector<glm::vec3> prePositions, float boundaryBottom, float boundaryTop, std::string fileName)
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> readPos = readMultiPos(fileName, numPoints);

	for (int i = 0; i < prePositions.size(); i++)
	{
		float x = prePositions[i].x;
		float y = prePositions[i].y;
		float z = readPos[i].z;
		glm::vec3 newPosition(x, y, z);
		positions.push_back(newPosition);

		std::cout << "updated positions:" << i << " x：" << x << ", y:" << y << ", z:" << z << std::endl;
	}
	return positions;

}

std::vector<glm::vec3> getFilteredPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset, float zPosOthers)
{
	std::vector<glm::vec3> positions;
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{

			if (j == filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x - xOffset, prePositions[i * column + j].y, prePositions[i * column + j].z));
				std::cout << "not change index  " << i * column + j << " x: " << prePositions[i * column + j].x - xOffset << " y:" << prePositions[i * column + j].y << " z: " << prePositions[i * column + j].z << std::endl;
			}
			if (j < filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x + xOffset + 0.01f * i, prePositions[i * column + j].y + yOffset, zPosOthers));
				std::cout << "change left index" << i * column + j << " x: " << prePositions[i * column + j].x + xOffset + 0.01f * i << " y:" << prePositions[i * column + j].y << " z: " << zPosOthers << std::endl;
			}
			if (j > filteredColume)
			{
				positions.push_back(glm::vec3(prePositions[i * column + j].x + xOffset + 0.01f * i, prePositions[i * column + j].y - yOffset, zPosOthers));
				std::cout << "change right index" << i * column + j << " x: " << prePositions[i * column + j].x + xOffset + 0.01f * i << " y:" << prePositions[i * column + j].y << " z: " << zPosOthers << std::endl;
			}
		}
	}
	return positions;
}

std::vector<glm::vec3> getFilteredAndSortedPositions(int numPoints, std::vector<glm::vec3> prePositions, int row, int column, int filteredColume, float xOffset, float yOffset)
{
	// get the filtered positions and then change their order based on the heights, assign them to new positions
	std::vector<glm::vec3> filteredPositions;  //get the specific column positions
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (j == filteredColume)
			{
				filteredPositions.push_back(glm::vec3(prePositions[i * column + j].x, prePositions[i * column + j].y, prePositions[i * column + j].z));
				std::cout << "not change index" << i * column + j << std::endl;
			}
		}
	}
	std::vector<std::pair<float, int>> filteredHeight;

	//save the order
	std::vector<int> sortedOrder;

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		filteredHeight.push_back(std::make_pair(filteredPositions[i].z, i));
	}
	std::sort(filteredHeight.begin(), filteredHeight.end());

	std::cout << "Sorted, ascending order \n";
	for (int i = 0; i < filteredPositions.size(); i++)
	{
		std::cout << "height:" << filteredHeight[i].first << " " << "original index: " << filteredHeight[i].second << std::endl;   // outPout the sorted height and oringianl index
	}

	std::vector<std::pair<int, int>> IndexOrderPair;   // with correct index
	int order[] = { 0,0,0,0,0 };

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		IndexOrderPair.push_back(std::make_pair(filteredHeight[i].second, i));
		std::cout << "oringinal index:" << filteredHeight[i].second << " order : " << i << std::endl;
	}
	//save the order
	for (int i = 0; i < filteredPositions.size(); i++)
	{
		//[IndexOrderPair[i].first  get oringinal index
		order[filteredHeight[i].second] = i;		// get the order is important
	}

	std::vector<glm::vec3> sortedPositions;  // sort the filtered column

	for (int i = 0; i < filteredPositions.size(); i++)
	{
		std::cout << "order : " << order[i] << std::endl;
		sortedPositions.push_back(glm::vec3(-0.04f, -0.04f + order[i] * 0.04f, filteredPositions[i].z));
	}
	std::vector<glm::vec3> returnedPositions;   //final returned positions 

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (j == filteredColume)
			{
				returnedPositions.push_back(sortedPositions[i]);
				std::cout << "re-new positions: " << sortedPositions[i].x << ", " << sortedPositions[i].y << ", " << sortedPositions[i].z << std::endl;
			}
			else
				returnedPositions.push_back(prePositions[i * column + j]);
		}
	}

	return returnedPositions;
}



std::vector<glm::vec3> readMultiPos(std::string fileName, int numPos)
{
	std::vector<glm::vec3> positions;
	int lineNum = 0;
	// 1. Read a file and store it
	std::ifstream ifs(fileName);
	if (ifs.is_open())
	{
		std::string fileLine;
		getline(ifs, fileLine);     // skip the first line
		while (getline(ifs, fileLine)) // parse every line
		{
			std::stringstream toParse(fileLine); //stringstream of the line

			std::vector<std::string> lineTokens;  // store every token in the line
			std::string token;
			while (std::getline(toParse >> std::ws, token, ','))  // input, output, delimeter,  >> std::ws: means extract white space
			{
				lineTokens.push_back(token);
				//std::cout << "token: " << token << std::endl;
			}
			if (!lineTokens.empty())
			{
				float x = 0;  // convert string type to number type
				float y = 0;  // convert string type to number type
				float z = atof(lineTokens[0].c_str());  // convert string type to number type
				//std::cout << "float x: " << x <<"float y: "<< y <<std::endl;
				positions.push_back(glm::vec3(x, y, z));

			}
			lineNum += 1;
		}
		std::cout << numPos << std::endl;
		std::cout << lineNum << std::endl;
		if ((lineNum) == numPos)
			std::cout << "good match for num of claimed and generated  " << std::endl;
		else
			std::cout << "no match for num of claimed and generated  " << std::endl;
	}
	else {
		std::cout << "ERROR: File " << fileName << "cannot be opened!" << std::endl;
	}
	// 2. print what we have read
	//for (int i = 0; i < lineNum; i++)
	//{
	//	//printf("line: %i ", i);

	//	//printf(" pos %i : (%f, %f)", i, positions[i].x, positions[i].y);
	//	//std::cout << "pos " << i << ": "<<positions[i].x <<","<<positions[i].y<< std::endl;

	//}
	return positions;
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

//void otherPaths()
//{
//
//	//Only one particle move up and down --testName:6p
//	for (int i = 0; i < numAgents - 1; i++)
//	{
//		starts.push_back(glm::vec3(r * cos(i * 2.f * M_PI / (float)(numAgents-1)), r * sin(i * 2.f * M_PI / (float)(numAgents-1)), 0));
//	}
//	starts.push_back(glm::vec3(0, 0, 0));
//
//	for (int i = 0; i < numAgents - 1; i++)
//	{
//		targets.push_back(glm::vec3(r * cos(i * 2.f * M_PI / (float)(numAgents - 1)), r * sin(i * 2.f * M_PI / (float)(numAgents - 1)), 0));
//	}
//	targets.push_back(glm::vec3(0, 0, 0.06f));
//
//
//	//From horizontal to vertical  --testName:6pHV
//	for (int i = 0; i < numAgents; i++)
//	{
//		starts.push_back(glm::vec3(r * cos(i * 2.f * M_PI / (float)numAgents), r * sin(i * 2.f * M_PI / (float)numAgents), 0));
//	}
//	for (int i = 0; i < numAgents; i++)
//	{
//		targets.push_back(glm::vec3(0, r * cos(i * 2.f * M_PI / (float)numAgents), r * sin(i * 2.f * M_PI / (float)numAgents)));
//	}
//
//
//	// start shape A
//	starts = {
//		glm::vec3(0.040, 0.0, 0.015),
//		glm::vec3(0.020, 0.013, 0.015),
//		glm::vec3(-0.01, 0.030, 0.015),
//		glm::vec3(-0.03, 0.045, 0.015),
//		glm::vec3(0.020, -0.013, 0.015),
//		glm::vec3(-0.01, 0.0, 0.015),
//		glm::vec3(-0.01, -0.030, 0.015),
//		glm::vec3(-0.03, -0.045, 0.015),
//	};
//
//	
//
//	// target shape U
//	
//	targets = {
//		glm::vec3(0.040, 0.035, 0),
//		glm::vec3(0.010, 0.035, 0),
//		glm::vec3(-0.020, 0.035, 0),
//		glm::vec3(-0.040, 0.015, 0),
//		glm::vec3(-0.040,-0.015, 0),
//		glm::vec3(-0.020,-0.035, 0),
//		glm::vec3(0.010,-0.035, 0),
//		glm::vec3(0.040,-0.035, 0)
//	};
//
//	// start shape C
//
//	starts = {
//		glm::vec3(0.030, -0.035, 0),
//		glm::vec3(0.040, -0.010, 0),
//		glm::vec3(0.035, 0.020, 0),
//		glm::vec3(0.015, 0.035, 0),
//		glm::vec3(-0.015, 0.035, 0),
//		glm::vec3(-0.035, 0.020, 0),
//		glm::vec3(-0.040, -0.010, 0),
//		glm::vec3(-0.030, -0.035, 0)
//	};
//
//	// target shape L
//
//	targets = {
//		glm::vec3(0.04, 0.03, 0),
//		glm::vec3(0.02, 0.03, 0),
//		glm::vec3(0.00, 0.03, 0),
//		glm::vec3(-0.02, 0.03, 0),
//		glm::vec3(-0.04, 0.03, 0),
//		glm::vec3(-0.04, 0.01, 0),
//		glm::vec3(-0.04,-0.01, 0),
//		glm::vec3(-0.04,-0.03, 0)
//	};
//
//
//	//simple vertical paths (up and down)
//
//	starts = {
//		glm::vec3(0.02, 0.02, 0),
//		glm::vec3(0.02, 0.0, 0),
//		glm::vec3(0.02, -0.02, 0),
//		glm::vec3(0.0, 0.02, 0),
//		glm::vec3(0.0, 0.0, 0),
//		glm::vec3(0.0, -0.02, 0),		
//	};
//	
//}