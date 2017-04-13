/*  A implementation of A* for solving the Paradox path-finding problem.
The solution is created by Alexander Poole alex.o.poole@gmail.com
*/
#include <queue>
#include <algorithm>
#include <iostream>

/* Custom classes ----
*/

// Map node
class Node
{
public:
	Node(const int nX, const int nY,
		const int pos, const int costToNode, const int costToGoal, Node * parent) {
		nMyX = nX;
		nMyY = nY;
		myPos = pos;
		parentNode = parent;
		g = costToNode; // Minimum cost from start to node
		h = costToGoal; // Minimum cost to target
		f = g + h; // Node score the lower the better
	};

	bool operator <(const Node& rhs) {
		return (this->getCost() < rhs.getCost());
	};

	bool operator==(const int& rhs) {
		return (this->getPos() == rhs);
	}

	int getX() const { return nMyX; };
	int getY() const { return nMyY; };
	int getScore() const { return f; };
	int getCost() const { return g; };
	int getPos() const { return myPos; };
	Node * getParent() const { return parentNode; };

	void setScore(const int newCost) {
		g = newCost;
		f = g + h;
	};
	void setParent(Node * newParent) {
		parentNode = newParent;
	}

private:
	int nMyX,
		nMyY,
		myPos,
		g,
		h,
		f;
	Node * parentNode;
};

// Searchable priority queue
/*template<
	class T,
	class Container = std::vector<T>,
	class Compare = std::less<typename Container::value_type>
> class AstarQueue : public std::priority_queue<T, Container, Compare>
{
public:
	typedef typename
		std::priority_queue<
		T,
		Container,
		Compare>::container_type::const_iterator const_iterator;

	const_iterator find(const int val) const {
		auto first = this->c.cbegin();
		auto last = this->c.cend();
		while (first != last) {
			if ((*first)->getPos() == val) {
				return first;
			}
			++first;
		}
		return last;
	}

	const_iterator end() const {
		return this->c.cend();
	}
};*/


/* Help functions ----
*/

// Heuristic estimate of distance to goal
int costToGoal(const int nMyX, const int nMyY,
	const int nTargetX, const int nTargetY) {
	return std::abs(nMyX - nTargetX) + std::abs(nMyY - nTargetY); // Manhattan distance
};

// Calculates position in the map
int Pos(const int nX, const int nY, const int nMapWidth) {
	return nX + nY*nMapWidth;
}

struct Compare {
	bool operator()(Node* lhs, Node* rhs) {
		return lhs->getScore() > rhs->getScore();
	}
};

std::vector<Node>::iterator AstarFind(std::vector<Node>& vec, const int val)
{
	std::vector<Node>::iterator first = vec.begin();
	std::vector<Node>::iterator last = vec.end();

	while (first != last) {
		if (first->getPos() == val) {
			return first; 
		}
		++first;
	}
	return last;
}

std::vector<Node*>::iterator AstarFind(std::vector<Node*>& vec, const int val)
{
	std::vector<Node*>::iterator first = vec.begin();
	std::vector<Node*>::iterator last = vec.end();

	while (first != last) {
		if ((*first)->getPos() == val) {
			return first;
		}
		++first;
	}
	return last;
}

bool SortFunc(Node * lhs, Node* rhs) {
	return lhs->getScore() > rhs->getScore();
}

/* FindPath ----
*/

int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize) {

	int shortestPath = 0;

	Node finalNode(nTargetX, nTargetY,
		Pos(nTargetX, nTargetY, nMapWidth), -1, -1, nullptr);

	Node startNode(nStartX, nStartY,
		Pos(nStartX, nStartY, nMapWidth), 0, -1, nullptr);

	// If startNode equals finalNode add it to buffer
	if (finalNode.getPos() == startNode.getPos()) {
		// Final pos added even if same as start pos
		pOutBuffer[0] = finalNode.getPos();
		return shortestPath;
	}

	std::vector<Node*> openNodes;
	std::vector<Node> closedNodes;
	openNodes.push_back(&startNode);

	while (!openNodes.empty() && shortestPath < nOutBufferSize) {
		std::sort(openNodes.begin(), openNodes.end(), SortFunc);
		Node * currentNode = openNodes.back();
		openNodes.pop_back();

		closedNodes.push_back(*currentNode);

		shortestPath = currentNode->getCost();

		// Best path to goal found
		if (finalNode.getPos() == currentNode->getPos()) {

			pOutBuffer[shortestPath - 1] = finalNode.getPos();

			int i = shortestPath - 1;
			Node * parent = currentNode->getParent();
			// Backtracing path to goal 
			while (parent->getParent() != nullptr)
			{
				i--;
				pOutBuffer[i] = parent->getPos();
				parent = parent->getParent();
			}
			return shortestPath;
		}
		else {
			int neighbourhood[4] = { 1, 0 , -1, 0 };

			// Loops through possible neighbourNodes and adds them if they are new
			for (int j = 0; j < 4; j++)
			{
				const int nX = currentNode->getX() + neighbourhood[j % 4];
				const int nY = currentNode->getY() + neighbourhood[(j + 1) % 4];
				const int pos = Pos(nX, nY, nMapWidth);

				// Check if neighbourNode is in valid position
				if ((nX < 0 || nX == nMapWidth) ||
					(nY < 0 || nY == nMapHeight) ||
					pMap[pos] == 0) {
					continue;
				}
				// Node has already been visited
				else if (AstarFind(closedNodes, pos) != closedNodes.end())
				{
					continue;
				}
				else
				{
					const int g = currentNode->getCost() + 1;
					const int h = costToGoal(nX, nY, nTargetX, nTargetY);

					std::vector<Node*>::iterator openNodesIt = AstarFind(openNodes, pos);

					// Adding a new node to openNodes
					if (openNodesIt == openNodes.end())
					{
						Node * neighbourNode = new Node(nX, nY,
							pos, g, h, currentNode);
						openNodes.push_back(neighbourNode);
					}
					// Node already exists, updating if a better 
					// path to node is found
					else if ((*openNodesIt)->getScore() > g + h) {
						(*openNodesIt)->setScore(g);
						(*openNodesIt)->setParent(currentNode);
					}
				}
			}
		}
	}

	return -1;
}

int main() {

	unsigned char pMap[] = { 
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,
		1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0 ,
		1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1 ,
		1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1 ,
		1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1 ,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	int pOutBuffer[60];
	int steps =  FindPath(0, 0, 11, 18, pMap, 12, 19, pOutBuffer, 60);

	/*unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
	int pOutBuffer[12];
	int steps = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);*/

	std::cout << "steps: " << steps << std::endl;

	if (steps != -1)
	{
		for (size_t i = 0; i < steps; i++)
		{
			std::cout << pOutBuffer[i] << std::endl;
		}
	}


	std::cin.get();
	return 0;
}
