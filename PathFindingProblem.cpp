#include <set>
#include <tuple>
#include <vector>
#include <iterator>
#include <cmath>

#include <iostream>


int manhattenDist(int xA, int yA, int xB, int yB){
    return std::abs(xA-xB) + std::abs(yA-yB);
}


int pointToIndex(int x, int y, int width){
    if(y==0){
        return x;
    }else{
        return (y*width+x);
    }
}


std::tuple<int,int> indexToPoint(int index, int width){
    int x = index % width;
    int y = index / width;
    return std::make_tuple(x,y);
}


int allNeighbours(int node, const int nMapWidth, const int nMapHeight, std::vector<int>& neighbours){

    if(node >= nMapWidth){ //checks if the node is on the upper or lower border of the map
        neighbours.push_back(node-nMapWidth);
    }

    if(node < nMapWidth * (nMapHeight-1) ){
        neighbours.push_back(node+nMapWidth);
    }

    if(node % nMapWidth != 0){ //checks if the node is on the lefthand or righthand border of the map
        neighbours.push_back(node-1);
    }

    if(node % nMapWidth != nMapWidth - 1 ){
        neighbours.push_back(node+1);
    }

    return 0;
}


int validNeighbours(int node, int foundFrom, const unsigned char* pMap, const std::vector<bool> explored, 
                    const int nMapWidth, const int nMapHeight, std::vector<int>& neighbours){

    allNeighbours(node, nMapWidth, nMapHeight, neighbours);

    //std:: cout << "Neighbourship of: " << node << "\n";

    for(auto it=neighbours.begin(); it != neighbours.end(); ){
        //std::cout << " " << *it << "\n";
        if(pMap[*it] == 0 || *it == foundFrom || explored[*it]){
            it = neighbours.erase(it); //Always has a maximum of 4 elements so O(n) time for erase doesn't matter
        } else {
            ++it;
        }
    }

    return 0;
}



int heuristicDist(int point, int nTargetX, int nTargetY, int nMapWidth, const std::vector<int>& currentDist){
    std::tuple<int,int> tPoint = indexToPoint(point, nMapWidth);
    int pointX = std::get<0>(tPoint);
    int pointY = std::get<1>(tPoint);
    
    int hDist = currentDist[point] + manhattenDist(pointX, pointY, nTargetX, nTargetY);

    return hDist;
}


int reconstructPath(int target, int length, std::vector<int> origin, int* pOutBuffer){
    int node = target;
    for(int i = length-1; i>= 0; i = i-1){
        pOutBuffer[i] = node;
        node = origin[node];
    }
}


int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize){

                int pMapLength = nMapWidth * nMapHeight;

                //std::cout << pMapLength << "\n";

                int start = pointToIndex(nStartX, nStartY, nMapWidth);

                int target = pointToIndex(nTargetX, nTargetY, nMapWidth);

                //std::cout << "Target: " << target << "\n";

                std::vector<bool> explored(pMapLength, false); //True for every node if it is already fully explored and doesn't have to be visited further

                std::vector<int> origin(pMapLength, -1); //Saves either the predecessor of every node or -! if it wasn't visited yet. Used to reconstruct the shortest Path.

                std::vector<int> currentDist(pMapLength, -1); //Saves the distance of every node to the starting point. If the distance is -1 the node hasn't been explored yet

                int i = 0;

                for(auto it = currentDist.cbegin(); it != currentDist.cend(); ++it){
                    //std::cout << i << "-";
                    //std::cout << *it << "\n";
                    i = i + 1;
                }

                auto comp = [nTargetX, nTargetY, nMapWidth, &currentDist]
                 (int pointA, int pointB) -> bool { //Two points are compared by their current distance to the start as well as their estimated distance to the target
                    return heuristicDist(pointA, nTargetX, nTargetY, nMapWidth, currentDist) < heuristicDist(pointB, nTargetX, nTargetY, nMapWidth, currentDist);
                    };

                std::multiset<int,decltype(comp)> openSet(comp);

                std::vector<std::multiset<int>::const_iterator> position(pMapLength); //Current position of every node in OpenSet. Used for fast erasure.

                explored[start] = true;

                origin[start] = start;

                currentDist[start] = 0;

                position[start] = openSet.insert(start);

                int current;

                while(!openSet.empty()){ //gradually approaches the target by looking at the most promising (closest according to the heuristic) node

                    current = *openSet.begin();

                    if(currentDist[current] > nOutBufferSize){ //Buffer is to small for solution
                        return -2;
                    }

                    std::vector<int> neighbours;

                    validNeighbours(current, origin[current], pMap, explored, nMapWidth, nMapHeight, neighbours);

                    //std::cout << "Finisehd neighbourship analysis \n";

                    if(!neighbours.empty()){
                        //std::cout << "Start adding to Open Set \n";
                        for(auto it=neighbours.cbegin(); it != neighbours.cend(); ++it){

                            if(currentDist[*it] > currentDist[current] + 1){ //only updates node, if not explored or new path is shorter

                                //std::cout << "Started Distance Update of " << *it << "\n";
                                //std::cout << "Distance from Start to " << *it << " is " << currentDist[*it] << "\n";
                                //std::cout << "Distance from Start to " << current << " is " << currentDist[current] << "\n";

                                origin[*it] = current;
                                currentDist[*it] = currentDist[current] + 1;
                                openSet.erase(position[*it]);
                                position[*it] = openSet.insert(*it);

                                //std::cout << "Updated Distance of " << *it << "\n";

                            } else if(currentDist[*it] == -1){ //no old version is removed from the open Set, if the node is first explored

                                origin[*it] = current;
                                currentDist[*it] = currentDist[current] + 1;
                                position[*it] = openSet.insert(*it);

                                //std::cout << "Added " << *it << "to Open Set \n";

                                if(*it == target){ //Shortest Path was found, reconstructs recursively via the origin vector
                                    reconstructPath(*it, currentDist[*it], origin, pOutBuffer);
                                    return currentDist[*it];
                                }

                            }
                            
                        }
                        //std::cout << "Finished adding to Open Set \n";
                    }

                    explored[current] = true;
                    openSet.erase(position[current]);
                }
                
                return -1; //If the Open Set empties out, before the function terminates there exists no valid path between start and target
             }



int main(){
    /*
    unsigned char pMap[] = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
    int pOutBuffer[12];
    //std::cout << "Start: \n";
    int succsess = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
    **/

   unsigned char pMap[] = {0, 0, 1, 0, 1, 1, 1, 0, 1};
    int pOutBuffer[7];
    int succsess = FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7);

    std::cout << "Finished. \n";
    for(int i = 0; i < succsess; ++i){
        std::cout << pOutBuffer[i] << "\n";
    }

    std::cout << "Length: " << succsess << "\n";
    while(true){

    }
    return 0;
}