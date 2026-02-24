#include "maze.h"
#include "Arduino.h"

coord inicio = {kBaseCoord, kBaseCoord, kBaseCoord};
coord robotCoord = {kBaseCoord, kBaseCoord, kBaseCoord};
TileDirection directions[4] = {TileDirection::kLeft, TileDirection::kDown, TileDirection::kRight, TileDirection::kUp};
coord checkpointCoord = {kBaseCoord, kBaseCoord, kBaseCoord};
int robotOrientation = 0;
uint8_t level = kBaseCoord;
coord past;
maze::maze(){}

// ---------------------------------------------------------------
void detection(Tile* curr){
    if(curr == nullptr) return;
    if(curr->position_ == kInvalidPosition) return; // not initialized
    if(!curr->hasVictim()){
        if(robot.buttonPressed) return;
        if(robot.victim != 0) curr->setVictim();
        robot.victim = 0;
        robot.kitState = kitID::kNone;
    }
}

// Safe detection wrapper — skips uninitialized tiles
static void safeDetect(Tile* t){
    if(t != nullptr && t->position_ != kInvalidPosition) detection(t);
}

// ---------------------------------------------------------------
void maze::followPath(Stack& path, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap){

    if(path.empty()){
        Serial.println("followPath: path is empty, returning");
        return;
    }

    const coord init = path.top();
    uint8_t initIdx = tilesMap.getIndex(init);
    if(initIdx == kMaxInt){
        Serial.println("followPath: init coord not in map, aborting");
        return;
    }

    Tile* curr = &tiles.getValue(initIdx);
    // Guard: init tile must be initialized
    if(curr->position_ == kInvalidPosition){
        Serial.println("followPath: init tile not initialized, aborting");
        return;
    }
    path.pop();

    while(!path.empty()){
        const coord next = path.top();
        path.pop();

        Serial.print("followPath: moving to (");
        Serial.print(next.x); Serial.print(",");
        Serial.print(next.y); Serial.print(",");
        Serial.print(next.z); Serial.println(")");

        // Validate next coord is in the map
        uint8_t nextIdx = tilesMap.getIndex(next);
        if(nextIdx == kMaxInt){
            Serial.println("followPath: next not in map, skipping");
            continue;
        }

        // Validate the tile at nextIdx is actually initialized
        Tile* nextTile = &tiles.getValue(nextIdx);
        if(nextTile->position_ == kInvalidPosition){
            Serial.println("followPath: next tile not initialized, skipping");
            continue;
        }

        // Guard curr before any access
        if(curr == nullptr || curr->position_ == kInvalidPosition){
            Serial.println("followPath: curr tile invalid, skipping detection");
            curr = nextTile;
            continue;
        }

        if(next.x > robotCoord.x){
            if(robotOrientation != 90) safeDetect(curr);
            if(robotOrientation == 270){ robot.rotate(180); if(robot.buttonPressed) break; safeDetect(curr); }
            robot.rotate(90); if(robot.buttonPressed) break; safeDetect(curr);
            robotOrientation = 90;
        } else if(next.x < robotCoord.x){
            if(robotOrientation != 270) safeDetect(curr);
            if(robotOrientation == 90){ robot.rotate(180); if(robot.buttonPressed) break; safeDetect(curr); }
            robot.rotate(270); if(robot.buttonPressed) break; safeDetect(curr);
            robotOrientation = 270;
        } else if(next.y > robotCoord.y){
            if(robotOrientation != 0) safeDetect(curr);
            if(robotOrientation == 180){ robot.rotate(90); if(robot.buttonPressed) break; safeDetect(curr); }
            robot.rotate(0); if(robot.buttonPressed) break; safeDetect(curr);
            robotOrientation = 0;
        } else if(next.y < robotCoord.y){
            if(robotOrientation != 180) safeDetect(curr);
            if(robotOrientation == 0){ robot.rotate(90); if(robot.buttonPressed) break; safeDetect(curr); }
            robot.rotate(180); if(robot.buttonPressed) break; safeDetect(curr);
            robotOrientation = 180;
        }

        if(robot.buttonPressed) break;
        robot.ahead();
        if(robot.buttonPressed) break;
        if(robot.blackTile) continue;

        past = robotCoord;
        robotCoord = next;
        curr = nextTile; // already validated above
    }
}

// ---------------------------------------------------------------
void maze::dijkstra(coord& start, coord& end, arrCustom<coord>& tilesMap, arrCustom<Tile>& tiles){

    uint8_t startIdx = tilesMap.getIndex(start);
    uint8_t endIdx   = tilesMap.getIndex(end);

    if(startIdx == kMaxInt){
        Serial.println("dijkstra: start not in map, skipping");
        return;
    }
    if(endIdx == kMaxInt){
        Serial.println("dijkstra: end not in map, skipping");
        return;
    }
    if(start == end){
        Serial.println("dijkstra: start == end, skipping");
        return;
    }

    uint8_t used = tilesMap.getUsed();
    if(used == 0){
        Serial.println("dijkstra: map is empty, skipping");
        return;
    }

    Stack path;
    arrCustom<bool>  explored(used, false);
    arrCustom<int>   distance(used, INT_MAX);
    arrCustom<coord> previousPositions(used, kInvalidPosition);

    distance.set(startIdx, 0);
    explored.set(startIdx, true);

    uint8_t minDist;
    coord current = start;
    robot.screenPrint("Dijkstra");

    while(!explored.getValue(endIdx)){
        uint8_t currIdx = tilesMap.getIndex(current);
        if(currIdx == kMaxInt) break;

        const Tile& currentTile = tiles.getValue(currIdx);

        // Guard: skip uninitialized tiles
        if(currentTile.position_ == kInvalidPosition) break;

        for(const TileDirection& direction : directions){
            if(currentTile.adjacentTiles_[static_cast<int>(direction)] == nullptr) continue;
            if(currentTile.hasWall(direction)) continue;
            if(currentTile.hasBlackTile()) continue;

            const coord& adjacent = currentTile.adjacentTiles_[static_cast<int>(direction)]->position_;
            if(adjacent == kInvalidPosition) continue; // adjacent tile not initialized

            uint8_t adjIdx = tilesMap.getIndex(adjacent);
            if(adjIdx == kMaxInt) continue;

            const int weight = currentTile.weights_[static_cast<int>(direction)] + distance.getValue(currIdx);
            if(weight < distance.getValue(adjIdx)){
                distance.set(adjIdx, weight);
                previousPositions.set(adjIdx, current);
            }
            if(robot.buttonPressed) return;
        }

        minDist = kMaxInt;
        for(uint8_t i = 0; i < used; i++){
            const coord& currentCoord = tilesMap.getValue(i);
            if(currentCoord == kInvalidPosition) continue;

            uint8_t idx = tilesMap.getIndex(currentCoord);
            if(idx == kMaxInt) continue;

            const int currentDistance = distance.getValue(idx);
            if(currentDistance < minDist && !explored.getValue(idx)){
                minDist = currentDistance;
                current = currentCoord;
            }
            if(robot.buttonPressed) return;
        }

        if(minDist == kMaxInt){
            Serial.println("dijkstra: destination unreachable");
            return;
        }
        explored.set(tilesMap.getIndex(current), true);
        if(robot.buttonPressed) return;
    }

    // Reconstruct path
    current = end;
    bool pathFound = true;
    while(current != start){
        if(robot.buttonPressed) return;
        uint8_t idx = tilesMap.getIndex(current);
        if(idx == kMaxInt){
            Serial.println("dijkstra: path broken, coord missing");
            pathFound = false;
            break;
        }
        coord prev = previousPositions.getValue(idx);
        if(prev == kInvalidPosition){
            Serial.println("dijkstra: no path to start");
            pathFound = false;
            break;
        }
        path.push(current);
        current = prev;
    }

    if(!pathFound) return;

    path.push(start);
    followPath(path, tiles, tilesMap);
}

// ---------------------------------------------------------------
void maze::dfs(arrCustom<coord>& visitedMap, arrCustom<Tile>& tiles, arrCustom<coord>& tilesMap){
    Stack unvisited;
    unvisited.push(robotCoord);
    coord next;
    Tile* currentTile;
    TileDirection oppositeDirection;
    bool visitedFlag = false;
    bool wall = false;

    while(!unvisited.empty()){
        coord current = unvisited.top();
        unvisited.pop();

        visitedFlag = false;
        for(int i = 0; i < visitedMap.getSize(); ++i){
            if(visitedMap.getValue(i) == current){ visitedFlag = true; break; }
        }
        if(visitedFlag) continue;

        dijkstra(robotCoord, current, tilesMap, tiles);
        visitedMap.push_back(current);

        if(robot.blackTile){
            uint8_t idx = tilesMap.getIndex(current);
            if(idx != kMaxInt){
                currentTile = &tiles.getValue(idx);
                currentTile->setBlackTile();
            }
            robot.blackTile = false;
            continue;
        }

        if(robot.checkpoint == true){
            uint8_t idx = tilesMap.getIndex(current);
            if(idx != kMaxInt){
                currentTile = &tiles.getValue(idx);
                currentTile->setCheckpoint();
            }
            checkpointCoord = current;
            robot.checkpoint = false;
        }

        robotCoord = current;

        if(robot.buttonPressed){
            delay(50);
            if(digitalRead(Pins::checkpointPin) == 1){
                robot.screenPrint("LoP");
                robotCoord = inicio;
                robotOrientation = 0;
                robot.screenPrint("Inicio");
                bool braker = false;
                while(true){
                    Serial.println("wat");
                    robot.screenPrint("Esperando");
                    if(!robot.buttonPressed){
                        unsigned long time = millis();
                        while(digitalRead(Pins::checkpointPin) == 1){
                            if((millis() - time) > 500){
                                ESP.restart();
                                robot.screenPrint("r");
                                delay(500);
                                robot.resetVlx();
                                robot.bno.setupBNO();
                                braker = true;
                                break;
                            }
                        }
                        break;
                    }
                }
                robot.screenPrint("Dale");
                visitedMap.reset();
                tilesMap.reset();
                tiles.reset();
                current = robotCoord;
                unvisited.~Stack();
                unvisited.push(robotCoord);
                tilesMap.push_back(robotCoord);
                tiles.set(tilesMap.getIndex(robotCoord), Tile(robotCoord));
                if(!braker) robot.checkpointElection();
                robot.resetOrientation();
                robot.buttonPressed = false;
                continue;
            } else {
                robot.buttonPressed = false;
            }
        }

        uint8_t currentIdx = tilesMap.getIndex(current);
        if(currentIdx == kMaxInt){
            Serial.println("dfs: current not in map after dijkstra, skipping");
            continue;
        }
        currentTile = &tiles.getValue(currentIdx);

        // Guard: tile must be initialized before using it
        if(currentTile->position_ == kInvalidPosition){
            Serial.println("dfs: current tile not initialized, skipping");
            continue;
        }

        if(robot.rampState != 0){
            int rampDirection = robot.rampState == 1 ? 1 : -1;
            robot.rampState = 0;

            uint8_t rcIdx = tilesMap.getIndex(robotCoord);
            if(rcIdx == kMaxInt) continue;
            Tile* currTile = &tiles.getValue(rcIdx);
            if(currTile->position_ == kInvalidPosition) continue;

            for(int i = 0; i < kNumberOfDirections; i++){
                if(currTile->adjacentTiles_[i] != nullptr &&
                   currTile->adjacentTiles_[i]->position_ == past){
                    Tile* pastTile = currTile->adjacentTiles_[i];

                    current.z += rampDirection;
                    visitedMap.push_back(current);
                    tilesMap.push_back(current);
                    tiles.set(tilesMap.getIndex(current), Tile(current));
                    Tile* newTile = &tiles.getValue(tilesMap.getIndex(current));

                    for(TileDirection dir : directions){
                        TileDirection opposite;
                        if(dir == TileDirection::kUp)         opposite = TileDirection::kDown;
                        else if(dir == TileDirection::kDown)  opposite = TileDirection::kUp;
                        else if(dir == TileDirection::kRight) opposite = TileDirection::kLeft;
                        else                                  opposite = TileDirection::kRight;

                        if(pastTile->adjacentTiles_[static_cast<int>(dir)] != nullptr &&
                           pastTile->adjacentTiles_[static_cast<int>(dir)]->position_ == robotCoord){
                            pastTile->addAdjacentTile(dir, newTile, false, false);
                            newTile->addAdjacentTile(opposite, pastTile, false, false);
                            break;
                        }
                    }
                    break;
                }
            }

            robotCoord = current;
            currentIdx = tilesMap.getIndex(current);
            if(currentIdx == kMaxInt) continue;
            currentTile = &tiles.getValue(currentIdx);
            if(currentTile->position_ == kInvalidPosition) continue;

            for(const TileDirection direction : directions){
                wall = false;
                if(robot.isWall(static_cast<int>(direction))) wall = true;
                switch(direction){
                    case TileDirection::kRight:
                        next = {static_cast<uint8_t>(current.x + 1), current.y, current.z};
                        if(robotOrientation == 270) continue;
                        oppositeDirection = TileDirection::kLeft;
                        break;
                    case TileDirection::kUp:
                        next = {current.x, static_cast<uint8_t>(current.y + 1), current.z};
                        if(robotOrientation == 180) continue;
                        oppositeDirection = TileDirection::kDown;
                        break;
                    case TileDirection::kLeft:
                        next = {static_cast<uint8_t>(current.x - 1), current.y, current.z};
                        if(robotOrientation == 90) continue;
                        oppositeDirection = TileDirection::kRight;
                        break;
                    case TileDirection::kDown:
                        next = {current.x, static_cast<uint8_t>(current.y - 1), current.z};
                        if(robotOrientation == 0) continue;
                        oppositeDirection = TileDirection::kUp;
                        break;
                }
                if(currentTile->adjacentTiles_[static_cast<int>(direction)] == nullptr){
                    uint8_t index = tilesMap.getIndex(next);
                    Tile* nextTile;
                    if(index == kMaxInt){
                        tilesMap.push_back(next);
                        tiles.set(tilesMap.getIndex(next), Tile(next));
                        nextTile = &tiles.getValue(tilesMap.getIndex(next));
                    } else {
                        nextTile = &tiles.getValue(index);
                    }
                    if(nextTile->position_ == kInvalidPosition) nextTile->setPosition(next);
                    currentTile->addAdjacentTile(direction, nextTile, wall, false);
                    nextTile->addAdjacentTile(oppositeDirection, currentTile, wall, false);
                    if(!wall){
                        visitedFlag = false;
                        for(uint8_t i = 0; i < visitedMap.getSize(); ++i){
                            if(visitedMap.getValue(i) == next){ visitedFlag = true; break; }
                        }
                        if(!visitedFlag){ unvisited.push(next); robot.screenPrint("Entre"); }
                    }
                }
            }
            continue;

        } else {

            for(const TileDirection direction : directions){
                wall = false;
                if(robot.isWall(static_cast<int>(direction))) wall = true;
                switch(direction){
                    case TileDirection::kRight:
                        next = {static_cast<uint8_t>(current.x + 1), current.y, current.z};
                        oppositeDirection = TileDirection::kLeft;
                        break;
                    case TileDirection::kUp:
                        next = {current.x, static_cast<uint8_t>(current.y + 1), current.z};
                        oppositeDirection = TileDirection::kDown;
                        break;
                    case TileDirection::kLeft:
                        next = {static_cast<uint8_t>(current.x - 1), current.y, current.z};
                        oppositeDirection = TileDirection::kRight;
                        break;
                    case TileDirection::kDown:
                        next = {current.x, static_cast<uint8_t>(current.y - 1), current.z};
                        oppositeDirection = TileDirection::kUp;
                        break;
                }
                if(currentTile->adjacentTiles_[static_cast<int>(direction)] == nullptr){
                    uint8_t index = tilesMap.getIndex(next);
                    Tile* nextTile;
                    if(index == kMaxInt){
                        tilesMap.push_back(next);
                        tiles.set(tilesMap.getIndex(next), Tile(next));
                        nextTile = &tiles.getValue(tilesMap.getIndex(next));
                    } else {
                        nextTile = &tiles.getValue(index);
                    }
                    if(nextTile->position_ == kInvalidPosition) nextTile->setPosition(next);
                    currentTile->addAdjacentTile(direction, nextTile, wall, false);
                    if(robot.blueTile){
                        nextTile->addAdjacentTile(oppositeDirection, nextTile, wall, true);
                        robot.blueTile = false;
                    } else {
                        nextTile->addAdjacentTile(oppositeDirection, currentTile, wall, false);
                    }
                    if(!wall){
                        visitedFlag = false;
                        for(uint8_t i = 0; i < visitedMap.getSize(); ++i){
                            if(visitedMap.getValue(i) == next){ visitedFlag = true; break; }
                        }
                        if(!visitedFlag){ unvisited.push(next); }
                    }
                }
            }
        }
    }

    if(unvisited.empty()){ robot.screenPrint("Unvisited empty"); delay(5000); }

    // Only return to start if we actually explored more than just the start tile
    if(!robot.buttonPressed && tilesMap.getUsed() > 1){
        dijkstra(robotCoord, inicio, tilesMap, tiles);
    }
}

void maze::run_algs(){
    Serial.println(robot.bno.getOrientationX());

    // Wait for VLX tasks to produce real readings
    Serial.println("Waiting for VLX sensors...");
    delay(3000);

    // Debug: print wall readings before starting
    Serial.println("Wall readings:");
    for(int i = 0; i < 4; i++){
        Serial.print("  Direction "); Serial.print(i);
        Serial.print(": isWall="); Serial.println(robot.isWall(i));
    }

    arrCustom<coord> visitedMap(kMaxSize, kInvalidPosition);
    arrCustom<coord> tilesMap(kMaxSize, kInvalidPosition);
    arrCustom<Tile>  tiles(kMaxSize, Tile(kInvalidPosition));

    tilesMap.push_back(robotCoord);
    tiles.set(tilesMap.getIndex(robotCoord), Tile(robotCoord));

    Serial.println("Starting DFS");
    dfs(visitedMap, tiles, tilesMap);
}