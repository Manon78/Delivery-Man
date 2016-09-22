BetterDM <- function(roads,car,packages) {
  nextMove <- 0
  toGo <- 0
  offset <- 0
  if (car$load == 0) {
    # pick which package to pick up somehow
    #By selecting the one with lowest total cost after A* algorithm has run for all options
    #for movement to package and movement from package to destination
  }
  
  else {
    toGo <- car$load  
    offset <- 2
  }
  
  goal <- c(packages[toGo, offset + 1], packages[toGo, offset + 2])
  
  car$nextMove <- aStar(car, goal, roads)  # the optimal next move to make, as integer
  car$mem <- list()
  return (car)
  
}


AStar <- function(car, goal, roads) {
  gridSize <- nrow(roads$hroads)
  frontierSize <- 0  # use this to track how many nodes currently in frontier
  
  # initialise all the nodes with default values of attributes
  # access a node by its coordinates: nodes[[x]][[y]]
  nodes = list()
  for (i in 1:gridSize) {
    nodes[[i]] <- list()
    for (j in 1:gridSize) {
      nodes[[i]][[j]] <- list(realCost <- 10000, f <- 10000, h <- 10000, parent <- c(x <- i, y <- j), # c() is "combine" into a single list 
                              inFrontier <- FALSE, visited <- FALSE)  
    }
  }
  
  # set up the start node at car position with f = 0, add it to the frontier
  nodes[[car$x]][[car$y]]$f <- 0
  nodes[[car$x]][[car$y]]$inFrontier <- TRUE
  frontierSize <- frontierSize + 1
  
  # main loop
  while (frontierSize > 0) {
    # explore the node on the frontier with lowest f
    current <- FindLeastCostFrontierNode(nodes)
    
    # if that is the goal, done :)
    if (IsGoal(current, goal)) {
      # return the best next move
      return(GetMove(car, goal, nodes))
      #TODO: also return the value of realCost to goal
    }
    
    # else pop current node off the frontier, add to visited
    nodes[[current$x]][[current$y]]$inFrontier <- FALSE
    nodes[[current$x]][[current$y]]$visited <- TRUE
    #frontierSize <- frontierSize - 1
    
    # "explore" the current node
    neighbours <- GenerateNeighbour(current, gridSize)  # list of coordinates of all the neighbours
    # look at each of the neighbours in turn
    for (i in 1:length(neighbours)) {
      neighbour <- neighbours[[i]]
      if (isVisited(neighbour, nodes)) {  #TODO is this the right behaviour?
        #TODO: semi-certain answer: only ignore node if f is higher than f same position visited node
        i <- i + 1  # don't do anything with this node, as it has already been visited 
      }
      else{
        # calculate known cost of getting to that node, given current traffic
        cost <- nodes[[current$x]][[current$y]]$realCost + TravelCost(current, neighbour, roads)
        # if the node isn't yet in the frontier, or there is a copy of it in the frontier but this one has lower cost
        if ((!isInFrontier(neighbour, nodes)) || (cost < nodes[[neighbour$x]][[neighbour$y]]$realCost)) {  
          # add it to the frontier and set its attributes
          nodes[[neighbour$x]][[neighbour$y]]$parent <- c(current$x, current$y)
          nodes[[neighbour$x]][[neighbour$y]]$realCost <- cost
          nodes[[neighbour$x]][[neighbour$y]]$h <- Heuristic(neighbour$x, neighbour$y, goal)
          nodes[[neighbour$x]][[neighbour$y]]$f <- cost + nodes[[neighbour$x]][[neighbour$y]]$h
          
          # if that node wasn't already in the frontier, add it, and update the size of the frontier
          if (!isInFrontier(neighbour, nodes)) {
            nodes[[current$x]][[current$y]]$inFrontier <- TRUE 
            frontierSize <- frontierSize + 1
          }
        }
      }
    }
  }
  return(-1)  # something went wrong!
}


Heuristic <- function(currX, currY, goalXY) {
  # Manhattan distance from current coordinates to goal
  cost <- abs(goalXY[1] - currX) + abs(goalXY[2] - currY)
  return(cost)
} 


IsGoal <- function(currentNode, goal) {
  return(currentNode$x == goal[1] && currentNode$y == goal[2])
}

IsVisited <- function(currentNode, nodes) {
  return(nodes[[currentNode$x]][[currentNode$y]]$visited)
}

IsInFrontier <- function(currentNode, nodes) {
  return(nodes[[currentNode$x]][[currentNode$y]]$inFrontier)
}


GenerateNeighbours <- function(node, gridSize) {
  # Create a list of coordinates of all neighbours of node
  # gridSize required to ensure no nodes outside grid are explored
  
  x <- node$x
  y <- node$y
  neighbourNodes <- list()
  i <- 1
  if (x < gridSize) {  # right
    neighbourNodes[[i]] <- list(x <- x+1,y <- y)
    i <- i + 1
  }
  if (y < gridSize) {  # up
    neighbourNodes[[i]] <- list(x <- x,y <- y+1)
    i <- i + 1
  }
  if (x > 1) {  # left
    neighbourNodes[[i]] <- list(x <- x-1,y <- y)
    i <- i + 1
  }
  if (y > 1) {  # down
    neighbourNodes[[i]] <- list(x <- x,y <- y-1)
    i <- i + 1
  }
  return(neighbourNodes)
}


TravelCost <- function(current, neigbour, roads) {
  # calculate cost of travel between two adjacent nodes, given traffic
  x1 <- current$x
  y1 <- current$y
  x2 <- neighbour$x
  y2 <- neighbour$y
  if (x1 < x2)
    return(roads$hroads[y1,x1])
  if (x1 > x2)
    return(roads$hroads[y2,x2])
  if (y1 < y2)
    return(roads$vroads[y1,x1])
  if (y1 > y2)
    return(roads$vroads[y2,x2])
}



FindLeastCostFrontierNode <- function(nodes) {
  #TODO
  # leastCost <- some big number
  # nodeX <- 0
  # nodeY <- 0
  # for nodes where frontier == TRUE  ## this is the bit I don't know how to do...
  #   if node.f < leastCost
  #      leastCost <- node.f
  #      nodeX <- node.x
  #      nodeY <- node.y
  # return a node 
}


GetMove <- function(car, goal, nodes) {
  # at the end of A* search, trace the optimal path back from goal to car
  # return the first move to be made in appropriate integer form
  cameFrom <- nodes[[goal[1]]][[goal[2]]]$parent
  current <- goal
  path <- list()
  # the path back to the car begins at the current (goal) node 
  path[[1]] <- current
  i = 2
  # trace back using the parent attribute of nodes until find the first node you moved to
  while(parent[1] != car$x || parent[2] != car$y) {
    current <- cameFrom
    cameFrom <- nodes[[current[1]]][[current[2]]]$parent
    path[[i]] <- current
    i <- i + 1
  }
  
  # calculate which direction you need to move in
  x = current[1]
  y = current[2]
  if (car$x < x)
    return(6)
  if (car$x > x)
    return(4)
  if (car$y < y)
    return(8)
  if (car$y > y)
    return(2)
  return(5)
}
