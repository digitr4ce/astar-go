package main

import (
	"container/heap"
	"fmt"
	"math/rand"
	"slices"
)

type Coordinate struct {
	X int
	Y int
}

type Neighbor struct {
	node *Node
	isDiag bool
}

func intAbs(number int) int {
	if number < 0 {
		return -number
	}
	return number
}

type octFunc func(Coordinate, Coordinate) int

// As per http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
// With 10x magnitude to not use floats
func octileDistance(nodeCoords Coordinate, goalCoords Coordinate) int {
	dx := intAbs(nodeCoords.X - goalCoords.X)
	dy := intAbs(nodeCoords.Y - goalCoords.Y)
	// return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
	// D = 10, D2 = 14
	return 10 * (dx + dy) - 6 * min(dx, dy)
}

func createBlocker(x, y int) Node {
	return Node{
		distance: 250,
		coordinate: Coordinate{
			X: x,
			Y: y,
		},
		blocker: true,
	}
}

// TODO: Forgot about southern and eastern walls?
func generateTerrain() [10][10]Node {
	var terrain [10][10]Node
	// Generate the walls
	for i := 0; i < 10; i++ {
		terrain[0][i] = createBlocker(0, i)
		terrain[9][i] = createBlocker(9, i)
		terrain[i][0] = createBlocker(i, 0)
		terrain[i][9] = createBlocker(i, 9)
	}
	for i := 1; i < 9; i++ {
		for j := 1; j < 9; j++ {
			terrain[i][j] = Node{
				distance: rand.Intn(6),
				coordinate: Coordinate{
					X: i,
					Y: j,
				},
				blocker: false,
			}
		}
	}
	return terrain
}

func reconstructPath(cameFrom map[*Node]*Node, current *Node) []Node {
	totalPath := []Node{*current}
	_, ok := cameFrom[current]
	for ok {
		current = cameFrom[current]
		totalPath = slices.Insert(totalPath, 0, *current)
		_, ok = cameFrom[current]
	}
	return totalPath
}

// TODO: Add simplified version.
func printResult(totalPath []Node, terrain [10][10]Node) {
	for i := 0; i < 9; i++ {
		for j := 0; j < 9; j++ {
			found := false
			for _, node := range totalPath {
				if i == node.coordinate.X && j == node.coordinate.Y {
					fmt.Printf("(#, %d) ", node.distance)
					found = true
				}
			}
			if !found {
				fmt.Printf("(., %d)", terrain[i][j].distance)
			}
		}
		fmt.Println()
	}
}

func getNeighbors(terrain [10][10]Node, nodeCoords Coordinate) []Neighbor {
	var crossNeighbors = []Coordinate{
		{X: 1, Y: 0},
		{X: 0, Y: 1},
		{X: -1, Y: 0},
		{X: 0, Y: -1},
	}
	var diagonalNeighbors = []Coordinate{
		{X: 1, Y: -1},
		{X: 1, Y: 1},
		{X: -1, Y: -1},
		{X:-1, Y: 1},
	}
	var neighbors []Neighbor
	for _, neighbor := range crossNeighbors {
		currentNode := terrain[nodeCoords.X + neighbor.X][nodeCoords.Y + neighbor.Y]
		if !currentNode.blocker {
			neighbors = append(neighbors, Neighbor{node: &currentNode, isDiag: false})
		}
	}
	for _, neighbor := range diagonalNeighbors {
		currentNode := terrain[nodeCoords.X + neighbor.X][nodeCoords.Y + neighbor.Y]
		if !currentNode.blocker {
			neighbors = append(neighbors, Neighbor{node: &currentNode, isDiag: true})
		}
	}
	return neighbors
}

func aStar(start Node, goal Node, heuristic octFunc, terrain [10][10]Node) []Node {
	openSet := make(PriorityQueue, 0)
	heap.Init(&openSet)
	cameFrom := make(map[*Node]*Node)
	gScore := make(map[*Node]int)
	gScore[&start] = 0
	fScore := make(map[*Node]int)
	fScore[&start] = heuristic(start.coordinate, goal.coordinate)
	openSet.Push(&start)
	for openSet.Len() != 0 {
		current := heap.Pop(&openSet).(*Node)
		if current.coordinate == goal.coordinate {
			return reconstructPath(cameFrom, current)
		}
		neighbors := getNeighbors(terrain, current.coordinate)
		for _, neighbor := range neighbors {
			distance := terrain[neighbor.node.coordinate.X][neighbor.node.coordinate.Y].distance
			if neighbor.isDiag {
				distance = distance * 14
			} else {
				distance = distance * 10
			}
			tentativeGScore := gScore[current] + distance
			// If gScore[*neighbor.node] is 0, then it was not yet initialized
			if tentativeGScore < gScore[neighbor.node] || gScore[neighbor.node] == 0 {
				cameFrom[neighbor.node] = current
				gScore[neighbor.node] = tentativeGScore
				fScore[neighbor.node] = tentativeGScore + octileDistance(neighbor.node.coordinate, goal.coordinate)
				if !neighbor.node.visited {
					neighbor.node.visited = true
					terrain[neighbor.node.coordinate.X][neighbor.node.coordinate.Y].visited = true
					openSet.Push(neighbor.node)
				} 
			}
		}
	}
	return make([]Node, 0)
}

func main() {
	terrain := generateTerrain()
	aStarRes := aStar(terrain[1][1], terrain[8][8], octileDistance, terrain)
	if len(aStarRes) > 0 {
		printResult(aStarRes, terrain)
	} else {
		fmt.Print("No path found!")
	}
}