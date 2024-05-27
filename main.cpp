/*
 Task:
 Generate the driver file (main.cpp) where you perform the following tasks.

 Operations to be performed:
    • Generate the following graph. Assume that all edge costs are 1.
    • Outdegree of a particular vertex in a graph is the number of edges going out from that vertex to other vertices
    For instance the outdegree of vertex "B" in the above graph is 1. Add a member function "OutDegree" to the
    "GraphType" class which returns the outdegree of a given vertex.
        `int  OutDegree(VertexType v);`

    • Add a member function to the class which determines if there is an edge between two vertices.
        `bool FoundEdge(VertexType u, VertexType v);`
    • Print the outdegree of the vertex "D".
    • Print if there is an edge between vertices "A" and D.
    • Print if there is an edge between vertices "B" and D.
    • Use depth first search in order to find if there is a path from "B" to "E".
    • Use depth first search in order to find if there is a path from "E" to "B".
    • Use breadth first search in order to find if there is a path from "B" to "E".
    • Use breadth first search in order to find if there is a path from "E" to "B".
    • Modify the "BreadthFirstSearch" function so that it also prints the length of the shortest path between two
    vertices.
    • Determine the length of the shortest path from "B" to "E".

 Note:
 Operations related to header files are added to the "graphtype.h" file.
*/

#include <iostream>
#include "graphtype.cpp"
using namespace std;

void Print(bool b){
    if(b)
        cout << "There is an edge." << endl;
    else
        cout << "There is no edge." << endl;
}

int main() {

    GraphType<char> g;                           // Create a graph object

    g.AddVertex('A');                            // Add specified number of vertices to the graph
    g.AddVertex('B');
    g.AddVertex('C');
    g.AddVertex('D');
    g.AddVertex('E');
    g.AddVertex('F');
    g.AddVertex('G');
    g.AddVertex('H');

    g.AddEdge('A', 'B', 1);                      // Add edges to the vertices assuming edge cost is 1
    g.AddEdge('A', 'C', 1);
    g.AddEdge('A', 'D', 1);

    g.AddEdge('B', 'A', 1);

    g.AddEdge('D', 'A', 1);
    g.AddEdge('D', 'E', 1);
    g.AddEdge('D', 'G', 1);

    g.AddEdge('G', 'F', 1);
    g.AddEdge('F', 'H', 1);
    g.AddEdge('H', 'E', 1);

    cout << g.OutDegree('D') << endl;            // Print outdegree of vertex D

    Print(g.FoundEdge('A', 'D'));                // Print if there is an edge between vertices A and D
    Print(g.FoundEdge('B', 'D'));                // Print if there is an edge between vertices B and D

    g.DepthFirstSearch('B', 'E');                // Use DFS in order to find if there is a path from B to E
    g.DepthFirstSearch('E', 'B');                // Use DFS in order to find if there is a path from E to B

    g.BreadthFirstSearch('B', 'E');              // Use BFS in order to find if there is a path from B to E
    g.BreadthFirstSearch('E', 'B');              // Use BFS in order to find if there is a path from E to B

    g.BreadthFirstSearch('B', 'E');              // Use BFS to print the length of the shortest path
    return 0;
}





/*

template<class VertexType>
void GraphType<VertexType>::DepthFirstSearch(VertexType startVertex, VertexType endVertex) {
    const int MAX_VERTICES = 50; // Maximum number of vertices
    VertexType currentPath[MAX_VERTICES]; // Store the current path being explored
    VertexType shortestPath[MAX_VERTICES]; // Store the shortest path found
    bool found = false; // Flag to indicate if the end vertex is found
    int currentPathLength = 0; // Length of the current path

    // Clear marks and start the DFS traversal from the start vertex
    ClearMarks();
    DFSRecursive(startVertex, endVertex, currentPath, shortestPath, currentPathLength, found);

    // If the end vertex is found, print the shortest path
    if (found) {
        cout << "Shortest path found: ";
        for (int i = 0; i < currentPathLength; i++) {
            cout << shortestPath[i] << " ";
        }
        cout << endl;
    } else {
        cout << "Path not found." << endl;
    }
}

template<class VertexType>
void GraphType<VertexType>::DFSRecursive(VertexType currentVertex, VertexType endVertex,
                                          VertexType currentPath[], VertexType shortestPath[],
                                          int currentPathLength, bool& found) {
    // Base case: If end vertex is reached
    if (currentVertex == endVertex) {
        currentPath[currentPathLength++] = currentVertex; // Include end vertex in the current path
        for (int i = 0; i < currentPathLength; i++) {
            shortestPath[i] = currentPath[i]; // Update shortest path
        }
        found = true; // Set found flag to true
        return;
    }

    // Mark the current vertex as visited
    MarkVertex(currentVertex);
    currentPath[currentPathLength++] = currentVertex; // Include current vertex in the current path

    // Explore adjacent vertices
    QueType<VertexType> vertexQ;
    GetToVertices(currentVertex, vertexQ);
    while (!vertexQ.IsEmpty()) {
        VertexType nextVertex;
        vertexQ.Dequeue(nextVertex);
        if (!IsMarked(nextVertex)) { // If the next vertex is not visited
            DFSRecursive(nextVertex, endVertex, currentPath, shortestPath, currentPathLength, found); // Recursive call
        }
    }

    // Backtrack: Remove the last vertex from the current path
    currentPathLength--;
}








template<class VertexType>
void GraphType<VertexType>::BreadthFirstSearch(VertexType startVertex, VertexType endVertex) {
    QueType<VertexType> queue;
    QueType<VertexType> vertexQ;

    QueType<int> lengthOfPath;           // Queue to store the length of the shortest path

    bool found = false;
    VertexType vertex, item;
    ClearMarks();
    queue.Enqueue(startVertex);

    lengthOfPath.Enqueue(0);             // Length of the shortest path is 0 for the start vertex

    do {
        queue.Dequeue(vertex);

        int length;
        lengthOfPath.Dequeue(length);     // Dequeue the length of the shortest path

        if (vertex == endVertex) {
            cout << vertex << " " << endl << length;
            found = true;
        } else {
            if (!IsMarked(vertex)) {
                MarkVertex(vertex);
                cout << vertex << " ";
                GetToVertices(vertex, vertexQ);
                while (!vertexQ.IsEmpty()) {
                    vertexQ.Dequeue(item);
                    if (!IsMarked(item)) {
                        queue.Enqueue(item);

                        lengthOfPath.Enqueue(length + 1);     // Length of the shortest path is increased by 1
                    }
                }
            }
        }
    } while (!queue.IsEmpty() && !found);
    cout << endl;
    if (!found)
        cout << "Path not found." << endl;
}


// Added OutDegree function which returns the outdegree of a given vertex
template<class VertexType>
int GraphType<VertexType>::OutDegree(VertexType v) {
    int res = 0;
    for (int i = 0; i < numVertices; i++) {
        if ((v != vertices[i]) && (WeightIs(v, vertices[i]) != NULL_EDGE))
            res++;
    }
    return res;
}

// Added FoundEdge function which determines if there is an edge between two vertices
template<class VertexType>
bool GraphType<VertexType>::FoundEdge(VertexType fromVertex, VertexType toVertex) {
    int row = IndexIs(vertices, fromVertex);
    int col = IndexIs(vertices, toVertex);
    if (edges[row][col] != NULL_EDGE)
        return true;
    else
        return false;
}



*/
