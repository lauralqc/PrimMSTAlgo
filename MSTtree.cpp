/******************************************************************************
 
 Online C++ Compiler.
 Code, Compile, Run and Debug C++ program online.
 Write your code in this editor and press "Run" button to compile and execute it.
 
 *******************************************************************************/
// Example program
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <string>


using namespace std;

#define FloatMax  (numeric_limits<float>::max())  // define the maximum value for float type

typedef pair<int, float> Pair;  // set node and vertices as a pair
typedef struct Edge
{
    int src, dest;
    float weight;
}edge_t;

// overloading compare operator in the priority_queue
struct Compare {
    constexpr bool operator()(pair<int, float> const & a,
                              pair<int, float> const & b) const noexcept
    { return a.second>b.second; }
};

// class for generating random graph matrix
class graph{
private:
    int vertices;  // number of vertices or nodes
    int edges;    // number of edges
    float **graph_ptr;  // generating the graph as a connectivity matrix
public:
    graph(int vertices, float dense, float low, float high); // constructor
    graph(string filename); // constructor that can read in a graph from a file
    ~graph();      // destructor
    int numVertice(void);  // get the number of vertices
    int numEdge(void); // get the number of edges
    bool adjacent (float **G, int x, int y); // tests whether there is an edge from node x to node y.
    unordered_map<int,float>neighbors (float **G, int x); // lists all nodes y such that there is an edge from x to y.
    float get_node_value (float**G,int x); // returns the value associated with the node x.
    void set_node_value( float**G, int x, float a); // sets the value associated with the node x to a.
    float get_edge_value(float **G, int x, int y); // returns the value associated to the edge (x,y).
    void set_edge_value (float **G, int x, int y, float v); // sets the value associated to the edge (x,y) to v.
    float** getGraphMatrix(void);  // get the pointer to the graph matrix
    void display(void); // display the graph matrix in the console
    
};

graph::graph(int vertices, float dense, float low, float high)
{
    int i=0,j = 0,edg = 0;
    this->edges = (int)(dense*vertices*(vertices-1)/2); // get the number of edges
    this->vertices = vertices;
    
    // allocate memory for the graph matrix
    this->graph_ptr = new float*[vertices];
    for (int i= 0; i < vertices; i++)
    {
        this->graph_ptr[i] = new float[vertices];
        for(int j = 0; j < vertices; j++)
        {
            graph_ptr[i][j] = FloatMax;
        }
    }
    
    /*  random graph procedure which means:
     if dense = 10%, then the graph would have 10% of its edges picked at random
     and its edge distance would be selected at random from the distance range
     */
    while(edg<edges)
    {
        i = rand()%this->vertices;
        j = rand()%this->vertices;
        if(i!=j && this->graph_ptr[i][j] == FloatMax)
        {
            float randV = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
            this->set_edge_value(this->graph_ptr,i,j,randV);
            this->set_edge_value(this->graph_ptr,j,i,randV);
            edg++;
        }
    }
}

// constructor that can read in a graph from a file
graph::graph(string filename)
{
    ifstream word_file(filename);
    istream_iterator<string> start(word_file), end;
    vector<string> words(start, end);
    if (!word_file.is_open()) {
        std::cout << "failed to open " << filename << '\n';
    }
    this->vertices = stoi(words.at(0));
    words.erase(words.begin());  // remove the first element as it is the info for number of vertices
    
    // allocate memory for the graph matrix
    this->graph_ptr = new float*[this->vertices];
    
    for (int i= 0; i < this->vertices; i++)
    {
        this->graph_ptr[i] = new float[vertices];
        for(int j = 0; j < this->vertices; j++)
        {
            graph_ptr[i][j] = FloatMax;
        }
    }
    
    int count =1,x=0,y=0;
    for(auto str: words)
    {
        if(count==3)
        {
            graph_ptr[x][y] = stoi(str);
            count =1;
        }
        else if(count == 2)
        {
            y = stoi(str);
            count++;
        }
        else if(count ==1 )
        {
            x = stoi(str);
            count++;
        }
    }
    
}

graph::~graph()
{
    for (int i= 0; i < this->vertices; i++)
    {
        delete graph_ptr[i];   // delete the allocated graph matrix
    }
}

// get the number of vertices
int graph::numVertice(void)
{
    return this->vertices;
}

// get the number of edges
int graph::numEdge(void)
{
    return this->edges;
}

// lists all nodes y such that there is an edge from x to y.
unordered_map<int,float> graph::neighbors (float **G, int x)
{
    int i;
    unordered_map<int, float> neighbours;

    for(i=0;i<this->vertices;i++)
    {
        if(this->adjacent(G,x,i))
        {
            neighbours.insert ( std::pair<int,float>(i,G[x][i]) );
        }
    }
    return neighbours;
}

// tests whether there is an edge from node x to node y.
bool graph::adjacent (float **G, int x, int y)
{
    if(x!=y && G[x][y]!=FloatMax)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// sets the edge value between node x and node y to v
void graph::set_edge_value(float **G,int x, int y, float v)
{
    G[x][y] = v;
}

// get the edge value between node x and node y
float graph::get_edge_value(float **G,int x, int y)
{
    return G[x][y];
}

// get the node value of node x
float graph::get_node_value (float**G,int x)
{
    return G[x][x];
}

// sets the value associated with the node x to a.
void graph::set_node_value( float**G, int x, float a)
{
    G[x][x] = a;
}

// get the graph matrix
float** graph::getGraphMatrix(void)
{
    
    return this->graph_ptr;
}

// output the graph matrix on the console
void graph::display(void)
{
    int i,j;
    for(i = 0;i < this->vertices;i++)
    {
        for(j = 0; j < this->vertices; j++)
            cout<<graph_ptr[i][j]<<"  ";
        cout<<endl;
    }
}


// class for getting minimum spinning tree
class MST
{
private:
    graph *graph_ptr;
    float **graph_matrix_ptr;
    unordered_map<int, vector<int>> path;    // the shortest path from srcNode to the rest of nodes
    vector<float> distance;  // the shortest path from srcNode to the rest of nodes
    float PathCost;   // the averaage shortest path between v and the rest of nodes
    int numOfVertices;   // number of nodes
    vector<edge_t> connectedNode; // return the connected node and its edge value
public:
    MST(graph* g_ptr);
    ~MST();
    bool pathGen(int src);  // getting the path and edges of the spinning tree
    float path_size(int v); // return the minimum sum cost of the spinning tree
    bool node_connected(edge_t node); // check if a node is connected or not
    
};


MST::MST(graph* g_ptr)
{
    this->graph_ptr = g_ptr;
    this->PathCost = 0;
    this->graph_matrix_ptr = this->graph_ptr->getGraphMatrix();
    this->numOfVertices = this->graph_ptr->numVertice();
    for(int i=0;i<this->numOfVertices;i++)
    {
        this->distance.push_back(FloatMax); // the shortest path from srcNode to the rest of nodes
    }
}

MST::~MST(void)
{
    
}

bool MST::pathGen(int src)
{
    // getting the matrix index of the source node
    // example, if the source node is 1, then the matrix index would be 0.
    int srcNode = src-1, curNode = src-1;
    int vertice = this->numOfVertices-1;
    float minCost = FloatMax;
    bool mstExist = false;
    
    unordered_map<int,float> neighbours;
    vector<int> visitedNode;
    edge_t node;
    priority_queue<pair<int,float>, vector<pair<int,float>>, Compare > q;
    
    q.push(make_pair(srcNode, 0));  // add the source node as the initial element in the priority_queue
    
    while(!q.empty() && vertice>0 )
    {
        curNode = q.top().first;
        q.pop();
        neighbours = this->graph_ptr->neighbors(this->graph_matrix_ptr,curNode);
        minCost = FloatMax;
        
        for (unordered_map<int,float>::iterator it=neighbours.begin(); it!=neighbours.end(); ++it)
        {
            
            std::vector<int>::iterator it_vec;
            it_vec = find (visitedNode.begin(), visitedNode.end(), it->first);
            // if the node hasn't been visiied
            if (it_vec == visitedNode.end())
            {
                if( minCost > it->second )
                {
                    minCost = it->second;
                    node.dest = it->first;
                    node.src = curNode;
                    node.weight = minCost;
                    q.push(make_pair(it->first,minCost));
                }
            }
        }
    
        if( node.weight != FloatMax && !this->node_connected(node))
        {
            this->connectedNode.push_back(node);
            node.weight = FloatMax;
            vertice--;
        }
        visitedNode.push_back(curNode);
    }
    
    if( vertice == 0 )
    {
        // get the minimum cost
        for(vector<edge_t>::iterator it = this->connectedNode.begin();it!=this->connectedNode.end();++it)
        {
           this->PathCost += it->weight;
        }
        mstExist = true;
    }
    return mstExist;
}


// return the path cost of the spinning tree
float MST::path_size(int v)
{
    if( this->pathGen(v) )
    {
        
        cout<<"\n"<<"the path cost of the spinning tree is \n";
        cout<<this->PathCost;
        
        cout<<"\n"<<"number of edges: "<<this->connectedNode.size()<<"\n";
        cout<<"\n"<<"the path/edges of the spinning tree is:\n";
        for(vector<edge_t>::iterator it = this->connectedNode.begin();it!=this->connectedNode.end();++it)
        {
            cout<<"\n src: " << it->src<<" dest: "<<it->dest<<" edge: "<<it->weight<<"\n";
        }
    }
    else
    {
        cout<<"spinning tree doesn't exist\n";
    }
    return this->PathCost;
}


// check if a node is connected
bool MST::node_connected(edge_t node)
{
  bool connected = false;
  for(vector<edge_t>::iterator it = this->connectedNode.begin();it!=this->connectedNode.end();++it)
  {
      if(node.dest == it->dest && node.src == it->src)
      {
          connected = true;
          break;
      }
      
      if(node.dest == it->src && node.src == it->dest)
      {
          connected = true;
          break;
      }
  }
 return connected;
}

int main(int argc, const char * argv[])
{
    graph graph1("test.txt");
    graph *graph_ptr;
    graph_ptr = &graph1;
    graph1.display();
    MST mst(graph_ptr);
    mst.path_size(1);
}


