//
// Created by markz on 2018-01-23.
//
#include "lib.h"
#ifndef AI_IMP_GRAPH_H
#define AI_IMP_GRAPH_H
using namespace std;

class Node{
public:
    int vertex;
    double upper;
    double lower;
    Node() = default;
    Node(int v, double p, double l=1) : vertex(v), upper(p), lower(l) {}
    bool operator < (const Node &a) const
    {
        if(a.upper == upper)
            return a.vertex > vertex;
        else
            return a.upper > upper;
    }
    bool operator != (const Node &a) const
    {
        return a.vertex != vertex;
    }
};

class Edge {
public:
    int vertex;
    double weight;
//    Edge* next;
    Edge(int v=-1, double w=0) : vertex(v),weight(w) {};
};

class Graph {
public:
    unsigned int vnums, enums;
    vector<Vdouble> matrix;
    vector<VEdge> neighbour;
    vector<VEdge> last_neighbour;
    vector<unordered_set<int>> uneighbour;
    Vbool visit;
    Vint visit_mark;
    Vint vertices;
    Vint q;
    Graph() = default;
    Graph(unsigned int v, unsigned int e);
    void addEdge(int vi, int vj, double weight);
    void adduEdge(int vi, int vj);
    void cleaning();
};



#endif //AI_IMP_GRAPH_H
