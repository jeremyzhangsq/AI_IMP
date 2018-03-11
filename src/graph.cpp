#include "../include/lib.h"

Graph::Graph(unsigned int v, unsigned int e) {
    vnums = v;
    enums = e;
    matrix.resize(v);
    if(e == 0)
        uneighbour.resize(v);
    else
    {
        neighbour.resize(v);
        //    last_neighbour.resize(v);
        visit.resize(v, false);
        visit_mark.resize(v, 0);
        q.resize(v);
    }

}

void Graph::addEdge(int vi, int vj, double weight) {
    neighbour[vi].emplace_back(Edge(vj, weight));
//    last_neighbour[vj].emplace_back(Edge(vi, weight));
}

void Graph::adduEdge(int vi, int vj) {
    uneighbour[vi].insert(vj);
//    last_neighbour[vj].emplace_back(Edge(vi, weight));
}

void Graph::cleaning(){
    for(int v=0;v<vnums;v++){
        if(!neighbour[v].empty())
            vertices.push_back(v);
    }
}