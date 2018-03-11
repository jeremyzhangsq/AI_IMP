#ifndef _GLOBAL_H
#define _GLOBAL_H
#ifdef linuxs
#include<gperftools/profiler.h>
#endif

#define Vdouble vector<double>
#define Vint vector<int>
#define Vbool vector<bool>
#define Vlong vector<long>
#define VEdge vector<Edge>
#define VGraph vector<Graph>
#define VNode vector<Node>
#define Gvisit graph.visit
#define Gvisit_mark graph.visit_mark
#define Gneighbour graph.neighbour
#define Glneighbour graph.last_neighbour
#define Gvertices graph.vertices
#define Gq graph.q

#include <chrono>
#include "stdlib.h"
#include "stdio.h"
#include <ctime>
#include <iostream>
#include <unordered_map>
#include <string>
#include <vector>
#include <set>
#include <random>
#include <fstream>
#include <unordered_set>
#include <stdexcept>
#include <queue>
#include <list>
#include <algorithm>
#include <stack>
#include <numeric>
#include <boost/math/distributions/students_t.hpp>

#include "function.h"
#include "graph.h"
#include "ISE.h"
#include "IMP.h"
#include "staticCELF.h"
//#include "MyDB.h"

using namespace std;
using namespace boost::math;

//enum string {IC,LT,WC};
//enum Algorithm {A_SEED, A_CI, A_CELF, A_SCELF, A_ADD, A_EXPECT, A_SG};
//enum Heuristic {NONE, H_DD, H_EXPECT};

extern Graph graph;
extern string type;
extern string algorithm;
extern string heuristic;

extern unordered_map<int, int> map,anti_map;
extern unordered_map<string, string> result;
extern unsigned int vnums, r, counter;
extern double p, dnum;
extern int layer;
#endif