// Created by markz on 2018-01-24.

#include "lib.h"
#ifndef AI_IMP_IMP_H
#define AI_IMP_IMP_H


void degree2(int cnum, Vint &candidate);
void expect(int k, Vint &candidate);
void heuristic_greddy(int k, Vint &candidate);
void IR_upper_initial_v1(VNode &heap, Vint &candidate);
void IR_upper_initial_v2(VNode &heap, Vint &candidate);
double eliminate_cycle(int src);
void CELF(int k, Vint &candidate, Vint &seeds);
Vint CELF_pp();
void CI_CELF(int k, Vint &candidate, Vint &seeds);

//SG
void SG(int k, Vint &candidate, Vint &seeds);
void getGraph(int n, VGraph &Graphs);
double BFS(VGraph &subGraphs, int v);
void markGraph(VGraph &subGraphs, int v);

//SCELF
void influ(int u, int& n_visit_mark, int& b);
double new_WC_sample(int v, Vdouble &times, Vdouble &remain, unordered_set<int> &infset);
double
static_sample(int v, int r, Vdouble &remain, Vdouble &times, unordered_set<int> &infset);
void staticCELF(int k, Vint &candidate, Vint &seeds);
#endif //AI_IMP_IMP_H
