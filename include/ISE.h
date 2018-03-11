//
// Created by markz on 2018-01-23.
//
#include "lib.h"
#ifndef AI_IMP_ISE_H
#define AI_IMP_ISE_H

//class Node;
pair<double, double> samples_mean(vector<int> &seeds, int r);
void samples(vector<int> &seeds, int r, vector<double> &samples);
int one_WC_sample(vector<int> &seeds);
int one_LT_sample(vector<int> &seeds, vector<int> status);
void influence_neighbour(int vertex, unordered_map<int, double> &impact, unordered_map<int, double> &gate,
                         vector<int> &status, int &influence_area);
void activate(vector<int> &seeds);
void inactivate(vector<int> &seeds);
void getConfidences(vector<int> &seeds, int r, priority_queue<Node> &confidences);
double getConfidence(Vint &seeds, double converged);
pair<double, double> interval(vector<double> &sample);
bool convergence(pair<double, double> &interval);

#endif //AI_IMP_ISE_H
