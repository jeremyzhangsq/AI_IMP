//
// Created by markz on 2018-01-23.
//
#include "lib.h"
#ifndef AI_IMP_FUNCTION_H
#define AI_IMP_FUNCTION_H
using namespace std;

void read_network(string file_path);
void read_seeds(string file_path, vector<int> &seeds);
bool random(double &w, double &remain);
void fast_srand(int seed);
int fast_rand();
double myrandom();
double time_by(double start);
void print_vector(vector<int>* data);
void parameter(int argc, char** argv, unordered_map<string, string> &map);
int getMemory();
#endif //AI_IMP_FUNCTION_H
