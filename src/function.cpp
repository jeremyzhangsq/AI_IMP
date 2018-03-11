#include "../include/lib.h"

void read_network(string file_path) {
    FILE* fin=fopen(file_path.c_str(),"r");
    assert(fin != nullptr);
    int num = 0, vi, vj;
    double w;
    int i = fscanf(fin,"%d%d", &vi,&vj);
    assert(i==2);
    graph = Graph(vi,vj);
    vnums = graph.vnums;
    while (fscanf(fin,"%d%d%lf", &vi,&vj,&w)==3)
    {
        if (map.count(vi) != 0)
            vi = map[vi];
        else {
            map.insert({ vi, num });
            anti_map.insert({ num, vi });
            vi = num;
            num++;
        }
        if (map.count(vj) != 0)
            vj = map[vj];
        else {
            map.insert({ vj, num });
            anti_map.insert({ num, vj });
            vj = num;
            num++;
        }
        if(type == "IC")
            w = p*RAND_MAX;
        else
            w = p*w*RAND_MAX;
        if(vi!=vj)
            graph.addEdge(vi, vj, w);
    }
    graph.cleaning();
    fclose(fin);
}

void read_seeds(string file_path, Vint &seeds) {
    FILE* fin = fopen(file_path.c_str(), "r");
    assert(fin != nullptr);
    string line;
    int s;
    while (fscanf(fin,"%d",&s) == 1){
        seeds.push_back(map[s]);
    }
    fclose(fin);
}

double time_by(double start){
    return (clock()-start)/CLOCKS_PER_SEC;
}

void print_vector(Vint* data){
    for(int i : *data){
        cout << i << " " <<endl;
    }
}

void parameter(int argc, char** argv, unordered_map<string, string> &map){
    for(int i=1;i<argc;i+=2){
        map.insert({argv[i], argv[i+1]});
    }
}

bool random(double &w, double &remain){
    return rand() < w*remain;
}

int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int getMemory(){ //Note: this value is in MB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result/1024;
}