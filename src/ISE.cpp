//
// Created by markz on 2018-01-23.
//

#include "../include/lib.h"


pair<double, double> samples_mean(Vint &seeds, int r) {
    Vdouble sample;
    samples(seeds, r, sample);
    return interval(sample);
}


void samples(Vint &seeds, int r, Vdouble &sample) {
//    sample.resize(r);
//    Vint status(vnums);
    fill(Gvisit.begin(),Gvisit.end(),false);
    activate(seeds);
    for(int j=0;j<seeds.size();j++)
        Gq[j] = seeds[j];
    for(int i=0;i<r;i++){
       if(type == "LT")
           ;
//           sample.push_back(one_LT_sample(seeds));
       else
       {
           sample.push_back(one_WC_sample(seeds));
       }
    }
    inactivate(seeds);
}

int one_WC_sample(Vint &seeds) {
    int u, f = 0, b = seeds.size(), n_visit_mark = 0, influence_area = seeds.size();
    while(f < b){
        u = Gq[f++];
        for(Edge &e : Gneighbour[u])
        {
            int &n = e.vertex;
            double &w = e.weight;
            if(!Gvisit[n] && rand()<=w)
            {
                Gvisit[n] = true;
                Gvisit_mark[n_visit_mark++] = n;
                Gq[b++] = n;
                influence_area++;
            }
        }
    }
    for(int i=0;i<n_visit_mark;i++)
        Gvisit[Gvisit_mark[i]] = false;
    return influence_area;
}

int one_LT_sample(Vint &seeds, Vint status) {
    int influence_area = seeds.size();
    unordered_map<int, double> gate;
    unordered_map<int, double> impact;
    for(auto s : seeds){
        influence_neighbour(s, impact, gate, status, influence_area);
    }
    return influence_area;
}

void
influence_neighbour(int vertex, unordered_map<int, double> &impact, unordered_map<int, double> &gate,
                         Vint &status, int &influence_area) {
    for(Edge e : graph.neighbour[vertex]){
        int n = e.vertex;
        double w = e.weight;
        if(status[n])
            continue;
        if(impact.count(n) == 0){
            gate.insert({n, rand()});
            impact.insert({n, 0});
        }
        impact[n] += w;
        if(impact[n] >= gate[n]){
            status[n] = 1;
            influence_area += 1;
            influence_neighbour(n, impact, gate, status, influence_area);
        }
    }
}

void activate(Vint &vertices){
    for(auto v : vertices){
        Gvisit[v] = true;
    }
}

void inactivate(Vint &vertices){
    for(auto v : vertices){
        Gvisit[v] = false;
    }
}

void getConfidences(Vint &seeds, int r, priority_queue<Node> &confidences) {
    Vint vertex(1);
    for(int i : seeds)
    {
        vertex[0] = i;
        confidences.push(Node(i, getConfidence(vertex, -1)));
    }
}

double getConfidence(Vint &seeds, double converged) {
    Vdouble sample;
    pair<double, double> p;
    int r = 1000;
    if(converged != -1){
        int temr = 0;
        do{
            temr += r;
            samples(seeds, temr, sample);
            p = interval(sample);
            p.first -= converged;
            p.second -= converged;
        }while (!convergence(p));
//        cout << seeds.size()<< " "<< temr << " " << p.first<< " " << p.second <<endl;
        return p.first;
    } else{
        samples(seeds, r, sample);
        return interval(sample).second;
    }
}

pair<double, double> interval(Vdouble &sample){
    double sum = accumulate(begin(sample), end(sample), 0.0);
    int n = sample.size();
    double mean =  sum / n;
    double var = 0.0;
    for (const double d : sample) {
        var += (d - mean) * (d - mean);
    };
    double std = sqrt(var/(n-1));
    if(std == 0)
        return make_pair(mean, mean);
    const double alpha = 0.01;
    students_t dist(n-1);
    double T = quantile(complement(dist, alpha));
    double w = T * std / sqrt(double(n));
    return make_pair(mean, mean+w);
}

bool convergence(pair<double, double> &interval){
    return (interval.second/interval.first)<1.1;
}