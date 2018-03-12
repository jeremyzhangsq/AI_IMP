#include "../include/lib.h"
//#include "../im_benchmarking-master/irie/graph.h"


void degree2(int cnum, Vint &candidate){
    double start = clock();
    Vint &nonactive = Gvertices;
    Vdouble degree_1(vnums, 1);
    Vdouble degree_2(vnums, 0);
    for(int r = dnum;r>0;r--) {
        for (int v : nonactive) {
            double degree2 = 0;
            for (Edge &e : Gneighbour[v]) {
                degree2 += e.weight*degree_1[e.vertex];
            }
            degree_2[v] = degree_1[v] + degree2;
        }
        degree_1.swap(degree_2);
    }

    VNode heap(vnums);
    for(int i=0; i<vnums; i++)
    {
        heap[i] = Node(i, degree_1[i]);
    }
    make_heap(heap.begin(), heap.end());
    for(int i=0;i<cnum && !heap.empty();i++)
    {
//        if(i<10)
//            cout << heap[0].upper << endl;
        candidate.push_back(heap[0].vertex);
        pop_heap(heap.begin(),heap.end()); heap.pop_back();
    }
    cout << candidate.size() <<endl;
    printf("heuristic:%fs\n",time_by(start));
}

void DFS2(int v, double w, int num, Vint &status, vector<vector<double> > &prob){
    if(num>dnum)
        return;
    status[v] = 1;
    for(Edge &e : Gneighbour[v]){
        if(status[e.vertex] == 0){
            prob[e.vertex].push_back(e.weight*w);
            DFS2(e.vertex, e.weight*w, num+1, status, prob);
        }
    }
    status[v] = 0;
}

void DFS(int v, double w, int num, Vint &status, vector<vector<double> > &prob){
    stack<int> st;
    st.push(v);
    status[v] = 1;
    prob[v].push_back(1);
    vector<vector<Edge>::iterator> viter(vnums);
    for(int i : Gvertices)
        viter[i] = Gneighbour[i].begin();
    int top;
    while (!st.empty()){
        top = st.top();
        auto &iter = viter[top];
        num++;
        while (iter!=Gneighbour[top].end() && status[iter->vertex]!=0)
            iter++;
        if(iter!=Gneighbour[top].end())
        {
            prob[iter->vertex].push_back(iter->weight * prob[top].back());
            if(prob[iter->vertex].back() > 0.01)
            {
                st.push(iter->vertex);
                status[iter->vertex] = 1;
            }
            iter++;
        }else{
            st.pop();
            status[top] = 0;
            iter = Gneighbour[top].begin();
        }
    }
}

void expect(int k, Vint &candidate) {
    double start = clock();
    Vint &nonactive = Gvertices;
    Vint status(vnums);
    vector<vector<double> > prob(vnums);
    Vdouble degree_1(vnums, 0);
    Vdouble degree_2(vnums, 0);
    for(int v : nonactive){
        DFS(v, 1, 0, status, prob);
        for(int i=0;i<vnums;i++){
            double np=1;
            for(double p : prob[i])
                np *= (1-p);
            degree_1[v] += 1-np;
            prob[i].clear();
        }
    }


    VNode heap(vnums);
    for(int i=0; i<vnums; i++)
    {
        heap[i] = Node(i, degree_1[i]);
    }
    make_heap(heap.begin(), heap.end());
    for(int i=0;i<k;i++)
    {
        if(i<10)
        cout << heap[0].upper << endl;
        candidate.push_back(heap[0].vertex);
        pop_heap(heap.begin(),heap.end()); heap.pop_back();
    }
    printf("heuristic:%fs\n",time_by(start));
}


void heuristic_greddy(int k, Vint &candidate) {
    expect(k, candidate);
}

void CELF(int k, Vint &candidate, Vint &seeds) {
    priority_queue <Node> nonactive;
    int r = 100;
    seeds.resize(1);

    for(int c : candidate){
        seeds[0] = c;
        nonactive.push(Node(c, samples_mean(seeds, r).first));
    }
    Node current_node;
    double current_spread = 0;
    seeds.clear();
    for(int i=0;i<k;i++){
        Node best_node(-1,-1);
        seeds.push_back(nonactive.top().vertex);
        while(nonactive.top() != best_node){
            current_node = nonactive.top();
            seeds[i] = current_node.vertex;
            current_node.upper = samples_mean(seeds, r).first - current_spread;
            nonactive.pop();
            nonactive.push(current_node);
            if(best_node < current_node)
                best_node = current_node;
        }
        nonactive.pop();
        seeds[i] = best_node.vertex;
        current_spread += best_node.upper;
    }
}

Vint* CELF_pp(int k, Vint* candidate){
	return NULL;
}

void CI_CELF(int k, Vint &candidate, Vint &seeds) {
    double start = clock();
    priority_queue <Node> nonactive;
    int r = 20000;
    getConfidences(candidate, r, nonactive);
    printf("initialization:%fs\n",time_by(start));
    result["initial"]=to_string(time_by(start));
    Node current_node;
    double current_spread = 0;
    for(int i=0;i<k;i++){
        Node best_node(-1,-1);
        seeds.push_back(nonactive.top().vertex);
        while(nonactive.top() != best_node){
            current_node = nonactive.top();
            seeds[i] = current_node.vertex;
//            cout << current_node.upper << " "<< best_node.upper << " ";
            current_node.upper = getConfidence(seeds, current_spread);
            nonactive.pop();
            nonactive.push(current_node);
            if(best_node < current_node)
                best_node = current_node;
        }
        nonactive.pop();
        seeds[i] = best_node.vertex;
        current_spread += best_node.upper;
    }
}

void SG(int k, Vint &candidate, Vint &seeds){
    double start = clock();
    VNode heap;
    heap.reserve(candidate.size());
    VGraph Graphs;
    getGraph(r, Graphs);
    double Tgraph = clock();
    printf("generate subgraph:%fs\n",time_by(start));
    result["subgraph"]=to_string(time_by(start));
    for(int c : candidate){
        heap.emplace_back(Node(c, BFS(Graphs, c)));
    }
    make_heap(heap.begin(), heap.end());
    printf("initialization:%fs\n",time_by(Tgraph));
    result["initial"]=to_string(time_by(Tgraph));
    int sum=0;
    for(int i=0;i<k;i++){
        Node best_node(-1,-1);
        int num=0;
        while(heap.front() != best_node){
            num++;
            Node &cur_node = heap.front();
            cur_node.upper = BFS(Graphs, cur_node.vertex);
            if(best_node < cur_node)
                best_node = cur_node;
            pop_heap(heap.begin(), heap.end());
            push_heap(heap.begin(),heap.end());
        }
        pop_heap(heap.begin(), heap.end()); heap.pop_back();
        seeds.push_back(best_node.vertex);
        markGraph(Graphs, best_node.vertex);
        sum+=num;
//        cout << seeds.size() << " " << best_node.upper << endl;
    }
    cout << sum << endl;
    int memory = getMemory();
    cout << memory<<"MB" << endl;
    result["memory"]=to_string(memory);
    counter+=k;
}

void getGraph(int n, VGraph &subGraphs){
    subGraphs.reserve(n);
    for(int i=0;i<n;i++){
        subGraphs.emplace_back(Graph(vnums, graph.enums));
        for(int v=0;v<vnums;v++){
            for(Edge &e : Gneighbour[v]){
                if(rand() <= e.weight)
                {
                    subGraphs[i].addEdge(v, e.vertex, e.weight);
                }
            }
        }
        subGraphs[i].cleaning();
    }
}

double BFS(VGraph &subGraphs, int v){
    double spread = 0;
    for(auto &sub : subGraphs){
        if(sub.visit[v])
            continue;
        counter++;
        int u, f = 0, b = 0, n_visit_mark = 0;
        Gq[b++] = v;
        sub.visit[v] = true;
        Gvisit_mark[n_visit_mark++] = v;
        while (f < b){
            u = Gq[f++];
            spread++;
            for(Edge &e : sub.neighbour[u]){
                if(!sub.visit[e.vertex]){
                    sub.visit[e.vertex] = true;
                    Gvisit_mark[n_visit_mark++] = e.vertex;
                    Gq[b++] = e.vertex;
                }
            }
        }
        for(int i=0;i<n_visit_mark;i++)
            sub.visit[Gvisit_mark[i]] = false;
    }
    return spread;
}

void markGraph(VGraph &subGraphs, int v){
    for(auto &sub : subGraphs)
    {
        int u, f = 0, b = 0;
        Gq[b++] = v;
        sub.visit[v] = true;
        while (f < b){
            u = Gq[f++];
            for(Edge &e : sub.neighbour[u]){
                if(!sub.visit[e.vertex]){
                    sub.visit[e.vertex] = true;
                    Gq[b++] = e.vertex;
                }
            }
        }
    }
}

static vector<Vint > SCCG;
static Vint DFN, LOW, st, viter, id, Ssize;
static int top, Tindex, stop, snum;


void tarjan_re(int u){
    int v;
    DFN[u] = LOW[u] = ++Tindex;
    Gvisit[u] = true;
    Gq[++top] = u;
    for(Edge &e : Gneighbour[u]) {
        v = e.vertex;
        if (!DFN[v]) {
            tarjan_re(v);
            if (LOW[v] < LOW[u])
                LOW[u] = LOW[v];
        }else if (Gvisit[v] && DFN[v] < LOW[u])
            LOW[u] = DFN[v];
    }
    if(DFN[u] == LOW[u]){
        SCCG.emplace_back(Vint());
        do{
            v = Gq[top--];
            Gvisit[v] = false;
            SCCG.back().push_back(v);
        }while (v != u);
    }
}

void tarjan(int u){
    int v;
    DFN[u] = LOW[u] = ++Tindex;
    Gvisit[u] = true;
    Gq[++top] = u;
    st[++stop] =u;
    while (stop!=-1){
        u = st[stop];
        for(auto &iter = viter[u];iter < Gneighbour[u].size();iter++) {
            v = Gneighbour[u][iter].vertex;
            if (!DFN[v]) {
                DFN[v] = LOW[v] = ++Tindex;
                Gvisit[v] = true;
                Gq[++top] = v;
                st[++stop] = v;
                break;
            } else if (DFN[v] > DFN[u] && LOW[v] < LOW[u]) {
                LOW[u] = LOW[v];
            } else if (Gvisit[v] && DFN[v] < LOW[u])
                LOW[u] = DFN[v];
        }
        if(u == st[stop]) {
            if (DFN[u] == LOW[u]) {
                do {
                    v = Gq[top--];
                    Gvisit[v] = false;
                    id[v] = snum;
                } while (v != u);
                snum++;
            }
            stop--;
        }
    }
}


void SCC_init(VNode &heap, Vint &candidate){
    DFN.resize(vnums); LOW.resize(vnums); viter.resize(vnums);st.resize(vnums);id.resize(vnums);
    top=-1; Tindex=0;stop=-1;
    for(int i=0;i<vnums;i++){
        if(!DFN[i])
            tarjan(i);
    }
    Graph SCCG(snum,0);
    DFN.assign(vnums,0);
    for(int i=0;i<vnums;i++){
        for(Edge &e : Gneighbour[i])
            SCCG.adduEdge(id[i],id[e.vertex]);
        DFN[id[i]]++;
    }
    cout << snum << endl;
    for(int i=0;i<snum;i++)
    {
        int f=0,b=0,spread=0,n_visit_mark = 0;
        Gq[b++] = i;
        Gvisit[i] = true;
        Gvisit_mark[n_visit_mark++] = i;
        while (f<b){
            int u = Gq[f++];
            spread += DFN[u];
            for(int n : SCCG.uneighbour[u]){
                if(!Gvisit[n]){
                    Gq[b++] = n;
                    Gvisit[n] = true;
                    Gvisit_mark[n_visit_mark++] = n;
                }
            }
        }
        LOW[i] = spread;
        for(int j=0;j<n_visit_mark;j++)
            Gvisit[Gvisit_mark[j]] = false;
    }

    for(int i : candidate)
        heap.emplace_back(Node(i, LOW[id[i]]));
    Vint().swap(DFN);
    Vint().swap(LOW);
    Vint().swap(viter);
    Vint().swap(st);
}

static Vdouble upper;
static Vdouble real_;

void SC_compare_initial(VNode &heap, Vint &candidate) {
    upper.resize(vnums); real_.resize(vnums);
    VNode different;
    if(dnum == 1)
    {
        double inf;
        for(int v : candidate) {
            inf = Gneighbour[v].size();
            for(Edge &e : Gneighbour[v]){
                inf+=Gneighbour[e.vertex].size();
            }
            inf*=r;
            upper[v] = inf;
        }
    }

    int f, b, u, n_visit_mark;
    double sum, spread;
    counter = candidate.size()*r;
    for(int v : candidate){
        sum = 0;
        for(int j=0;j<r;j++)
        {
            Gvisit[v] = true; f = 0; b = 0; spread = 0; n_visit_mark = 0;
            Gvisit_mark[n_visit_mark++] = v;
            Gq[b++] = v;
            while (f < b){
                u = Gq[f++];
                spread++;
                for(Edge &e : Gneighbour[u])
                {
                    int n = e.vertex;

                    double w = e.weight;
                    if(!Gvisit[n] && rand() <= w)
                    {
                        Gvisit[n] = true;
                        Gvisit_mark[n_visit_mark++] = n;
                        Gq[b++] = n;
                    }
                }
            }
            sum += spread;
            if(v==125950)
            {
                cout<<spread<<endl;
            }
            for(int i=0;i<n_visit_mark;i++)
                Gvisit[Gvisit_mark[i]] = false;
        }
        heap.emplace_back(Node(v,sum));
        real_[v] = sum;
        different.emplace_back(Node(v,real_[v]/upper[v]));
    }
    make_heap(different.begin(),different.end());
    for(int i=0;i<30;i++){
        Node v = different.front();
        pop_heap(different.begin(),different.end()); different.pop_back();
        printf("%d\t %.1f\t %.1f\t %.1f\n",v.vertex,real_[v.vertex],upper[v.vertex],v.upper);
    }
}

void SC_ci_initial(VNode &heap, Vint &candidate) {
    upper.resize(vnums); real_.resize(vnums);
    int f, b, u, n_visit_mark;
    double sum, spread;
    counter = candidate.size();
    Vdouble samples(dnum);
    for(int v : candidate){
        for(int r=0;r<dnum;r++)
        {
            Gvisit[v] = true; f = 0; b = 0; spread = 0; n_visit_mark = 0;
            Gvisit_mark[n_visit_mark++] = v;
            Gq[b++] = v;
            while (f < b){
                u = Gq[f++];
                spread++;
                for(Edge &e : Gneighbour[u])
                {
                    int &n = e.vertex;
                    double &w = e.weight;
                    if(!Gvisit[n] && rand() <= w)
                    {
                        Gvisit[n] = true;
                        Gvisit_mark[n_visit_mark++] = n;
                        Gq[b++] = n;
                    }
                }
            }
            for(int i=0;i<n_visit_mark;i++)
                Gvisit[Gvisit_mark[i]] = false;
            samples[r] = spread;
        }

        pair<double, double> sta = interval(samples);
        sum = sta.second*r;
        heap.emplace_back(Node(v,sum));
        upper[v]=sum;
    }
}

void SC_alpha_initial(VNode &heap, Vint &candidate) {
    upper.resize(vnums); real_.resize(vnums);
    int f, b, u, n_visit_mark;
    double sum, spread;
    counter = candidate.size();
    for(int v : candidate){
        Gvisit[v] = true; f = 0; b = 0; spread = 0; n_visit_mark = 0;
        Gvisit_mark[n_visit_mark++] = v;
        Gq[b++] = v;
        while (f < b){
            u = Gq[f++];
            spread++;
            for(Edge &e : Gneighbour[u])
            {
                int &n = e.vertex;
                double &w = e.weight;
                if(!Gvisit[n] && rand() <= w)
                {
                    Gvisit[n] = true;
                    Gvisit_mark[n_visit_mark++] = n;
                    Gq[b++] = n;
                }
            }
        }
        for(int i=0;i<n_visit_mark;i++)
            Gvisit[Gvisit_mark[i]] = false;

        sum = spread*r*dnum;
        heap.emplace_back(Node(v,sum));
        upper[v]=sum;
    }
}

void SC_upper_initial(VNode &heap, Vint &candidate) {
    upper.resize(vnums); real_.resize(vnums);

    if(dnum == 1)
    {
        double inf;
        for(int v : candidate) {
            inf = Gneighbour[v].size()*r;
            heap.emplace_back(Node(v, inf));
            upper[v] = inf;
        }
        return;
    }
    if(dnum == 2){
        double inf;
        for(int v : candidate) {
            inf = Gneighbour[v].size();
            for(Edge &e : Gneighbour[v]){
                inf+=e.weight*Gneighbour[e.vertex].size();
            }
            inf*=r;
            heap.emplace_back(Node(v, inf));
            upper[v] = inf;
        }
        return;
    }

    int f, b, u, n_visit_mark;
    double sum, spread;
    Vint layer(vnums);
    counter = candidate.size()*r;
    for(int v : candidate){
        Gvisit[v] = true; f = 0; b = 0; spread = 0; n_visit_mark = 0;
        Gvisit_mark[n_visit_mark++] = v;
        Gq[b++] = v;
        layer[v] = 1;
        while (f < b){
            u = Gq[f++];
            spread++;
            if(layer[u]>dnum)
                continue;
            for(Edge &e : Gneighbour[u])
            {
                int &n = e.vertex;
                double &w = e.weight;
                if(!Gvisit[n])
                {
                    Gvisit[n] = true;
                    Gvisit_mark[n_visit_mark++] = n;
                    Gq[b++] = n;
                    layer[n] = layer[u]+1;
                }
            }
        }
        sum = spread*r;
        for(int i=0;i<n_visit_mark;i++)
            Gvisit[Gvisit_mark[i]] = false;

        heap.emplace_back(Node(v,sum));
        upper[v]=sum;
    }
}

// ir upper bound (version 1)
void IR_upper_initial_v1(VNode &heap, Vint &candidate){
    upper.resize(vnums, 1);
    Vdouble degree_2(vnums, 0);

    for(int r = dnum;r>0;r--) {
        for (int v : candidate) {
            double degree2 = 0;
            for (Edge &e : Gneighbour[v]) {
                // calculate expection
                degree2 += e.weight/RAND_MAX*upper[e.vertex];
            }
            degree_2[v] = 1 + degree2;
        }
        upper.swap(degree_2);
    }

    for(int v : candidate) {
        upper[v]*=r;
        heap.emplace_back(Node(v,upper[v]));
    }

}
// ir upper bound (version 2 + eliminate oscillation)
void IR_upper_initial_v2(VNode &heap, Vint &candidate){
    upper.resize(vnums, 1);
    Vdouble degree_2(vnums, 0);
    int count = 0;
    double whole_residual=0;
    double whole_origin=0;
    for(int r = dnum;r>0;r--) {
        for (int v : candidate) {
            double degree2 = 0;
            double residual = 0;
            for (Edge &e : Gneighbour[v]) {
                // calculate expection
                degree2 += e.weight/RAND_MAX*upper[e.vertex];
                // elinimate layer1 circle
                if(r==dnum-1){
                    for(Edge &r :Gneighbour[e.vertex]){
                        if(r.vertex==v){
                            residual+=(e.weight/RAND_MAX)*(r.weight/RAND_MAX);
                            count++;
                            break;
                        }
                    }
                }
            }
            whole_origin+=degree2;
            whole_residual+=residual;
            degree2 -= residual;
            degree_2[v] = 1 + degree2;
        }
        upper.swap(degree_2);
    }
    printf("single cycles: %d\n",count);
    printf("origin total spread:%f\tcycle:%f\tratio:%f\n",whole_origin,whole_residual,whole_residual/whole_origin);
//    for(int v:candidate){
//        double result = eliminate_cycle(v);
//        upper[v]-= result;
//    }

    for(int v : candidate) {
        upper[v]*=r;
        heap.emplace_back(Node(v,upper[v]));
    }

}

double eliminate_cycle(int src){

    Vint stack;
    Vint visit_times;
    Vint pointer;
    visit_times.resize(vnums,0);
    stack.resize(1000,-1);
    pointer.resize(vnums,0);
    int top = 1;
    stack[top] = src;
    visit_times[src] = 1;
    int now;
    double sum = 0;
    double prob;
    while(top>0){
        now = stack[top];
        if(pointer[now]==Gneighbour[now].size() or top>layer){
            visit_times[stack[top]] = 2;
            top--;
        }
        else{
            while (top<=layer and pointer[now]<Gneighbour[now].size()){
                Edge &e = Gneighbour[now][pointer[now]];
                if(visit_times[e.vertex]==1){
//                  calculate chained prob of this cycle
                    prob = 1;
                    for(int i = 1;i<top;i++){
                        for(Edge &a:Gneighbour[stack[i]])
                            if(a.vertex==stack[i+1]){
                                prob*=a.weight/RAND_MAX;
                                break;
                            }
                    }
                    for(Edge &s:Gneighbour[stack[top]]){
                        if(s.vertex==e.vertex){
                            prob*=s.weight/RAND_MAX;
                            break;
                        }

                    }
                    sum+=prob;
                    total_cycle+=1;
                    pointer[now]++;
                    continue;
                }
                else{
                    visit_times[e.vertex] = 1;
                    top++;
                    stack[top] = e.vertex;
                    pointer[now]++;
                    break;
                }
            }
        }
    }
    return sum;
}


void SC_initial(VNode &heap, Vint &candidate) {
    upper.resize(vnums); real_.resize(vnums);
    int f, b, u, n_visit_mark;
    double sum, spread;
    counter = candidate.size()*r;
    for(int v : candidate){
        sum = 0;
        for(int j=0;j<r;j++)
        {
            Gvisit[v] = true; f = 0; b = 0; spread = 0; n_visit_mark = 0;
            Gvisit_mark[n_visit_mark++] = v;
            Gq[b++] = v;
            while (f < b){
                u = Gq[f++];
                spread++;
                for(Edge &e : Gneighbour[u])
                {
                    int &n = e.vertex;
                    double &w = e.weight;
                    if(!Gvisit[n] && rand() <= w)
                    {
                        Gvisit[n] = true;
                        Gvisit_mark[n_visit_mark++] = n;
                        Gq[b++] = n;
                    }
                }
            }
            sum += spread;
            for(int i=0;i<n_visit_mark;i++)
                Gvisit[Gvisit_mark[i]] = false;
        }
        heap.emplace_back(Node(v,sum));
    }
}

double
new_WC_sample(int v, Vdouble &times, Vdouble &remain, unordered_set<int> &infset) {
    counter++;
    int f = 0, b = 0, n_visit_mark = 0;
    double influence_area = 0;
    Gvisit[v] = true;
    Gvisit_mark[n_visit_mark++] = v;
    Gq[b++] = v;
    while(f < b){
        int u = Gq[f++];
        influence_area ++;
        times[u]++;
        infset.insert(u);
        for(Edge &e : Gneighbour[u])
        {
            int &n = e.vertex;
            double &w = e.weight;
            if(!Gvisit[n] && random(w, remain[n]))
            {
                Gvisit[n] = true;
                Gvisit_mark[n_visit_mark++] = n;
                Gq[b++] = n;
            }
        }
    }
    for(int i=0;i<n_visit_mark;i++)
        Gvisit[Gvisit_mark[i]] = false;
    return influence_area;
}

double
static_sample(int v, int r, Vdouble &remain, Vdouble &times, unordered_set<int> &infset) {
    if(Gvisit[v])
        return 0;
    double sum=0;
    for(int i : infset)
        times[i] = 0;
    infset.clear();
    for(int i=0;i<r;i++)
    {
//        if(rand()/double(RAND_MAX) < )
            sum += new_WC_sample(v, times, remain, infset);
    }
    return sum*remain[v];
}

void update(Vdouble &best_times, Vdouble &times, unordered_set<int> &best_infset, unordered_set<int> &infset){
    best_times.swap(times);
    best_infset.swap(infset);
}

void staticCELF(int k, Vint &candidate, Vint &seeds) {
    double start = clock();
    VNode heap;
    heap.reserve(candidate.size());
    Vdouble remain(vnums, 1);
    Vdouble times(vnums, 0);
    unordered_set<int> infset;
//    IR_upper_initial_v1(heap,candidate);
    IR_upper_initial_v2(heap,candidate);
//    SC_initial(heap, candidate);
//    SC_upper_initial(heap, candidate);
//    SC_alpha_initial(heap, candidate);
//    SC_ci_initial(heap, candidate);
//    SC_compare_initial(heap, candidate);
    make_heap(heap.begin(), heap.end());

    Node best_node;
    Vdouble best_times(vnums, 0);
    unordered_set<int> best_infset;
    printf("initialization:%fs\n",time_by(start));
    result["initial"]=to_string(time_by(start));
    int sum=0;

    for(int i=0;i<k;i++){
        best_node = Node(-1,-1);
        int num = 0;
        while(heap.front() != best_node){
            num++;
            Node &cur_node = heap.front();

//            printf("%d %d %f %d %f\n",seeds.size(),cur_node.vertex,cur_node.upper, best_node.vertex, best_node.upper);
            cur_node.upper = static_sample(cur_node.vertex, r, remain, times, infset);
            if(best_node < cur_node)
            {
                best_node = cur_node;
                best_times.swap(times);
                best_infset.swap(infset);
            }

            pop_heap(heap.begin(), heap.end());
            push_heap(heap.begin(),heap.end());

        }

        pop_heap(heap.begin(),heap.end()); heap.pop_back();
        seeds.push_back(best_node.vertex);
        for(int j : best_infset){
            remain[j] -= best_times[j]/r;
            if(remain[j] <= 0)
                Gvisit[j] = true;
        }

        sum+=num;
//        Gvisit[best_node.vertex] = true;
        double ratio = best_node.upper/upper[best_node.vertex];
        printf("%d\t %d\t %d\t %.1f\t %.1f\t %.2f\n",(int)seeds.size(), num, best_node.vertex,best_node.upper,upper[best_node.vertex],ratio);
//        printf("%d\t %d\t %.1f\n",(int)seeds.size(), num, best_node.upper);
//        cout << seeds.size() <<" " << num << endl;
    }

    cout << "heap adjust:"<<sum << endl;
    result["heap_adjust"] = to_string(sum);
    int memory = getMemory();
    printf("%.2fGB\n",memory/double(1024));
    result["memory"]=to_string(memory);
}