#include "../include/lib.h"
void run(int argc,char **argv){
    unordered_map<string, string> param;
    parameter(argc, argv, param);
    int fileno, k, cnum, i;

//    MyDB db;
//    db.initDB("120.24.163.35", "root", "933d980084", "experiment");

    string prefix = "./dataset/", postfix = ".txt";
    string filename[] = {"NetHEPT-ud","NetHEPT-d","HepPh-ud","network","GowallaNew","dblp-ud","YouTube-ud","LiveJournal-d","epinions-d"};
    string seed_file = "dataset/seeds.txt";
    string environment,database;

    fileno = stoi(param.count("-f")?param["-f"]:"4");   result["file"] = filename[fileno];
    type = param.count("-t")?param["-t"]:"WC";          result["model"]= type;
    k = stoi(param.count("-k")?param["-k"]:"1000");     result["k"] = to_string(k);
    algorithm = param.count("-a")?param["-a"]:"SCELF";  result["algorithm"]= algorithm;
    heuristic = param.count("-h")?param["-h"]:"NO";     result["heuristic"]= heuristic;
    cnum = stoi(param.count("-c")?param["-c"]:"500");   result["candidate"]= to_string(cnum);
    dnum = stod(param.count("-d")?param["-d"]:"3");     result["depth"]= to_string(dnum);
    r = stoi(param.count("-r")?param["-r"]:"100");      result["r"]= to_string(r);
    i = stoi(param.count("-i") ? param["-i"] : "1");
    p = stof(param.count("-p") ? param["-p"] : "1");    result["possibility"]= to_string(p);
    layer = stoi(param.count("-l") ? param["-l"] : "1");    result["layer"]= to_string(layer);
    environment = param.count("-e")?param["-e"]:"unknow";   result["environment"] = environment;
    database = param.count("-db")?param["-db"]:"y";
    time_t date = time(0);
    char tmpBuf[255];
    strftime(tmpBuf, 255, "%Y%m%d%H%M", localtime(&date)); result["date"] = tmpBuf;

//    srand((unsigned int)time(nullptr));
    double read = clock();
    read_network(prefix+filename[fileno]+postfix);
    printf("read network:%fs\n",time_by(read));

#ifdef linuxs
    ProfilerStart("my.prof");
#endif
    double start = clock();
    Vint seeds;
    Vint candidate;
    if(heuristic == "EX")
        expect(cnum, candidate);
    else if(heuristic == "D2")
        degree2(cnum, candidate);
    else if(heuristic == "NO")
        candidate = graph.vertices;
    else
        throw runtime_error{"error heuristic"};

    if(algorithm == "NO")
        seeds = candidate;
    else if(algorithm == "SEED")
        read_seeds(seed_file, seeds);
    else if(algorithm == "IMM")
        read_seeds(seed_file, seeds);
    else if(algorithm == "CI")
        CI_CELF(k, candidate, seeds);
    else if(algorithm == "CELF")
        CELF(k, candidate, seeds);
    else if(algorithm == "SG")
        SG(k, candidate, seeds);
    else if(algorithm == "SCELF")
        staticCELF(k, candidate, seeds);
    else
        throw runtime_error{"error algorithm"};

#ifdef linuxs
    ProfilerStop();
#endif

//    print_vector(&seeds);
    if(counter!=0){
        printf("total time:%fs\n",time_by(start));
        printf("iteration:%d\n",counter);
        result["total_time"]=to_string(time_by(start));
        result["iteration"]=to_string(counter);
    }

    if(i)
    {
        pair<double, double> sta= samples_mean(seeds, 10000);
        cout << sta.first <<" " << sta.second << endl;
        result["spread"]=to_string(sta.first);
    }

//    if(database=="y")
//        db.addRecord("IMP",result);

}

//void db_test(){
//    MyDB db;
//    db.initDB("120.24.163.35", "root", "933d980084", "experiment");
//    unordered_map<string, string> result;
//    result.insert({"seeds","3"});
//    result.insert({"name","nethept"});
//    db.addRecord("IMP",result);
//}


int main(int argc,char **argv) {
    run(argc,argv);
    return 0;
}