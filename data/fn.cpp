#include <iomanip>
#include <fstream>
#include <vector>

#include <iostream> // added klh
#include <stdio.h>
#include <string.h>
#include <time.h> 

using namespace std;

int main(int argc, const char *argv[])
{
    if (argc < 2) {
        ::std::cerr << "Usage: " << argv[0] << "<filename>\n";
        return 1;
    }
    /*
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"%d%m%G_%H%M",timeinfo);
    
    std::string name="rrt_" + buffer + ".dat"; // C++11 for std::to_string 
    std::ofstream file(name);
   

  //puts (buffer);
    
    string path="/media/k/3832-3661/APM/";
    
    char str[80];
 // strcpy (str, argv[1]);
    strcpy (str, *path)
    strcat(str, "rrtlog.dat" );
    ::std::ifstream fin( str, ::std::ios::binary);
    //std::ifstream fin("rrtlog.dat", std::ios::binary);
    
    char sto[80];
    strcpy(sto, &path);
    strcat(sto, "rrt_");
    strcat(sto, buffer);
    strcat(sto, "csv");
   
    ::std::ofstream fou( sto, std::ofstream::out );
    
    strcpy (str, argv[1]);
    if(!fin)
    {
        cout << " Error, Couldn't find the file" << "\n"; //std::
        return 0;
    }

    fin.seekg(0, std::ios::end);
    const size_t num_elements = fin.tellg() / sizeof(float);
    fin.seekg(0, std::ios::beg);

    std::vector<float> data(num_elements);
    fin.read(reinterpret_cast<char*>(&data[0]),num_elements*sizeof(float));
    
    //cout << "time,gyrx,gyry,gyrz,rol,pit,yaw,accx,accy,accz,vCmds"<< "\n";
    int count = 0;
    for(size_t i = 0; i < data.size(); ++i) {
//      std::cout << std::fixed << std::setprecision(3) << data[i] << ",";
        set::cout << std::fixed << std::setprecision(3) << data[i] << ",";
        if(count++ > 9) {
            fou << "\n";
            count = 0;
        }
    }
    fin.close();
    fou.close();
    */
    return 0;
}
