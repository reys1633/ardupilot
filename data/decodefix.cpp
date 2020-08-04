#include <iomanip>
#include <fstream>
#include <vector>

#include <iostream> // added klh
#include <stdio.h>
#include <string.h>
using namespace std;

int main(int argc, const char *argv[])
{
    if (argc < 2) {
        ::std::cerr << "Usage: " << argv[0] << "<filename>\n";
        return 1;
    }
    
    char str[80];
    strcpy (str, argv[1]);
    
    strcat(str, ".dat" );
    
    ::std::ifstream fin( str, ::std::ios::binary);
    //std::ifstream fin("rrtlog.dat", std::ios::binary);
    
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
    
//  cout << "time,gyrx,gyry,gyrz,rol,pit,yaw,accx,accy,accz,vCmds"<< "\n";
    bool firstln = true;
    int count = 1;
    for(size_t i = 0; i < data.size(); ++i) {
  //for(size_t i = 0; i < 100; ++i) {
        
        std::cout << std::fixed << std::setprecision(3) << data[i] << ",";
        if( firstln ) {
            if( count > 10 ) {  // add missing zero
                std::cout << "0.0,\n";
                firstln = false;
                count = 1;
            }
        } else {
            if( count > 12 ) {
                std::cout << "\n";
                count = 1;
            }
        }
        ++count;
    }
    return 0;
}