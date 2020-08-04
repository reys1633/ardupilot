#include <fstream>
#include <iostream>
#include <string>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h> 
#include <stdlib.h>

using namespace std;

int main()
{
  ifstream fin;
  string dir, filepath;
  int num;
  DIR *dp;
  struct dirent *dirp;
  struct stat filestat;
  
  dir = "/media/rlach";
  dp = opendir( dir.c_str() );
  if (dp == NULL)
  {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }
    
  string pathandfn;
  int ndir = 0;
  while ((dirp = readdir( dp )))
  { 
    filepath = dir + "/" + dirp->d_name;
    if (stat( filepath.c_str(), &filestat ))
    {
        cout << filepath << endl;  
    }
    if (S_ISDIR( filestat.st_mode ) )
    {
     // cout << filepath << endl;  
    }
    
    if( ndir == 2 ) {
        cout << filepath << endl;
        pathandfn.append( filepath );
    }
    ndir++;
  }
  pathandfn.append("/rrtlog.dat");
  pathandfn= "/media/rlach/A261-E20D1/APM/BRANDNEWFILENAME_12311970_2359.dat";
    
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer,80,"%m%d%G_%H%M",timeinfo);
    
  string newname="rrt_";
  newname.append(buffer);
  newname.append(".dat");
  cout << newname << endl;
    
  string cmdstr = "cp ";
  cmdstr.append(pathandfn);
  cmdstr.append(" ");
  cmdstr.append(newname);
  cout << cmdstr << endl;
    // make local copy with descriptive name
//int system(const char *command);
  
  const char *com = cmdstr.c_str(); // string to const *char
  system( com );
  
  string csvname="rrt_";
  csvname.append(buffer);
  csvname.append(".csv");
  
  newname="rrt_";
  newname.append(buffer);
  
  cmdstr = "./decoderrt6 ";
  cmdstr.append(newname);
  cmdstr.append(" > ");
  cmdstr.append(csvname);
  com = cmdstr.c_str();
  system(com);
  
  cmdstr = "more ";
  cmdstr.append(csvname);
  com = cmdstr.c_str();
  system(com);
}