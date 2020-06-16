//#ifndef ATTCMD_H
//#define ATTCMD_H
  
class attcmd {
public:
    attcmd();
    ~attcmd();
    void update( float & ypCmd, double tof );
    
private:
    float ypCmd[2];
    
};
