// dithercross.cpp
#include <ImageStreamIO.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>
#include <cstring>

//g++ dithertest.cpp -std=c++17 -I/usr/include/eigen3 -lImageStreamIO -pthread -lrt -o dithertest
void writeDM(IMAGE* dmImage, const Eigen::VectorXd& dmCmd) {
    auto md = dmImage->md;
    int nelem = md->nelement;
    if ((int)dmCmd.size() != nelem) {
        std::cerr << "DM command size mismatch: expected "
                  << nelem << " got " << dmCmd.size() << "\n";
        return;
    }
    md->write = 1;
    std::memcpy(dmImage->array.D, dmCmd.data(), sizeof(double)*nelem);
    md->cnt0++;
    md->cnt1 = 0;
    ImageStreamIO_sempost(dmImage, -1);
    md->write = 0;
}

int main(int argc, char** argv) {
    //std::string name = "dm" + std::to_string(beam_id) + "disp02"; //(argc>1 ? argv[1] : "dm_rtc");
    int beam_id = 2;
    if (argc > 1) {
        try { beam_id = std::stoi(argv[1]); }
        catch(...) { 
            std::cerr<<"Invalid beam_id \""<<argv[1]<<"\", using 0\n"; 
            beam_id = 0;
        }
    }

    // 2) form the SHM name
    std::string name = "dm" + std::to_string(beam_id) + "disp02";

    //ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id)).c_str());
    IMAGE dmImage;
    if (ImageStreamIO_openIm(&dmImage, name.c_str()) != IMAGESTREAMIO_SUCCESS) {
        std::cerr<<"Failed to open DM SHM \""<<name<<"\"\n"; return 1;
    }

    auto md = dmImage.md;
    int nx = md->size[0], ny = (md->naxis>1?md->size[1]:1);
    int nelem = md->nelement;              // should now be 144
    std::cout<<"DM actuators: "<<nelem<<"\n";

    // build cross‐pattern indices
    int cx = nx/2, cy = ny/2;
    std::vector<int> crossIdx;
    for(int x=0;x<nx;++x) crossIdx.push_back(cy*nx + x);
    for(int y=0;y<ny;++y) crossIdx.push_back(y*nx + cx);

    Eigen::VectorXd dmCmd(nelem);
    int toggle = 0;

    while(true) {
        dmCmd.setZero();
        double amp = (toggle++%2 ? +0.1 : -0.1);   // <<-- use ±0.1 now
        for(int idx:crossIdx) dmCmd(idx) = amp;
        writeDM(&dmImage, dmCmd);
        std::cout<<"Wrote cross with amplitude "<<amp<<"\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}