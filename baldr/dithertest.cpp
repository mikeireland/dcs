// dithercross.cpp
#include <ImageStreamIO.h>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>
#include <cstring>

//g++ dithertest.cpp -std=c++17 -I/usr/include/eigen3 -lImageStreamIO -pthread -lrt -o dithertest
// void writeDM(IMAGE* dmImage, const Eigen::VectorXd& dmCmd) {
//     auto md = dmImage->md;
//     int nelem = md->nelement;
//     if ((int)dmCmd.size() != nelem) {
//         std::cerr << "DM command size mismatch: expected "
//                   << nelem << " got " << dmCmd.size() << "\n";
//         return;
//     }
//     md->write = 1;
//     std::memcpy(dmImage->array.D, dmCmd.data(), sizeof(double)*nelem);
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//     md->cnt0++;
//     md->cnt1 = 0;
//     ImageStreamIO_sempost(dmImage, -1);
//     md->write = 0;
// }


// hmm lets do..
// with lots of prints! 

void updateDMSharedMemory(IMAGE &dmImg, const Eigen::VectorXd &dmCmd) {
    auto *md = dmImg.md;
    int N = md->nelement;

    // 1) Entry log
    std::cout << "[DBG] updateDMSharedMemory() called with dmCmd.size() = "
              << dmCmd.size() << " (expecting " << N << ")\n";

    // 2) Check size
    if ((int)dmCmd.size() != N) {
        std::cerr << "[ERROR] size mismatch: got " << dmCmd.size()
                  << " vs " << N << "\n";
        return;
    }

    // 3) Log a few sample values from the command
    std::cout << "[DBG] dmCmd first 5 vals: ";
    for (int i = 0; i < std::min(5, N); ++i) std::cout << dmCmd[i] << " ";
    std::cout << "\n";

    // 4) Print metadata before write
    std::cout << "[DBG] Before write: cnt0=" << md->cnt0
              << "  cnt1=" << md->cnt1
              << "  write=" << md->write << "\n";

    // 5) Mark write
    md->write = 1;
    std::atomic_thread_fence(std::memory_order_release);

    // 6) Copy the data
    std::memcpy(dmImg.array.D, dmCmd.data(), N * sizeof(double));

    // 7) Bump counters
    md->cnt0++;
    md->cnt1++; //!!! MJI, not sure why this was set to 0.

    // 8) Post semaphore
    int ret = ImageStreamIO_sempost(&dmImg, -1);
    std::cout << "[DBG] ImageStreamIO_sempost returned " << ret << "\n";

    // 9) Clear write flag
    md->write = 0;

    // 10) Print metadata after write
    std::cout << "[DBG] After write: cnt0=" << md->cnt0
              << "  cnt1=" << md->cnt1
              << "  write=" << md->write << "\n";

    // 11) Peek at shared memory contents
    std::cout << "[DBG] dmImg.array.D first 10 vals: ";
    for (int i = 0; i < std::min(10, N); ++i) std::cout << dmImg.array.D[i] << " ";
    std::cout << "\n";
}

int main(int argc, char** argv) {
    // Determine beam_id from argv or default to 2
    int beam_id = 2;
    if (argc > 1) {
        try { beam_id = std::stoi(argv[1]); }
        catch(...) {
            std::cerr << "[WARN] Invalid beam_id \"" << argv[1]
                      << "\", using 2\n";
            beam_id = 2;
        }
    }

    // Form the shared-memory name, e.g. "dm2disp02"
    std::string name = "dm" + std::to_string(beam_id) + "disp02";
    std::string name0 = "dm" + std::to_string(beam_id);
    std::cout << "[INFO] Opening DM SHM \"" << name << "\"\n";

    IMAGE dmImage, dmImage0;
    if (ImageStreamIO_openIm(&dmImage, name.c_str()) != IMAGESTREAMIO_SUCCESS) {
        std::cerr << "[ERROR] Failed to open DM SHM \"" << name << "\"\n";
        return 1;
    }
    if (ImageStreamIO_openIm(&dmImage0, name0.c_str()) != IMAGESTREAMIO_SUCCESS) {
        std::cerr << "[ERROR] Failed to open DM SHM \"" << name0 << "\"\n";
        return 1;
    }


    // Query dimensions
    auto *md = dmImage.md;
    int nx = md->size[0];
    int ny = (md->naxis > 1 ? md->size[1] : 1);
    int N  = md->nelement;
    std::cout << "[INFO] DM grid: " << nx << "x" << ny
              << "  (nelement=" << N << ")\n";

    // Build cross indices (middle row + middle column)
    int cx = nx / 2, cy = ny / 2;
    std::vector<int> crossIdx;
    crossIdx.reserve(nx + ny);
    for (int x = 0; x < nx; ++x) crossIdx.push_back(cy * nx + x);
    for (int y = 0; y < ny; ++y) crossIdx.push_back(y * nx + cx);

    Eigen::VectorXd dmCmd(N);
    int toggle = 0;

    // Loop: write ±0.1 pattern every 500 ms
    while (true) {
        dmCmd.setZero();
        double amp = (toggle++ % 2 ? +0.1 : -0.1);
        for (int idx : crossIdx) dmCmd(idx) = amp;

        std::cout << "[INFO] Writing cross with amplitude " << amp << "\n";
        updateDMSharedMemory(dmImage, dmCmd);

        // Signal the master DM process to update itself.
        ImageStreamIO_sempost(&dmImage0, 1);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    return 0;
}