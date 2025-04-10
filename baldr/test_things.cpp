



// g++ -std=c++17 -O2 -o test_things test_things.cpp -I/path/to/ImageStreamIO/include -L/path/to/ImageStreamIO/lib -lImageStreamIO -lpthread -lrt
#include <iostream>
#include <cstdlib>
#include <cstdint>   // for uint16_t
#include <ImageStreamIO.h> // Make sure this header is found in your include path

int main() {
    
    const char* shmName = "cred1";

    IMAGE img;

    // Open the shared memory image.
    // This call will invoke ImageStreamIO_read_sharedmem_image_toIMAGE() internally.
    int ret = ImageStreamIO_openIm(&img, shmName);
    if (ret != IMAGESTREAMIO_SUCCESS) {
        std::cerr << "Error: Failed to open shared memory image for " << shmName << std::endl;
        return EXIT_FAILURE;
    }

    // Verify we have valid metadata.
    if (!img.md) {
        std::cerr << "Error: No metadata found in the shared memory image." << std::endl;
        ImageStreamIO_destroyIm(&img);
        return EXIT_FAILURE;
    }

    // Print image metadata: name, dimensions, and total number of elements.
    int nx = img.md->size[0];
    int ny = (img.md->naxis > 1) ? img.md->size[1] : 1;
    std::cout << "Shared memory image: " << img.md->name << std::endl;
    std::cout << "Dimensions: " << nx << " x " << ny;
    if (img.md->naxis > 2)
        std::cout << " x " << img.md->size[2];
    std::cout << std::endl;
    std::cout << "Total elements: " << img.md->nelement << std::endl;

    // Access the pixel data.
    // The data pointer used depends on the datatype. For example, if the image was created with
    // _DATATYPE_UINT16, then use the UI16 member.
    uint16_t* data = img.array.UI16;
    if (data == nullptr) {
        std::cerr << "Error: Data pointer in shared memory is null." << std::endl;
        ImageStreamIO_destroyIm(&img);
        return EXIT_FAILURE;
    }

    // print the first 16 pixel values.
    int nprint = (nx * ny < 16) ? nx * ny : 16;
    std::cout << "First " << nprint << " pixel values: ";
    for (int i = 0; i < nprint; ++i) {
        std::cout << data[i] << " ";
    }
    std::cout << std::endl;


    // Clean up: unmap the shared memory and free the IMAGE structure.
    //ret = ImageStreamIO_destroyIm(&img);
    // if (ret != IMAGESTREAMIO_SUCCESS) {
    //     std::cerr << "Error: Failed to destroy shared memory image." << std::endl;
    //     return EXIT_FAILURE;
    // }

    std::cout << "Shared memory image processed successfully." << std::endl;
    return EXIT_SUCCESS;
}