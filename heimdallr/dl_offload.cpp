
#include "heimdallr.h"

// Local globals.

int controllinoSocket;
Eigen::Vector4d next_offload;
Eigen::Vector4d last_offload = Eigen::Vector4d::Constant(0.01);
int offloads_done = 0;
int search_ix = 0;
int search_length = 0;
int search_dl = 0;
double search_delta = 0.5;
double search_start = 0.0;
double hfo_offsets[N_TEL] = {0.0, 0.0, 0.0, 0.0};

// Initialize global ZMQ variables for MultiDeviceServer
zmq::context_t mds_zmq_context(1);
zmq::socket_t mds_zmq_socket(mds_zmq_context, zmq::socket_type::req);
std::string mds_host_str = "tcp://172.16.8.6:5555";
bool mds_zmq_initialized = false;

void init_mds_zmq() {
    if (!mds_zmq_initialized) {
        mds_zmq_socket.connect(mds_host_str);
        mds_zmq_initialized = true;
    }
}

std::string send_mds_cmd(const std::string& message) {
    init_mds_zmq();
    mds_zmq_socket.send(zmq::buffer(message), zmq::send_flags::none);
    zmq::message_t reply;
    auto result = mds_zmq_socket.recv(reply, zmq::recv_flags::none);
    if (!result.has_value()) {
        throw std::runtime_error("Timeout or error receiving reply from MDS.");
    }
    return std::string(static_cast<char*>(reply.data()), reply.size());
}

// Set the delay line offsets (from the servo loop). Units are microns of OPD.
void set_delay_lines(Eigen::Vector4d dl) {
    next_offload=dl;
    offloads_to_do++;
}

void add_to_delay_lines(Eigen::Vector4d dl) {
    // This function adds the delay line values to the current delay line values.
    // The value is in K1 wavelengths.
    next_offload += dl;
    offloads_to_do++;
}

// Set one delay line value. !!! Code needs neatening as this should be in heimdallr.cpp
void set_delay_line(int dl, double value) {
    // This function sets the delay line value for a specific telescope.
    // The value is in K1 wavelengths.
    if (dl < 0 || dl >= N_TEL) {
        std::cout << "Delay line number out of range" << std::endl;
        return;
    }
    next_offload[dl] = value;
    offloads_to_do++;
}

void start_search(uint search_dl_in, double start, double stop, double rate) {
    // This function sets the search parameters for the delay line.
    search_ix = 0;
    search_length = (int)((stop - start) / rate);
    search_dl = search_dl_in;
    search_delta = rate;
    search_start = start;
}

void move_piezos(){
    // This function sets the piezo delay line to the stored value.
    char message[20];
    char buffer[64] = { 0 };
    int dl_value;
    for (int i = 0; i < N_TEL; i++) {
        if (last_offload(i) != next_offload(i) + search_offset(i)){
            dl_value = 2048 + (int)( (next_offload(i) + search_offset(i)) / OPD_PER_PIEZO_UNIT);
            sprintf(message, "a%d %d\n", i, dl_value);
            recv(controllinoSocket, buffer, sizeof(buffer), 0);
            //std::cout << "Received from controllino: " << buffer << std::endl;
            send(controllinoSocket, message, strlen(message), 0);
            std::cout << "Sending to controllino: " << message << std::endl;
            usleep(1000);
        }
    }
    last_offload = next_offload + search_offset;
    offloads_done++;
}

void move_hfo(){
    // This function sets the piezo delay line to the stored value.
    for (int i = 0; i < N_TEL; i++) {
        if ( std::abs(last_offload(i)  - next_offload(i) - search_offset(i)) > 1) {
            // Set the delay line value for the current telescope (value in mm of physical motion)
            double dl_value = hfo_offsets[i] - (next_offload(i) + search_offset(i)) * 0.0005;
            std::string message = "moveabs HFO" + std::to_string(i+1) + " " + std::to_string(dl_value);
            fmt::print(send_mds_cmd(message));
        }
        last_offload(i) = next_offload(i) + search_offset(i);
    }
    offloads_done++;
}

// The main thread function
void dl_offload(){
    // Connect to MDS and find the delay line positions.
    for (int i = 0; i < N_TEL; i++) {
        std::string message = "read HFO" + std::to_string(i+1);
        std::string reply = send_mds_cmd(message);
        //fmt::print(reply);
        hfo_offsets[i] = std::stod(reply);
        fmt::print("HFO{} offset: {}\n", i, hfo_offsets[i]);
    }

    // Connect to the Controllino
    controllinoSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(23);
    serverAddress.sin_addr.s_addr = inet_addr("172.16.8.200");
    connect(controllinoSocket, (struct sockaddr*)&serverAddress,
            sizeof(serverAddress));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(controllinoSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    set_delay_lines(Eigen::Vector4d::Zero());

    while (keep_offloading) {
        // Wait for the next offload
        usleep(100000);

        if (search_ix < search_length) {
            double max_snr = 0.0;
            // Check all baselines associated with the current delay line,
            // using beam_baselines[search_dl]
            for (int i = 0; i < N_TEL-1; i++) {
                // Get the SNR for the current baseline
                double snr = baselines.pd_snr(i);
                if (snr > max_snr) {
                    max_snr = snr;
                }
            }
            fmt::print("Search beam max SNR: {}\n", max_snr);
            // Check if the SNR is above the threshold
            if (max_snr > 5.0) {
                // Set the delay line value to the current search offset
                //next_offload(search_dl) = search_start + search_ix * search_delta;
                
                // Set the search value to the current search offset !!! TODO
                //search_offset(search_dl) = search_start + search_ix * search_delta;
                // Finish the search by setting search_length to 0
                search_length = 0;
            } else {
                // Move the piezo to the next position
                //search_offset(search_dl) = search_start + search_ix * search_delta;
                next_offload(search_dl) = search_start + search_ix * search_delta;
                // Indicate that there is another piezo offload to do.
                offloads_to_do++;
                search_ix++;
            }
        } else search_offset(search_dl) = 0.0;
        if (offloads_to_do > offloads_done) {
            if (delay_line_type == "piezo") {
                // Move the piezo delay line to the next position
                move_piezos();
            } else if (delay_line_type == "hfo") {
                // Move the delay line to the next position
                move_hfo();
            } 
            // Send the delay line values to the controllino
            std::cout << "Sent delay line values: " << next_offload.transpose() << std::endl;
        } 
    }
    close(controllinoSocket);
}

