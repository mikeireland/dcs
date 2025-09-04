#include "heimdallr.h"
#define OFFLOAD_DT 0.01
#define HFO_DEADBAND 2.05 //Deadband of HFO motion in microns of OPD
// Local globals.

int controllinoSocket;
Eigen::Vector4d next_offload;
// This is non-zero to make sure if using the Piezos the DLs are centred.
Eigen::Vector4d last_offload = Eigen::Vector4d::Constant(0.01);
int offloads_done = 0;
int search_ix = 0;
int search_length = 0;
int search_dl = 0;
double search_delta = 0.5;
double search_start = 0.0;
double hfo_running_average = 0.0; //!!! delete this - simpler now !!!
double hfo_offsets[N_TEL] = {0.0, 0.0, 0.0, 0.0};
auto last_hfo = std::chrono::high_resolution_clock::now();
// Add a local global to track last HFO offset send time
auto last_hfo_offset = std::chrono::high_resolution_clock::now();

// Initialize global ZMQ variables for MultiDeviceServer
zmq::context_t mds_zmq_context(1);
int timeout_ms = 1000;
zmq::socket_t mds_zmq_socket(mds_zmq_context, zmq::socket_type::req);
std::string mds_host_str = "tcp://192.168.100.2:5555";
bool mds_zmq_initialized = false;

void init_mds_zmq() {
    if (!mds_zmq_initialized) {
        mds_zmq_socket.setsockopt(ZMQ_CONNECT_TIMEOUT, &timeout_ms, sizeof(timeout_ms));
        mds_zmq_socket.setsockopt(ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
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
}

void move_hfo(){
    // Only send if more than 0.5 since last offset
    auto now = std::chrono::high_resolution_clock::now();
    double seconds_since_last = std::chrono::duration<double>(now - last_hfo_offset).count();
    if (seconds_since_last < 0.5) {
        fmt::print("Not enough time for a new offload.\n");
        return;
    }
    // Check to find the total offload requested, adding all values of
    // last_offload - (next_offload + search_offset). 
    double total_offload = 0.0;
    for (int i = 0; i < N_TEL; i++) {
        total_offload += std::fabs(last_offload(i) - (next_offload(i) + search_offset(i)));
    }
    // If the total offload is less than the deadband of about  micron, do not send
    if (total_offload < HFO_DEADBAND) return;

    // This function sets the piezo delay line to the stored value.
    for (int i = 0; i < N_TEL; i++) {
        if ( last_offload(i)  != next_offload(i) + search_offset(i) ) {
            // Set the delay line value for the current telescope (value in mm of physical motion)
            double dl_value = hfo_offsets[i] - (next_offload(i) + search_offset(i)) * 0.0005;
            std::string message = "moveabs HFO" + std::to_string(i+1) + " " + std::to_string(dl_value);
            fmt::print(message + "\n");
            fmt::print(send_mds_cmd(message));
            last_offload(i) = next_offload(i) + search_offset(i);
        } 
    }
    last_hfo_offset = now;
}

// The main thread function
void dl_offload(){
    // Connect to MDS and find the delay line positions.
    for (int i = 0; i < N_TEL; i++) {
        std::string message = "read HFO" + std::to_string(i+1);
        std::string reply = send_mds_cmd(message);
        fmt::print(reply);
        hfo_offsets[i] = std::stod(reply);
        fmt::print("HFO{} offset: {}\n", i+1, hfo_offsets[i]);
    }

    // Connect to the Controllino
    controllinoSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(23);
    serverAddress.sin_addr.s_addr = inet_addr("192.168.100.10");
    connect(controllinoSocket, (struct sockaddr*)&serverAddress,
            sizeof(serverAddress));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(controllinoSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    set_delay_lines(Eigen::Vector4d::Zero());

    while (keep_offloading) {
        // Wait for the next offload - 100Hz
        usleep(OFFLOAD_DT*1000000);

        if (search_ix < search_length) {
            double max_snr = 0.0;
            // Check all baselines associated with the current delay line,
            // using beam_baselines[search_dl]
            for (int i = 0; i < N_TEL-1; i++) {
                // Get the SNR for the current baseline
                double snr = baselines.pd_snr(beam_baselines[search_dl][i]);
                if (snr > max_snr) {
                    max_snr = snr;
                }
            }
            fmt::print("Search beam max SNR: {}\n", max_snr);
            // Check if the SNR is above the threshold
            if (max_snr > 10.0) {
                // Set the search value to the current search offset !!! TODO - everything is next_offload for now.
                //search_offset(search_dl) = search_start + search_ix * search_delta;
                // Finish the search by setting search_length to 0
                search_length = 0;
            } else {
                // Move the piezo to the next position
                //search_offset(search_dl) = search_start + search_ix * search_delta;
                next_offload(search_dl) = search_start + search_ix * search_delta*OFFLOAD_DT;
                // Indicate that there is another piezo offload to do.
                offloads_to_do++;
                search_ix++;
            }
        } else search_offset(search_dl) = 0.0;
        if (offloads_to_do > offloads_done) {
            offloads_done = offloads_to_do;
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

