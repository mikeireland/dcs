#include <commander/commander.h>
#include <iostream>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <fstream>

#include <string>

namespace co = commander;

int add(int i, int j) {
    return i + j;
}

// A custom type.
struct Status
{
    std::size_t code;
    std::string message;
};

// A function that return a custom type.
Status get_status() {
    return {42, "Nominal"};
}

struct configuration {
    int a;
    float b;
    std::string c;
};

// convert configuration to and from json
namespace nlohmann {
    template <>
    struct adl_serializer<configuration> {
        static void to_json(json& j, const configuration& p) {
            j = json{{"a", p.a}, {"b", p.b}, {"c", p.c}};
        }

        static void from_json(const json& j, configuration& p) {
            j.at("a").get_to(p.a);
            j.at("b").get_to(p.b);
            j.at("c").get_to(p.c);
        }
    };
}

struct centroid {
    double x;
    double y;
};

namespace nlohmann {
    template <>
    struct adl_serializer<centroid> {
        static void to_json(json& j, const centroid& c) {
            j = json{{"x", c.x},{"y", c.y}};
        }

        static void from_json(const json& j, centroid& c) {
            j.at("x").get_to(c.x);
            j.at("y").get_to(c.y);
        }
    };
}



std::string test(centroid pos){

    std::string retval = "received centroid";

    std::cout << pos.x << std::endl;
    std::cout << pos.y << std::endl;
    
    return retval;
}




void set_configuration(configuration c, int d) {
    fmt::print("a: {}, b: {}, c: {}, d: {}\n", c.a, c.b, c.c, d);
}

// This code allow json to be converted to a custom type and vice versa.
// For more information, see: https://github.com/nlohmann/json#arbitrary-types-conversions
namespace nlohmann {
    template <>
    struct adl_serializer<Status> {
        static void to_json(json& j, const Status& p) {
            j = json{{"code", p.code}, {"message", p.message}};
        }

        static void from_json(const json& j, Status& p) {
            j.at("code").get_to(p.code);
            j.at("message").get_to(p.message);

        }
    };
}

void overloaded_function(int i) {
    fmt::print("int: {}\n", i);
}

void overloaded_function(float f) {
    fmt::print("float: {}\n", f);
}

COMMANDER_REGISTER(m)
{
    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("add", add, "A function that adds two numbers");

    // You still can register an overloaded function by providing the signature as shown below.
    m.def("overloaded_function", static_cast<void (*)(int)>(overloaded_function), "An overloaded function");

    // You can register a function that takes or returns custom type as long as
    // the types are convertible to json.
    m.def("status", get_status, "Get the status");


    m.def("set_configuration", set_configuration, "Set the configuration");
    
    m.def("test", test, "test");
}

int main(int argc, char* argv[]) {
    std::string TCPString = "tcp://127.0.0.1:5555";
    char TCPCharArr[TCPString.length() + 1];
    std::strcpy(TCPCharArr, TCPString.c_str());

    // Argc/Argv to turn into the required server input
    argc = 3;
    char* argv_new[3];
    argv_new[1] = (char*)"--socket";
    argv_new[2] = TCPCharArr;

    // Start server!
    co::Server s(argc, argv_new);

    s.run();
}
