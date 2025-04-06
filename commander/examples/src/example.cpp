#include <commander/commander.h>
#include <Eigen/Dense>

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <string>

namespace co = commander;

// A test function that returns Eigen::Vector3f
Eigen::Vector3f get_vector(float x, float y, float z) {
    // This function returns a 3D vector.
    return Eigen::Vector3f(x, y, z);
}

// Julien's initial example.
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

// A test function for a vector of floats
void float_vector(std::vector<float> v) {
    fmt::print("vector: {}\n", fmt::join(v, ", "));
}

// A struct including a string
struct configuration {
    int a;
    float b;
    std::string c;
};

struct name_value {
    std::string name;
    double value;
};

// convert name_value to and from json
namespace nlohmann {
    template <>
    struct adl_serializer<name_value> {
        static void to_json(json& j, const name_value& p) {
            j = json{{"name", p.name}, {"value", p.value}};
        }

        static void from_json(const json& j, name_value& p) {
            j.at("name").get_to(p.name);
            j.at("value").get_to(p.value);
        }
    };
}

void set_name_value(name_value nv) {
    fmt::print("name: {}, value: {}\n", nv.name, nv.value);
}

void name_value_vector(std::vector<name_value> nv) {
    for (const auto& n : nv) {
        fmt::print("name: {}, value: {}\n", n.name, n.value);
    }
}

struct command {
    std::string name;
    std::string time;
    std::vector<name_value> parameters;
};

// convert command to and from json
namespace nlohmann {
    template <>
    struct adl_serializer<command> {
        static void to_json(json& j, const command& p) {
            j = json{{"name", p.name}, {"time", p.time}, {"parameters", p.parameters}};
        }

        static void from_json(const json& j, command& p) {
            j.at("name").get_to(p.name);
            j.at("time").get_to(p.time);
            j.at("parameters").get_to(p.parameters);
        }
    };
}

void RTS(command c) {
    fmt::print("command name: {}, time: {}\n", c.name, c.time);
    for (const auto& nv : c.parameters) {
        fmt::print("parameter name: {}, value: {}\n", nv.name, nv.value);
    }
}

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

namespace co = commander;

COMMANDER_REGISTER(m)
{
    using namespace co::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("add", add, "A function that adds two numbers", "i"_arg, "j"_arg = 42);

    // You still can register an overloaded function by providing the signature as shown below.
    m.def("overloaded_function", static_cast<void (*)(int)>(overloaded_function), "An overloaded function");

    // You can register a function that takes or returns custom type as long as
    // the types are convertible to json.
    m.def("status", get_status, "Get the status");

    // Here is an example of an input list that includes a struct. e.g.
    // set_configuration [{"a":1,"b":2.0,"c":"three"},1]
    m.def("set_configuration", set_configuration, "Set the configuration");

    m.def("RTS", RTS, "Receive a Real Time System command");
    m.def("set_name_value", set_name_value, "Set a name value pair");
    m.def("float_vector", float_vector, "A function that takes a vector of floats");
    m.def("name_value_vector", name_value_vector, "A function that takes a vector of name value pairs");

    // Let's see if Eigen works too (it doesn't - leave this commented out!)
    //m.def("get_vector", get_vector, "Get a 3D vector", "x"_arg, "y"_arg, "z"_arg);
}

int main(int argc, char* argv[]) {
    co::Server s(argc, argv);
    fmt::print("Starting server\n");
    s.run();
}
