#include <commander/commander.h>

#include <fmt/core.h>

#include <tuple>

struct Gyro {

    Gyro()
    {
        fmt::print("Gyro\n");
    }

    ~Gyro()
    {
        fmt::print("~Gyro\n");
    }

    void update(float new_x, float new_y, float new_z) {
        fmt::print("Update gyro with x: {}, y: {}, z: {}\n", new_x, new_y, new_z);
        x = new_x; y = new_y; z = new_z;
    }

    std::tuple<float, float, float> read() {
        return std::make_tuple(x,y,z);
    }

    float x, y, z;
};


COMMANDER_REGISTER(m)
{

    // You can register a class or a struct using the instance method.
    // Using `instance<Gyro>` will instantiate a new instance of Gyro
    // only when a gyro command is called.
    m.instance<Gyro>("gyro")
        // To insterface a class method, you can use the `def` method.
        .def("update", &Gyro::update, "Update the gyro")
        .def("read", &Gyro::read, "Read the gyro")
        // You can also register a lambda function or any other callable
        // as long as the signature takes a gyro reference as the
        // first argument.
        .def("info", [](Gyro& g) {
            return fmt::format("Gyro info: x: {}, y: {}, z: {}\n", g.x, g.y, g.z);
        })
        // You can also register struct or class members as long as they are public.
        .def("x", &Gyro::x, "Get the x value")
        .def("y", &Gyro::y, "Get the y value")
        .def("z", &Gyro::z, "Get the z value");

}