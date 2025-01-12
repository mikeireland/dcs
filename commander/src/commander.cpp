#include <commander/commander.h>

namespace commander {

    std::vector<RegisterFunction>& register_functions_instance()
    {
        static std::vector<RegisterFunction> register_functions;
        return register_functions;
    }

    void register_function(RegisterFunction fn)
    {
        register_functions_instance().push_back(fn);
    }

    void register_functions(Module& module)
    {
        for (auto& fn : register_functions_instance()) {
            fn(module);
        }
    }

}