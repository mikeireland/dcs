#ifndef COMMANDER_REGISTRY_H
#define COMMANDER_REGISTRY_H

#include <string>
#include <vector>
#include <functional>

namespace commander
{

    struct Module;

    using RegisterFunction = std::function<void(Module&)>;

    std::vector<RegisterFunction>& register_functions_instance();

    void register_function(RegisterFunction fn);

    void register_functions(Module& module_);

} // namespace commander

#define CMD_CAT(A, B) A ## B

#define COMMANDER_REGISTER_IMPL(MOD, CNT)                                   \
namespace commander{namespace{                                              \
struct CMD_CAT(ModuleRegister,CNT){                                         \
    CMD_CAT(ModuleRegister,CNT)(){                                          \
        commander::register_function(register_module);                      \
    }                                                                       \
    static inline void register_module(Module& MOD);                        \
} CMD_CAT(module_register_,CNT);                                            \
}}                                                                          \
void commander::CMD_CAT(ModuleRegister,CNT)::register_module(Module& MOD)

/**
 * This macro creates an entry point that will be invoked when a commander::Module
 * is created. The macro argument defines a variable of type see also commander::Module
 * which can be used to initialize the module.

\rst
    .. code-block:: cpp

        COMMANDER_REGISTER(example, m) {
            // Add command here.
            m.def("foo", []() {
                return "Hello, World!";
            });
        }
\endrst
 */
#define COMMANDER_REGISTER(MOD) COMMANDER_REGISTER_IMPL(MOD, __COUNTER__)

#endif //COMMANDER_REGISTRY_H
