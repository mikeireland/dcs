#ifndef COMMANDER_MODULE_H
#define COMMANDER_MODULE_H

#include <commander/function_parser.h>
#include <commander/registry.h>

#include <boost/range/adaptors.hpp>

#include <fmt/core.h>

#include <string>
#include <unordered_map>

namespace commander
{

    struct Argument
    {
        std::string name;
        std::string description;
        // std::string type;
        // std::string default_value;
    };

    struct Command
    {
        std::string description;

        // std::vector<Argument> arguments;

        // std::vector<std::string> arg_names() {
        //     std::vector<std::string> names;
        //     for (auto &arg : arguments) {
        //         names.push_back(arg.name);
        //     }
        //     return names;
        // }
    };

}

// template <> struct fmt::formatter<commander::Argument> {

//   constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
//     // Parse the presentation format and store it in the formatter:
//     auto it = ctx.begin(), end = ctx.end();

//     // Check if reached the end of the range:
//     // if (it != end && *it != '}') throw fmt::format_error("invalid format");

//     // Return an iterator past the end of the parsed range:
//     return it;
//   }

//   // Formats the point p using the parsed format specification (presentation)
//   // stored in this formatter.
//   template <typename FormatContext>
//   auto format(const commander::Command& cmd, FormatContext& ctx) -> decltype(ctx.out()) {
//     // ctx.out() is an output iterator to write to.
//     return format_to(ctx.out(), "{}", cmd.description);
//   }
// };

template <> struct fmt::formatter<commander::Command> {

  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // Parse the presentation format and store it in the formatter:
    auto it = ctx.begin(), end = ctx.end();

    // Check if reached the end of the range:
    // if (it != end && *it != '}') throw fmt::format_error("invalid format");

    // Return an iterator past the end of the parsed range:
    return it;
  }

  // Formats the point p using the parsed format specification (presentation)
  // stored in this formatter.
  template <typename FormatContext>
  auto format(const commander::Command& cmd, FormatContext& ctx) -> decltype(ctx.out()) {
    // ctx.out() is an output iterator to write to.
    return format_to(ctx.out(), "{}", cmd.description);
  }
};

namespace commander
{

    struct Module;

    /**
     * @brief Struct that old the instance reference and allow to register functions that depend on it.
     *
     * Every function that gonna be registered will receive the instance reference as the first parameter.
     *
     * @tparam In The type of the instance provider.
     */
    template<typename In>
    struct ModuleObject
    {
        Module& module_;
        std::string prefix;
        In in;

        template<typename Fn, typename... Args>
        ModuleObject& def(const std::string& name, Fn&& fn, const std::string& description = "", Args&&... args);

    };

    template<typename In>
    ModuleObject(Module&, std::string, In) -> ModuleObject<In>;

    /**
     * @brief Struct for registering commands.
     *
     * When created, it will gather every registered commands referenced by either COMMANDER_REGISTER(MOD)
     * or commander::register_function.
     * It is also possible to manually register commands by calling the def and instance method.
     *
     * @note Every functions and instances manually registered by commander::Module::def and instance will only be valid for the
     * current instance of the Module.
     *
     * When a function is registered, it will possible to invoke it by calling the execute method. This method will required
     * the arguments as a json array or an empty json object when the function's arity is 0.
     */
    struct Module
    {

        /// @brief A hash map of all commands
        std::unordered_map<std::string, json_function> functions;
        /// @brief A hash map of all arguments
        std::unordered_map<std::string, Command> commands;

        Module():
            functions(),
            commands()
        {
            register_functions(*this);

            def("help", [this]() {
                return get_help();
            });
            def("command_names", [this]() {
                return command_names();
            });
            def("description", [this](std::string name) {
                return description(name);
            });
        }

        std::string get_help() const {
            std::string help;
            for (auto& [name, cmd]: commands)
                help = fmt::format("{}{}: {}\n", help, name, cmd);
            return help;
        }

        std::vector<std::string> command_names() const {
            std::vector<std::string> names; names.reserve(commands.size());
            for(auto& e : commands | boost::adaptors::map_keys)
                names.push_back(e);
            return names;
        }

        std::string description(const std::string& name) {
            return commands.at(name).description;
        }

        /**
         * @brief Register a function.
         *
         * Takes a command name, a callable and a description. The callable will be registered
         * as a function that takes a json array as input and will return a json value as output.
         *
         * @note The callable could be a lambda, a std::function or a function pointer and more.
         * It should be possible to invoke the function by calling std::invoke(fn, ...).
         * Generic functions and object with multiple operator() are not supported.
         *
         * @tparam Fn The type of the callable.
         * @tparam Args The types of the arguments.
         * @param name The name of the command.
         * @param fn The callable.
         * @param description The description of the command.
         * @param args The types of the arguments.
         * @return Module& A reference to the current module.
         */
        template<typename Fn, typename... Args>
        Module& def(const std::string& name, Fn&& fn, const std::string &description = "", Args&&... args)
        {
            Command command{description}; //, parse_args(fn, std::forward<Args>(args)...)};

            functions.emplace(name, parse(std::forward<Fn>(fn)));
            commands.emplace(name, command);

            return *this;
        }

        template<typename T>
        auto instance(std::string name)
        {
            auto static_instance = []() -> T& {
                static T instance;
                return instance;
            };
            return ModuleObject{*this, name, static_instance};
        }

        template<typename Instance>
        auto instance(std::string name, Instance&& instance)
        {
            return ModuleObject{*this, name, instance};
        }

        json execute(const std::string& name, const json& args)
        {
            try {
                return functions.at(name)(args);
            } catch (const std::exception& e) {
                return json{{"error", e.what()}};
            }
        }
    };

    template<typename Instance>
    template<typename Fn, typename... Args>
    ModuleObject<Instance>& ModuleObject<Instance>::def(const std::string& name, Fn&& fn, const std::string& description, Args&&... args)
    {
        Command command{description}; //, parse_args(fn, std::forward<Args>(args)...)};

        auto full_name = fmt::format("{}.{}", prefix, name);

        module_.functions.emplace(full_name, parse(std::forward<Fn>(fn), in));
        module_.commands.emplace(full_name, command);

        return *this;
    }

} // namespace commander

#endif //COMMANDER_MODULE_H