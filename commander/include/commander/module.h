#pragma once

#include <commander/argument.h>
#include <commander/function_parser.h>
#include <commander/registry.h>

#include <boost/range/adaptors.hpp>

#include <nlohmann/json.hpp>

#include <fmt/core.h>
#include <unordered_map>



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
        string prefix;
        In in;

        template<typename Fn, typename... Args>
        ModuleObject& def(const string& name, Fn&& fn, const string& description = "", Args&&... args);

    };

    template<typename In>
    ModuleObject(Module&, string, In) -> ModuleObject<In>;

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
        std::unordered_map<string, json_function> functions;
        /// @brief A hash map of all arguments
        std::unordered_map<string, Command> commands;

        Module();

        /**
         * @brief Get the help message.
         *
         * @return string The help message.
         */
        string get_help() const;

        /**
         * @brief Get all command names.
         *
         * @return std::vector<string> A vector of all command names.
         */
        vector<string> command_names() const;

        /**
         * @brief Get the description of a command.
         *
         * @param name The name of the command.
         * @return string The description of the command.
         */
        string description(const string& name) const;

        json signature(const string& name) const;

        json arguments(const string& name) const;

        json return_type(const string& name) const;

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
        Module& def(const string& name, Fn&& fn, const string &description = "", Args&&... args)
        {
            Command command{description}; //, parse_args(fn, std::forward<Args>(args)...)};

            // Adds all arguments to the command
            (command.arguments.push_back(args), ...);

            functions.emplace(name, parse(std::forward<Fn>(fn), command));
            commands.emplace(name, command);

            return *this;
        }

        template<typename T>
        auto instance(string name)
        {
            auto static_instance = []() -> T& {
                static T instance;
                return instance;
            };
            return ModuleObject{*this, name, static_instance};
        }

        template<typename Instance>
        auto instance(string name, Instance&& instance)
        {
            return ModuleObject{*this, name, instance};
        }

        json execute(const string& name, const json& args);
    };

    template<typename Instance>
    template<typename Fn, typename... Args>
    ModuleObject<Instance>& ModuleObject<Instance>::def(const string& name, Fn&& fn, const string& description, Args&&... args)
    {
        Command command{description}; //, parse_args(fn, std::forward<Args>(args)...)};

        // Adds all arguments to the command
        (command.arguments.push_back(args), ...);

        auto full_name = fmt::format("{}.{}", prefix, name);

        module_.functions.emplace(full_name, parse(std::forward<Fn>(fn), in, command));
        module_.commands.emplace(full_name, command);

        return *this;
    }

} // namespace commander
