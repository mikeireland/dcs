#ifndef COMMANDER_FUNCTION_PARSER_H
#define COMMANDER_FUNCTION_PARSER_H

#include <commander/argument.h>

#include <boost/callable_traits.hpp>

#include <nlohmann/json.hpp>

#include <fmt/core.h>

#include <tuple>
#include <functional>

namespace commander
{
    using json_function = std::function<json(const json& args)>;
    namespace ct = boost::callable_traits;

    /// C++17 version of std::type_identity
    /// Replace by std::type_identity when C++20.
    template< class T >
    struct type_identity {
        using type = T;
    };

    template< class T >
    using type_identity_t = typename type_identity<T>::type;

namespace detail
{


    template <typename Fn, typename ReturnType, typename... ParamTypes, std::size_t... Is>
    json_function parse_impl(Fn&& fn, Command& command, type_identity<ReturnType>, type_identity<std::tuple<ParamTypes...>>, std::index_sequence<Is...>) {


        command.arguments.resize(sizeof...(ParamTypes));
        int pos = 0;
        for (auto& args : command.arguments) {
            if (args.name.empty()) {
                args.name = fmt::format("arg_{}", pos);
            }
            pos++;
        }

        ((command.arguments[Is].type = typename_of<ParamTypes>()), ...);
        command.return_value.type = typename_of<ReturnType>();

        return [fn = std::forward<Fn>(fn), arguments = command.arguments](const json& args) -> json {
            auto get_value_at = [&](std::size_t index) -> json {
                if (index >= args.size()) {
                    auto argument = arguments[index];

                    if (argument.default_value) {
                        return argument.default_value.value();
                    } else {
                        throw std::runtime_error(fmt::format("Missing argument {} at position {}", argument.name, index));
                    }
                }
                return args.at(index);
            };


            if constexpr (std::is_same_v<ReturnType, void>) {
                std::invoke(fn, get_value_at(Is).template get<ParamTypes>()...);
                return nullptr;
            } else {
                return std::invoke(fn, get_value_at(Is).template get<ParamTypes>()...);
            }
        };
    }

    template <typename Fn, typename Instance, typename T, typename ReturnType, typename... ParamTypes, std::size_t... Is>
    json_function parse_impl(Fn&& fn, Instance&& instance, Command& command, type_identity<ReturnType>, type_identity<std::tuple<T, ParamTypes...>>, std::index_sequence<Is...>) {
        command.arguments.resize(sizeof...(ParamTypes));
        int pos = 0;
        for (auto& args : command.arguments) {
            if (args.name.empty()) {
                args.name = fmt::format("arg_{}", pos);
            }
            pos++;
        }

        ((command.arguments[Is].type = typename_of<ParamTypes>()), ...);
        command.return_value.type = typename_of<ReturnType>();

        return [fn = std::forward<Fn>(fn), instance = std::forward<Instance>(instance), arguments = command.arguments](const json& args) -> json {
            auto get_value_at = [&](std::size_t index) -> json {
                if (index >= args.size()) {
                    auto argument = arguments[index];

                    if (argument.default_value) {
                        return argument.default_value.value();
                    } else {
                        throw std::runtime_error(fmt::format("Missing argument {} at position {}", argument.name, index));
                    }
                }
                return args.at(index);
            };

            if constexpr (std::is_same_v<ReturnType, void>) {
                std::invoke(fn, instance(), get_value_at(Is).template get<ParamTypes>()...);
                return nullptr;
            } else {
                return std::invoke(fn, instance(), get_value_at(Is).template get<ParamTypes>()...);
            }
        };
    }

} // namespace detail

    template <typename Fn>
    json_function parse(Fn&& fn, Command& command) {
        using return_type = type_identity<ct::return_type_t<Fn>>;
        using args_type = type_identity<ct::args_t<Fn>>;
        constexpr auto tuple_size = std::tuple_size_v<ct::args_t<Fn>>;

        return detail::parse_impl(std::forward<Fn>(fn), command,
                                       return_type{},
                                       args_type{},
                                       std::make_index_sequence<tuple_size>{}
        );
    }

    template <typename Fn, typename Instance>
    json_function parse(Fn&& fn, Instance&& instance, Command& command) {
        using return_type = type_identity<ct::return_type_t<Fn>>;
        using args_type = type_identity<ct::args_t<Fn>>;
        constexpr auto tuple_size = std::tuple_size_v<ct::args_t<Fn>> - 1; // -1 because Instance is not in args_type.

        return detail::parse_impl(std::forward<Fn>(fn),
                                       std::forward<Instance>(instance),
                                       command,
                                       return_type{},
                                       args_type{},
                                       std::make_index_sequence<tuple_size>{}
        );
    }

} // namespace commander

#endif //COMMANDER_FUNCTION_PARSER_H
