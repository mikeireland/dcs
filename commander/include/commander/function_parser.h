#ifndef COMMANDER_FUNCTION_PARSER_H
#define COMMANDER_FUNCTION_PARSER_H

#include <boost/callable_traits.hpp>

#include <nlohmann/json.hpp>

#include <tuple>
#include <functional>

namespace commander
{
    using nlohmann::json;
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
    json_function parse_impl(Fn&& fn, type_identity<ReturnType>, type_identity<std::tuple<ParamTypes...>>, std::index_sequence<Is...>) {
        return [fn = std::forward<Fn>(fn)](const json& args) -> json {
            if constexpr (std::is_same_v<ReturnType, void>) {
                std::invoke(fn, args[Is].get<ParamTypes>()...);
                return nullptr;
            } else {
                return std::invoke(fn, args[Is].get<ParamTypes>()...);
            }
        };
    }

    template <typename Fn, typename Instance, typename T, typename ReturnType, typename... ParamTypes, std::size_t... Is>
    json_function parse_impl(Fn&& fn, Instance&& instance, type_identity<ReturnType>, type_identity<std::tuple<T, ParamTypes...>>, std::index_sequence<Is...>) {
        return [fn = std::forward<Fn>(fn), instance = std::forward<Instance>(instance)](const json& args) -> json {
            if constexpr (std::is_same_v<ReturnType, void>) {
                std::invoke(fn, instance(), args[Is].get<ParamTypes>()...);
                return nullptr;
            } else {
                return std::invoke(fn, instance(), args[Is].get<ParamTypes>()...);
            }
        };
    }

} // namespace detail

    template <typename Fn>
    json_function parse(Fn&& fn) {
        using return_type = type_identity<ct::return_type_t<Fn>>;
        using args_type = type_identity<ct::args_t<Fn>>;
        constexpr auto tuple_size = std::tuple_size_v<ct::args_t<Fn>>;

        return detail::parse_impl(std::forward<Fn>(fn),
                                       return_type{},
                                       args_type{},
                                       std::make_index_sequence<tuple_size>{}
        );
    }

    template <typename Fn, typename Instance>
    json_function parse(Fn&& fn, Instance&& instance) {
        using return_type = type_identity<ct::return_type_t<Fn>>;
        using args_type = type_identity<ct::args_t<Fn>>;
        constexpr auto tuple_size = std::tuple_size_v<ct::args_t<Fn>> - 1; // -1 because Instance is not in args_type.

        return detail::parse_impl(std::forward<Fn>(fn),
                                       std::forward<Instance>(instance),
                                       return_type{},
                                       args_type{},
                                       std::make_index_sequence<tuple_size>{}
        );
    }

} // namespace commander

#endif //COMMANDER_FUNCTION_PARSER_H