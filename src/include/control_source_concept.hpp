#ifndef FG_EXT_CONTROL_SOURCE_CONCEPT_HPP_INCLUDED
#define FG_EXT_CONTROL_SOURCE_CONCEPT_HPP_INCLUDED

#include <type_traits>

 namespace detail{
   
    template <typename T >
    inline constexpr bool is_control_source_impl = false;
 }

template <typename T>
concept control_source = detail::is_control_source_impl<
   std::remove_cvref_t<T>
>;

#endif // FG_EXT_CONTROL_SOURCE_CONCEPT_HPP_INCLUDED
