#pragma once

#include "Meta.h"
#include <json/json.hpp>

template <typename BasicJsonType, typename T, typename = std::enable_if_t<meta::isRegistered<T>()>>
void to_json(BasicJsonType& j, const T& t)
{
  meta::doForAllMembers<T>(
      [&j,&t](auto& member)
      {
        if (member.canGetConstRef())
          j[member.getName()] = member.get(t);
        else if (member.hasGetter())
          j[member.getName()] = member.getCopy(t);
      }
      );
}

template <typename BasicJsonType, typename T>
void from_json_inner(const BasicJsonType& j, T& t)
{
  if (!j.is_object())
    throw nlohmann::json::type_error::create(302,
        "Type must be an object, but is " + std::string(j.type_name()));

  // Unpack each registered member
  meta::doForAllMembers<T>(
      [&j,&t](auto& member)
      {
        using MemberT = meta::get_member_type<decltype(member)>;
        auto it = j.find(member.getName());
        if (it != j.end())
        {
          try {
            if (member.canGetRef())
              nlohmann::from_json(*it, member.getRef(t));
              // member.getRef(t) = it->template get<MemberT>();
            else if (member.hasSetter())
              member.set(t, it->template get<MemberT>());
            else
              throw nlohmann::detail::other_error::create(502,
                  "Cannot deserialize, member is read only.");
          } catch (...) {
            std::throw_with_nested(
                std::runtime_error(
                  std::string("Error in field \"") + member.getName() + "\":"
                  ));
          }
        }
        else if (!member.isOptional())
          throw nlohmann::detail::out_of_range::create(403,
              std::string("Required field \"") + member.getName() + "\" not found.");
      }
      );
}

template <typename BasicJsonType, typename T, typename = std::enable_if_t<meta::isRegistered<T>()>>
void from_json(const BasicJsonType& j, T& t)
{
  from_json_inner(j,t);
}

namespace std
{
  template <typename BasicJsonType, typename T>
  void from_json(const BasicJsonType& j, vector<T>& t)
  {
    t.resize(j.size());
    for (size_t i = 0; i < j.size(); ++i)
      nlohmann::from_json(j[i], t[i]);
  }
}

