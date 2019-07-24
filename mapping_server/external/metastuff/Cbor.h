#pragma once

#include <metastuff/Meta.h>
#include <cbor/cbor11.h>

// ----------------------------------------------------------------------
// Primitive types
// ----------------------------------------------------------------------

template<typename BasicCborType, typename T,
  std::enable_if_t< std::is_arithmetic<T>::value, int > = 0
  >
void from_cbor(const BasicCborType& c, T& val)
{
  val = c;
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, bool& val)
{
  val = c.to_bool();
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, std::string& val)
{
  val = c.to_string();
}

template <typename BasicCborType>
void from_cbor(const BasicCborType& c, cbor::binary& val)
{
  val = c.to_binary();
}

template <typename BasicCborType, typename T>
void from_cbor(const BasicCborType& c, std::vector<T>& val)
{
  auto& array = c.to_array();
  val.resize(array.size());
  for (size_t i = 0; i < array.size(); ++i)
    from_cbor(array[i], val[i]);
}

template <typename BasicCborType, typename K, typename V>
void from_cbor(const BasicCborType& c, std::map<K,V>& val)
{
  for (auto& kv : c.to_map())
  {
    K key;
    from_cbor(kv.first, key);
    from_cbor(kv.second, val[key]);
  }
}

// ----------------------------------------------------------------------
// Meta types
// ----------------------------------------------------------------------

template <typename BasicCborType, typename T>
void from_cbor_inner(const BasicCborType& c, T& t)
{
  // Ensure message is a map type
  auto& map = c.to_map();
  // Unpack each registered member
  meta::doForAllMembers<T>(
      [&map,&t](auto& member)
      {
        using MemberT = meta::get_member_type<decltype(member)>;
        auto it = map.find(member.getName());
        if (it != map.end())
        {
          try {
            if (member.canGetRef())
              from_cbor(it->second, member.getRef(t));
            else if (member.hasSetter())
            {
              MemberT val;
              from_cbor(it->second, val);
              member.set(t, val);
            }
            else
            {
              throw std::runtime_error("Cannot deserialize, member is read only.");
            }
          } catch (...) {
            std::throw_with_nested(
                std::runtime_error(
                  std::string("Error in field \"") + member.getName() + "\":"
                  ));
          }
        }
        else if (!member.isOptional())
          throw std::runtime_error(
              std::string("Required field \"") + member.getName() + "\" not found.");
      }
      );
}

template <typename BasicCborType, typename T, typename = std::enable_if_t<meta::isRegistered<T>()>>
void from_cbor(const BasicCborType& j, T& t)
{
  from_cbor_inner(j,t);
}

