#pragma once

#include "Meta.h"
#include <pugixml.hpp>

// ============================================================
// Generic XML serialization functions
// ============================================================

template <typename T> void to_xml(pugi::xml_node&, const T&);
template <typename T> void from_xml(const pugi::xml_node&, T&);

// ============================================================
// XML marshaling struct
// ============================================================

template <typename T>
struct XML
{
  static void to(pugi::xml_node& node, const T& t)
  {
    meta::doForAllMembers<T>(
        [&node,&t](auto& member)
        {
          auto child = node.append_child(member.getName());
          if (member.canGetConstRef())
            to_xml(child, member.get(t));
          else if (member.hasGetter())
            to_xml(child, member.getCopy(t));
        }
        );

  }
  static void from(const pugi::xml_node& node, T& t)
  {
    meta::doForAllMembers<T>(
        [&node,&t](auto& member)
        {
          auto child = node.child(member.getName());
          if (child)
          {
            if (member.canGetRef())
              from_xml(child, member.getRef(t));
            else if (member.hasSetter())
            {
              meta::get_member_type<decltype(member)> tmp;
              from_xml(child, tmp);
              member.set(t, tmp);
            }
            else
              throw std::runtime_error("Error: can't deserialize member because it's read only");
          }
        }
        );
  }
};

template <typename T>
void to_xml(pugi::xml_node& node, const T& t)
{
  XML<T>::to(node, t);
}
template <typename T>
void from_xml(const pugi::xml_node& node, T& t)
{
  XML<T>::from(node, t);
}

// ============================================================
// marshalling of string maps
// ============================================================

template <typename V>
struct XML<std::map<std::string, V>>
{
  static void to(pugi::xml_node& node, const std::map<std::string,V>& val)
  {
    for (auto& kv : val)
    {
      auto child = node.append_child(kv.first.c_str());
      to_xml(child, kv.second);
    }
  }
  static void from(const pugi::xml_node& node, std::map<std::string,V>& val)
  {
    for (auto& child : node.children())
      from_xml(child, val[child.name()]);
  }
};

// ============================================================
// Marshalling of basic types
// ============================================================

template <>
struct XML<std::string>
{
  static void to(pugi::xml_node& node, const std::string& val)
  {
    node.text().set(val.c_str());
  }
  static void from(const pugi::xml_node& node, std::string& val)
  {
    val = node.text().as_string();
  }
};

#define XML_BASIC(Type, AsType) \
  template <> \
  struct XML<Type> \
  { \
    static void to(pugi::xml_node& node, const Type& val) \
    { \
      node.text().set(val); \
    } \
    static void from(const pugi::xml_node& node, Type& val) \
    { \
      val = node.text().as_##AsType(); \
    } \
  }

XML_BASIC(double, double);
XML_BASIC(float, float);
XML_BASIC(int64_t, int);
XML_BASIC(int32_t, int);
XML_BASIC(int16_t, int);
XML_BASIC(int8_t,  int);
XML_BASIC(uint64_t, uint);
XML_BASIC(uint32_t, uint);
XML_BASIC(uint16_t, uint);
XML_BASIC(uint8_t,  uint);
XML_BASIC(bool, bool);

#undef XML_BASIC

