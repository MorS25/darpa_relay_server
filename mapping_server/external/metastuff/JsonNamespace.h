// Embed these functions in namespace of class

template <typename BasicJsonType, typename T>
void to_json(BasicJsonType& j, const T& t)
{
  ::to_json(j,t);
}

template <typename BasicJsonType, typename T>
void from_json(const BasicJsonType& j, T& t)
{
  ::from_json(j,t);
}

