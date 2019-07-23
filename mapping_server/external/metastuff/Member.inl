#include <stdexcept>

namespace meta
{

template <typename Class, typename T>
Member<Class, T>::Member(const char* name, member_ptr_t<Class, T> ptr, bool optional) :
    name(name),
    ptr(ptr),
    hasMemberPtr(true),
    refGetterPtr(nullptr),
    refSetterPtr(nullptr),
    valGetterPtr(nullptr),
    valSetterPtr(nullptr),
    nonConstRefGetterPtr(nullptr),
    optional(optional)
{ }

template <typename Class, typename T>
Member<Class, T>::Member(const char* name, ref_getter_func_ptr_t<Class, T> getterPtr, ref_setter_func_ptr_t<Class, T> setterPtr, bool optional) : 
    name(name),
    ptr(nullptr),
    hasMemberPtr(false),
    refGetterPtr(getterPtr),
    refSetterPtr(setterPtr),
    valGetterPtr(nullptr),
    valSetterPtr(nullptr),
    nonConstRefGetterPtr(nullptr),
    optional(optional)
{ }

template <typename Class, typename T>
Member<Class, T>::Member(const char* name, val_getter_func_ptr_t<Class, T> getterPtr, val_setter_func_ptr_t<Class, T> setterPtr, bool optional) :
    name(name),
    ptr(nullptr),
    hasMemberPtr(false),
    refGetterPtr(nullptr),
    refSetterPtr(nullptr),
    valGetterPtr(getterPtr),
    valSetterPtr(setterPtr),
    nonConstRefGetterPtr(nullptr),
    optional(optional)
{ }

template <typename Class, typename T>
Member<Class, T>& Member<Class, T>::addNonConstGetter(nonconst_ref_getter_func_ptr_t<Class, T> nonConstRefGetterPtr)
{
    this->nonConstRefGetterPtr = nonConstRefGetterPtr;
    return *this;
}

template <typename Class, typename T>
const T& Member<Class, T>::get(const Class& obj) const
{
    if (refGetterPtr) {
        return (obj.*refGetterPtr)();
    } else if (hasMemberPtr) {
        return obj.*ptr;
    }
    throw std::runtime_error("Cannot return const ref to member: no getter or member pointer set");
}

template <typename Class, typename T>
T Member<Class, T>::getCopy(const Class& obj) const
{
    if (refGetterPtr) {
        return (obj.*refGetterPtr)();
    } else if (valGetterPtr) {
        return (obj.*valGetterPtr)();
    } else if (hasMemberPtr) {
        return obj.*ptr;
    }
    throw std::runtime_error("Cannot return copy of member: no getter or member pointer set");
}

template <typename Class, typename T>
T& Member<Class, T>::getRef(Class& obj) const
{
    if (nonConstRefGetterPtr) {
        return (obj.*nonConstRefGetterPtr)();
    } else if(hasMemberPtr) {
        return obj.*ptr;
    }
    throw std::runtime_error("Cannot return ref to member: no getter or member pointer set");
}

template <typename Class, typename T>
member_ptr_t<Class, T> Member<Class, T>::getPtr() const {
    if (hasPtr()) {
        return ptr;
    }
    throw std::runtime_error("Cannot get pointer to member: it wasn't set");
}

template<typename Class, typename T>
template <typename V, typename>
void Member<Class, T>::set(Class& obj, V&& value) const
{
    // TODO: add rvalueSetter?
    if (refSetterPtr) {
        (obj.*refSetterPtr)(value);
    } else if (valSetterPtr) {
        (obj.*valSetterPtr)(value); // will copy value
    } else if (hasMemberPtr) {
        obj.*ptr = value;
    } else {
        throw std::runtime_error("Cannot access member: no setter or member pointer set");
    }
}

template <typename Class, typename T>
Member<Class, T> member(const char* name, T Class::* ptr, bool optional)
{
    return Member<Class, T>(name, ptr, optional);
}

template <typename Class, typename T>
Member<Class, T> member(const char* name, ref_getter_func_ptr_t<Class, T> getterPtr, ref_setter_func_ptr_t<Class, T> setterPtr, bool optional)
{
    return Member<Class, T>(name, getterPtr, setterPtr, optional);
}

template <typename Class, typename T>
Member<Class, T> member(const char* name, val_getter_func_ptr_t<Class, T> getterPtr, val_setter_func_ptr_t<Class, T> setterPtr, bool optional)
{
    return Member<Class, T>(name, getterPtr, setterPtr, optional);
}

// read only
template <typename Class, typename T>
Member<Class, T> member(const char* name, ref_getter_func_ptr_t<Class, T> getterPtr, bool optional = false)
{
    return Member<Class, T>(name, getterPtr, nullptr, optional);
}

template <typename Class, typename T>
Member<Class, T> member(const char* name, val_getter_func_ptr_t<Class, T> getterPtr, bool optional  = false)
{
    return Member<Class, T>(name, getterPtr, nullptr, optional);
}

// set only 
template <typename Class, typename T>
Member<Class, T> member(const char* name, ref_setter_func_ptr_t<Class, T> setterPtr, bool optional = false)
{
    return Member<Class, T>(name, nullptr, setterPtr, optional);
}

template <typename Class, typename T>
Member<Class, T> member(const char* name, val_setter_func_ptr_t<Class, T> setterPtr, bool optional = false)
{
    return Member<Class, T>(name, nullptr, setterPtr, optional);
}

} // end of namespace meta
