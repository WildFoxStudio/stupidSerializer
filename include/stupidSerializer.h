// //////////////////////////////////////////////////////////////////////////////////////////
// FILE: stupidSerializer.h
//
// AUTHOR: Kirichenko Stanislav
//
// DATE: 14 feb 2025
//
// DESCRIPTION: Stupid simple c++20 templated serialization with abstract concept for archive class.
//
// LICENSE: BSD-2
// Copyright (c) 2025, Kirichenko Stanislav
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions, and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions, and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cassert>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace Fox
{
/**
 * /brief Base interface for serialization.
 */
template<typename Archive_T>
struct IWrap
{
    inline static constexpr std::string_view INHERIT_TOKEN                                          = { "%inherit_from%" };
    virtual std::string_view                 Name() const                                           = 0;
    virtual void                             Serialize(std::reference_wrapper<Archive_T> j) const   = 0;
    virtual void                             Deserialize(std::reference_wrapper<const Archive_T> j) = 0;
};

template<typename Archive_T>
using metaptr = std::shared_ptr<IWrap<Archive_T>>;
template<typename Archive_T>
using meta_list = std::vector<metaptr<Archive_T>>;

namespace utils
{

template<typename T>
struct always_false : std::false_type
{
};

/**
 * /brief Check that reflect() returns a list of reflectable elements.
 */
template<typename T, typename Archive_T>
concept Reflectable = requires(T& t, Archive_T archive)
{
    {
        t.template reflect<Archive_T>(archive)
        } -> std::same_as<meta_list<Archive_T>>;
};

/**
 * /brief Checks if has post load method.
 */
template<typename T>
concept HasPostLoad = requires(T& t)
{
    {
        t.postLoad()
        } -> std::same_as<void>;
};

/**
 * /brief The archive class concept.
 */
template<typename W>
concept ArchiveConcept = requires(W w, const W cw, std::string_view key, int testValue, int outValue)
{
    // Check that contains exists and returns a bool.
    {
        w.contains(key)
        } -> std::convertible_to<bool>;
    {
        cw.contains(key)
        } -> std::convertible_to<bool>;

    // Check that get works for an int.
    {
        w.get(key, outValue)
        } -> std::convertible_to<bool>;

    // Check that set works for an int.
    {
        w.set(key, testValue)
        } -> std::same_as<void>;

    // Check that operator[] returns a new wrapper of the same type.
    {
        w[key]
        } -> std::same_as<W>;
    {
        cw[key]
        } -> std::same_as<W>;
};

template<typename T, typename Wrapper>
concept Archivable = requires(Wrapper w, const T& t, const std::string_view key, T temp, T outValue)
{
    {
        w.get(key, outValue)
        } -> std::convertible_to<bool>;
    {
        w.set(key, t)
        } -> std::same_as<void>;
};

template<typename T>
struct is_std_vector : std::false_type
{
};

template<typename... Args>
struct is_std_vector<std::vector<Args...>> : std::true_type
{
};
}

/**
 * /brief Reflection struct to reflect the class name and recursively othere members.
 * /tparam T The class type.
 * /tparam Archive_T The archive type.
 */
template<typename T>
struct meta
{
    // static_assert(utils::Reflectable<T>);
    T* const t;
};

/**
 * /brief Templated type erasure wrapper for serialization.
 * /tparam T The type to be serialized.
 * /tparam Archive_T The archive used to serialize.
 */
template<typename T, typename Archive_T>
struct CWrap : public IWrap<Archive_T>
{
    static_assert(std::is_same<T, typename std::decay<T>::type>::value, "Must not be reference, array, or function type");
    static_assert(!std::is_pointer<T>::value, "Must not be pointer");

    const std::string_view _name;
    T* const          _t;
    CWrap(std::string_view name, T* const t) : _name(name), _t(t) { assert(_t); };
    std::string_view Name() const override { return _name; }
    virtual void Serialize(std::reference_wrapper<Archive_T> j) const override
    {
        using namespace utils;

        auto& archive{ j.get() };
        if constexpr (Reflectable<T, Archive_T>)
            {
                // Recursive serialization
                Archive_T k{};
                for (const auto& e : _t->reflect(j.get()))
                    {
                        e->Serialize(k);
                    }

                archive.set(_name, k);
            }
        else if constexpr ((std::is_trivial<T>::value || utils::Archivable<T, Archive_T>) && !utils::is_std_vector<T>::value)
            {
                const auto& ref{ *_t };
                archive.set(_name, ref);
            }
        else if constexpr (utils::is_std_vector<T>::value)
            {
                using elementType = typename T::value_type;

                // static_assert(
                //     !std::is_trivial<elementType>::value &&
                //         !utils::Archivable<elementType, Archive_T>,
                //     "Must not have trivial types or archivable types because the
                //     " "archive_t must overload std::vector with any trivial
                //     types.");
                static_assert(Reflectable<elementType, Archive_T>, "Must have a Reflectable type.");

                std::vector<elementType>& vec{ *_t };

                // Recursive serialization
                std::vector<Archive_T> k{};

                for (auto it{ vec.cbegin() }; it != vec.cend(); it++)
                    {
                        auto* const                         data{ &(*it) };
                        const CWrap<elementType, Archive_T> e("element", const_cast<elementType* const>(data));
                        Archive_T                           j{};
                        e.Serialize(j);
                        k.push_back(j);
                    }

                archive.set(_name, k);
            }
        else
            {
                static_assert(always_false<T>(), "Unsupported type");
            }
    };

    virtual void Deserialize(std::reference_wrapper<const Archive_T> j) override
    {
        using namespace utils;

        const auto& archive{ j.get() };

        if constexpr (Reflectable<T, Archive_T>)
            {
                if (!archive.contains(_name))
                    throw std::runtime_error("Bad json.");
                const auto inner{ archive[_name] };

                const auto members{ _t->reflect(archive) };
                for (const auto& e : members)
                    {
                        const auto name{ e->Name() };
                        if (!inner.contains(name))
                            throw std::runtime_error("Bad json.");

                        e->Deserialize(inner);
                    }
            }
        else if constexpr ((std::is_trivial<T>::value || utils::Archivable<T, Archive_T>)&&!utils::is_std_vector<T>::value)
            {
                if (!archive.contains(_name))
                    throw std::runtime_error("Bad json.");

                auto& ref{ *_t };
                archive.get(_name, ref);
            }
        else if constexpr (utils::is_std_vector<T>::value)
            {
                using elementType = typename T::value_type;
                static_assert(!std::is_pointer<elementType>::value, "Must not be pointer");

                // static_assert(
                //     !std::is_trivial<elementType>::value &&
                //         !utils::Archivable<elementType, Archive_T>,
                //     "Must not have trivial types or archivable types because the
                //     " "archive_t must overload std::vector with any trivial
                //     types.");
                static_assert(Reflectable<elementType, Archive_T>, "Must have a Reflectable type.");

                std::vector<elementType>& vec{ *_t };

                if (!archive.contains(_name))
                    throw std::runtime_error("Bad json.");

                // Recursive deserialization
                std::vector<Archive_T> k{};
                archive.get(_name, k);
                vec.resize(k.size());

                for (auto it{ k.cbegin() }; it != k.cend(); it++)
                    {
                        elementType* const            data{ &(vec.at(std::distance(k.cbegin(), it))) };
                        CWrap<elementType, Archive_T> e("element", data);
                        e.Deserialize(*it);
                    }
            }
        else
            {
                static_assert(always_false<T>(), "Unsupported type");
            }

        if constexpr (HasPostLoad<T>)
            {
                _t->postLoad();
            }
    }
};

/**
 * /brief Templated type erasure wrapper for serialization of a derived class.
 * /tparam T
 */
template<typename T, typename Archive_T>
struct CWrapPolymorphic : public CWrap<T, Archive_T>
{
    metaptr<Archive_T> _b;

    CWrapPolymorphic(std::string_view name, metaptr<Archive_T> base, T* const t) : CWrap<T, Archive_T>(name, t), _b(base){};
    void Serialize(std::reference_wrapper<Archive_T> j) const override
    {
        auto& archive{ j.get() };
        CWrap<T, Archive_T>::Serialize(archive);
        // Inject the base class
        auto entry{ archive[CWrap<T, Archive_T>::Name()] };
        _b->Serialize(entry);
        // Apply changes
        archive.set(CWrap<T, Archive_T>::Name(), entry);
    };
    void Deserialize(std::reference_wrapper<const Archive_T> j) override
    {
        const auto& archive{ j.get() };
        CWrap<T, Archive_T>::Deserialize(archive);

        const auto inner{ archive[CWrap<T, Archive_T>::_name] };

        if (!inner.contains(IWrap<Archive_T>::INHERIT_TOKEN))
            throw std::runtime_error("Bad json.");

        // Inject the base class
        _b->Deserialize(std::ref(inner));
    }
};

/**
 * /brief Templated type erasure wrapper for serialization.
 * /tparam T The type to be serialized.
 * /tparam Archive_T The archive used to serialize.
 */
template<typename T, typename Archive_T>
struct CEnumWrap : public CWrap<T, Archive_T>
{
    using base = CWrap<T, Archive_T>;
    const std::string_view _enumName;
    CEnumWrap(std::string_view name, T* const t, std::string_view enumName) : CWrap<T, Archive_T>(name, t), _enumName(enumName) { assert(!_enumName.empty()); };

    virtual void Serialize(std::reference_wrapper<Archive_T> j) const override
    {
        auto& archive{ j.get() };

        Archive_T inner{};
        inner.set("Value", *base::_t);
        inner.set("EnumTypename", _enumName);
        archive.set(base::_name, inner);
    };

    virtual void Deserialize(std::reference_wrapper<const Archive_T> j) override
    {
        const auto& archive{ j.get() };

        if (!archive.contains(base::_name))
            return;

        const auto inner{ archive[base::_name] };
        if (!inner.contains("EnumTypename"))
            throw std::runtime_error("Serialized data is not an enum.");
        std::string enumTypename{};
        inner.get("EnumTypename", enumTypename);
        if (enumTypename != _enumName)
            throw std::runtime_error("Expected enum " + std::string(_enumName) + " but got instead " + enumTypename);

        if (!inner.contains("Value"))
            throw std::runtime_error("Serialized data is missing the enum value.");

        inner.get("Value", *base::_t);
    }
};

/**
 * /brief Helper to wrap primitives or reflected classes.
 */
namespace metaFrom
{
template<typename Archive_T, typename T>
inline static metaptr<Archive_T>
BaseClass(T* t)
{
    return metaptr<Archive_T>{ new CWrap<T, Archive_T>(IWrap<Archive_T>::INHERIT_TOKEN, t) };
}

template<typename Archive_T, typename T>
inline static metaptr<Archive_T>
Value(std::string_view name, T* const t)
{
    return metaptr<Archive_T>{ new CWrap<T, Archive_T>(name, t) };
}

template<typename Archive_T, typename TBase, typename T>
inline static metaptr<Archive_T>
DerivedClass(std::string_view name, TBase* base, T* t)
{
    auto baseClass{ metaFrom::BaseClass<Archive_T>(base) };
    return metaptr<Archive_T>{ new CWrapPolymorphic<T, Archive_T>(name, baseClass, t) };
}

template<typename Archive_T, typename T>
inline static metaptr<Archive_T>
Enum(std::string_view name, T* const t, std::string_view enumName)
{
    return metaptr<Archive_T>{ new CEnumWrap<T, Archive_T>(name, t, enumName) };
}

} // metaFrom

} // Fox