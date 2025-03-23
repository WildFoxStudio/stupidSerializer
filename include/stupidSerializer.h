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

#include <vector>
#include <type_traits>
#include <tuple>
#include <utility>
#include <functional>
#include <cassert>
#include <string>

namespace Fox
{
	/**
	 * /brief Base interface for serialization.
	 */
	template<typename Archive_T>
	struct IWrap
	{
		inline static constexpr char INHERIT_TOKEN[]{ "%inherit_from%" };
		virtual std::string Name()const = 0;
		virtual void Serialize(std::reference_wrapper<Archive_T> j) const = 0;
		virtual void Deserialize(std::reference_wrapper<const Archive_T> j) = 0;
	};

	template<typename Archive_T>
	using metaptr = std::shared_ptr<IWrap<Archive_T>>;
	template<typename Archive_T>
	using meta_list = std::vector<metaptr<Archive_T>>;

	namespace utils
	{
		/**
		 * /brief Check that reflect() returns a list of reflectable elements.
		 */
		template <typename T, typename Archive_T>
		concept Reflectable = requires(T & t, Archive_T archive) {
			{ t.template reflect<Archive_T>(archive) }  -> std::same_as<meta_list<Archive_T>>;
		};

		/**
		 * /brief Checks if has post load method.
		 */
		template <typename T>
		concept HasPostLoad = requires(T & t) {
			{ t.postLoad() } -> std::same_as<void>;
		};

		/**
		 * /brief The archive class concept.
		 */
		template<typename W>
		concept ArchiveConcept = requires(W w, const W cw, std::string key, int testValue, int outValue) {
			// Check that contains exists and returns a bool.
			{ w.contains(key) } -> std::convertible_to<bool>;
			{ cw.contains(key) } -> std::convertible_to<bool>;

			// Check that get works for an int.
			{ w.get(key, outValue) } -> std::convertible_to<bool>;

			// Check that set works for an int.
			{ w.set(key, testValue) } -> std::same_as<void>;

			// Check that operator[] returns a new wrapper of the same type.
			{ w[key] } -> std::same_as<W>;
			{ cw[key] } -> std::same_as<W>;
		};


		template<typename T, typename Wrapper>
		concept Archivable = requires(Wrapper w, const T & t, const std::string key, T temp, T outValue) {
			{ w.get(key, outValue) } -> std::convertible_to<bool>;
			{ w.set(key, t) } -> std::same_as<void>;
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
		//static_assert(utils::Reflectable<T>);
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
		const std::string _name;
		T* const _t;
		CWrap(std::string name, T* const t) : _name(name), _t(t) { assert(_t); };
		std::string Name()const override { return _name; }
		virtual void Serialize(std::reference_wrapper<Archive_T> j) const override {
			using namespace utils;

			auto& archive{ j.get() };
			if constexpr (Reflectable<T, Archive_T>)
			{
				//Recursive serialization
				Archive_T k{};
				for (const auto& e : _t->reflect(j.get()))
				{
					e->Serialize(k);
				}

				archive.set(_name, k);
			}
			else
				if constexpr (std::is_trivial<T>::value || utils::Archivable<T, Archive_T>)
				{
					const auto& ref{ *_t };
					archive.set(_name, ref);
				}
				else
					if constexpr (utils::is_std_vector<T>::value)
					{
						using elementType = typename T::value_type;
						static_assert((std::is_trivial<T>::value || utils::Archivable<T, Archive_T>));

						const std::vector<elementType>& vec{ *_t };

						// Recursive serialization
						std::vector<Archive_T> k{};

						for (auto it{ vec.cbegin() }; it != vec.cend(); it++)
						{
							const elementType* const data{ &(*it) };

							const CWrap e("element", data);

							Archive_T j{};
							e.Serialize(j);
							k.push_back(j);
						}

						archive.set(_name, k);
					}
					else
					{
						static_assert(false, "Unsupported type");
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
			else
				if constexpr (std::is_trivial<T>::value || utils::Archivable<T, Archive_T>)
				{
					if (!archive.contains(_name))
						throw std::runtime_error("Bad json.");

					auto& ref{ *_t };
					archive.get(_name, ref);
				}
				else
					if constexpr (utils::is_std_vector<T>::value)
					{
						using elementType = typename T::value_type;
						static_assert((std::is_trivial<T>::value || utils::Archivable<T, Archive_T>));

						std::vector<elementType>& vec{ *_t };

						if (!archive.contains(_name))
							throw std::runtime_error("Bad json.");

						// Recursive serialization
						std::vector<Archive_T> k{};
						archive.get(_name, k);
						vec.resize(k.size());

						for (auto it{ k.cbegin() }; it != k.cend(); it++)
						{
							elementType* const data{ &(vec.at(std::distance(k.cbegin(), it))) };
							CWrap e("element", data);
							data = e.Deserialize(*it);
						}
					}
					else
					{
						static_assert(false, "Unsupported type");
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

		CWrapPolymorphic(std::string name, metaptr<Archive_T> base, T* const t) : CWrap<T, Archive_T>(name, t), _b(base) {};
		void Serialize(std::reference_wrapper<Archive_T> j) const override {
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
	 * /brief Helper to wrap primitives or reflected classes.
	 */
	namespace metaFrom
	{
		template<typename Archive_T, typename T>
		inline static metaptr<Archive_T> BaseClass(T* t) { return metaptr<Archive_T>{ new CWrap<T, Archive_T>(IWrap<Archive_T>::INHERIT_TOKEN, t) }; }

		template<typename Archive_T, typename T>
		inline static metaptr<Archive_T>  Value(std::string name, T* const t) { return metaptr<Archive_T>{ new CWrap<T, Archive_T>(name, t) }; }

		template<typename Archive_T, typename TBase, typename T>
		inline static metaptr<Archive_T> DerivedClass(std::string name, TBase* base, T* t) {
			auto baseClass{ metaFrom::BaseClass<Archive_T>(base) };
			return metaptr<Archive_T>{ new CWrapPolymorphic<T, Archive_T>(name, baseClass, t) };
		}
	};
}