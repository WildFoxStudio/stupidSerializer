# stupidSerializer

Just a header only stupid templated c++20 serializer/deserializer.

Disclaimer: Only std::vector is serializable because lack of time.

# Usage example

```cpp
using namespace Fox;

struct Vec3
{
	float x, y, z;

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("x", &x),
			metaFrom::Value<Archive_T>("y", &y),
			metaFrom::Value<Archive_T>("z", &z), };
	}
};

struct metaVec3 : public meta<Vec3>
{
	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("Vec3", t)
		};
	}
};
```

Now serialization

```cpp
    Vec3 vec{0,1,2};
	CJsonArchive j{};//Defined elsewhere
	metaVec3{&vec}.reflect(j).at(0)->Serialize(j);
	std::cout << j.dump(4) << std::endl;
```

with output
```json
{
    "Vec3": {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0
            }
}
```


Otherwise if you don't care about the struct/class name

```cpp
    Vec3 vec{0,1,2};
	CJsonArchive j{};//Defined elsewhere
    for (const auto& e : vec.reflect(j))
	{
		e->Serialize(j);
	}
	std::cout << j.dump(4) << std::endl;
```
With output:
```json
{    
    "x": 0.0,
    "y": 1.0,
    "z": 2.0
}
```


# Polymorphic classes

```cpp
struct Component
{
	int Value{};
	Vec3 Pos{};

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("Value", &Value),
			metaFrom::Value<Archive_T>("Pos", &Pos)
		};
	}
};

struct metaComponent : meta<Component>
{
	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("Component", t)
		};
	}
};

struct Derived : public Component
{
	double Friction{};
	std::string Name;
	std::vector<std::string> Vector;

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("Friction", &Friction),
			metaFrom::Value<Archive_T>("Name", &Name),
			metaFrom::Value<Archive_T>("Vector", &Vector),
		};
	}
};

struct metaDerived : meta<Derived>
{
	metaComponent base{ t };

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::DerivedClass<Archive_T>("Derived", &base, t)
		};
	}
};

struct MoreDerived : public Derived
{
	double Foozbar{};

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::Value<Archive_T>("Foozbar", &Foozbar),
		};
	}
};

struct metaMoreDerived : meta<MoreDerived>
{
	metaDerived base{ t };

	template<typename Archive_T>
	auto reflect(Archive_T archive) -> meta_list<Archive_T> {
		return std::vector{
			metaFrom::DerivedClass<Archive_T>("MoreDerived", &base, t)
		};
	}
};
```

# Basic archive structure for nlohmann::json

```cpp

// Primary template: assume T is not serializable by default.
template<typename T, typename = void>
struct is_json_serializable : std::false_type {};

// Specialization: SFINAE kicks in if the following expression is valid:
//   to_json(json&, const T&)
template<typename T>
struct is_json_serializable<T, std::void_t<
	decltype(to_json(std::declval<nlohmann::json&>(), std::declval<const T&>()))
	>> : std::true_type {};

// Helper variable template (C++14 and later)
template<typename T>
inline constexpr bool is_json_serializable_v = is_json_serializable<T>::value;

class CJsonArchive {
public:
	CJsonArchive() = default;

	explicit CJsonArchive(const nlohmann::json& j)
		: m_json(j) {
	}

	explicit CJsonArchive(nlohmann::json&& j)
		: m_json(std::move(j)) {
	}

	CJsonArchive(const CJsonArchive& other) {
		m_json = other.m_json;
	}

	// Copy assignment operator.
	CJsonArchive& operator=(const CJsonArchive& other) {
		m_json = other.m_json;
		return *this;
	}

	CJsonArchive& operator=(const nlohmann::json& json) {
		m_json = json;
		return *this;
	}

	// Checks if a key exists in the underlying JSON.
	bool contains(const std::string& key) const {
		return m_json.contains(key);
	}

	// Templated getter that uses nlohmann::json::get_to.
	// Returns true if the key exists and the conversion succeeds.
	template<typename T, typename std::enable_if_t<is_json_serializable_v<T>, bool> = true>
	bool get(const std::string& key, T& outValue) const {
		//static_assert(!utils::is_std_vector<T>::value);

		if (!contains(key))
			return false;
		try {
			m_json.at(key).get_to(outValue);
			return true;
		}
		catch (const nlohmann::json::exception&) {
			return false;
		}
	}

	template<typename T, typename std::enable_if_t<std::is_same_v<T, CJsonArchive>, bool> = true>
	bool get(const std::string& key, CJsonArchive& outValue) const {
		if (!contains(key))
			return false;
		try {
			nlohmann::json out{};
			static_assert(!utils::is_std_vector<T>::value);
			static_assert(!std::is_same_v<T, std::string>);
			m_json.at(key).get_to(out);
			outValue = out;
			return true;
		}
		catch (const nlohmann::json::exception&) {
			return false;
		}
	}


	// Templated setter that assigns a value to a key.
	template<typename T>
	void set(const std::string& key, const T& value) {
		m_json[key] = value;
	}

	// Templated setter that assigns a value to a key.
	template<>
	void set(const std::string& key, const CJsonArchive& value) {
		m_json[key] = value.m_json;
	}

	// operator[] returns a new wrapper that references the sub-object for the given key.
	// This allows chaining calls like: wrapper["key1"]["key2"].set(...);
	CJsonArchive operator[](const std::string& key)const {
		return CJsonArchive(m_json[key]);
	}

	inline std::string dump(const int i)const { return m_json.dump(i); }

private:

	nlohmann::json m_json;
};
```

# Contribution

Feel free to create a pull request.


# Credits
If you intend to use this lib give credit please. 