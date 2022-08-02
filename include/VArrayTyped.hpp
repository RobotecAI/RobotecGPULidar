#pragma once

/*
 * Owning VArray wrapper with a typed interface.
 */
template<typename T>
struct VArrayTyped
{
	static VArrayTyped<T>::Ptr create()	{ return VArrayTyped<T>::Ptr(new VArrayTyped<T>());	}
	static VArrayTyped<T>::Ptr create(VArray&& src)	{ return VArrayTyped<T>::Ptr(new VArrayTyped<T>(src)); }

	// TODO: implement if needed :)

private:
	VArrayTyped(std::size_t initialSize) : src(typeid(T), sizeof(T), initialSize) {}
	explicit VArrayTyped(VArray&& src) : src(std::move(src)) {}

private:
	VArray src;
};
