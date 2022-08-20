#pragma once

/*
 * Owning VArray wrapper with a typed interface.
 */
template<typename T>
struct VArrayTyped
{
	static std::shared_ptr<VArrayTyped<T>> create()	{ return std::shared_ptr<VArrayTyped<T>>(new VArrayTyped<T>());	}
	static std::shared_ptr<VArrayTyped<T>> create(VArray&& src)	{ return std::shared_ptr<VArrayTyped<T>>(new VArrayTyped<T>(src)); }

	// TODO: implement if needed :)

private:
	VArrayTyped(std::size_t initialSize) : src(typeid(T), sizeof(T), initialSize) {}
	explicit VArrayTyped(VArray&& src) : src(std::move(src)) {}

private:
	VArray src;
};
