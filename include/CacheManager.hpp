#pragma once

template<typename Key, typename CacheType>
struct CacheManager
{
	using Ptr = std::shared_ptr<CacheManager>;
	using ConstPtr = std::shared_ptr<const CacheManager>;

	CacheManager(int ageToDelete = 5) : ageToDelete(ageToDelete) {}

	void trigger()
	{
		for (auto& [key, age] : cacheAge) {
			if (++age >= ageToDelete) {
				cache.erase(key);
				cacheAge.erase(key);
			}
		}
	}

 	void insert(Key key, CacheType& value, bool needUpdate = false)
	{
		int age = needUpdate ? 1 : 0;
		cache.insert({key, value});
		cacheAge.insert({key, age});
	}

	void remove(Key key)
	{
		cache.erase(key);
		cacheAge.erase(key);
	}

	void setUpdated(Key key) { cacheAge.at(key) = 0; }
	bool isLatest(Key key) const { return cacheAge.at(key) == 0; }
	bool contains(Key key) const { return cache.contains(key); }
	CacheType& getValue(Key key) { return cache.at(key); }
	CacheType getValueCopy(Key key) { return cache.at(key); }

private:
	std::unordered_map<Key, CacheType> cache;
	std::unordered_map<Key, int> cacheAge;

	int ageToDelete;
};
