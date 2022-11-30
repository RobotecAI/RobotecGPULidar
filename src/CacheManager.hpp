// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

template<typename Key, typename CacheType>
struct CacheManager
{
	using Ptr = std::shared_ptr<CacheManager>;
	using ConstPtr = std::shared_ptr<const CacheManager>;

	CacheManager(int ageToDelete = 5) : ageToDelete(ageToDelete) {}

	void trigger()
	{
		auto it = cacheAge.cbegin();
		while (it != cacheAge.cend()) {
			if (it->second >= ageToDelete) {
				cache.erase(it->first);
				it = cacheAge.erase(it);
				continue;
			}
			++it;
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

private:
	std::unordered_map<Key, CacheType> cache;
	std::unordered_map<Key, int> cacheAge;

	int ageToDelete;
};
