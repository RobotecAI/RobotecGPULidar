#pragma once

static std::string name(const std::type_info& type)
{
	std::string_view name = type.name();
	name.remove_prefix(name.find_first_not_of("0123456789"));
	return std::string(name);
}