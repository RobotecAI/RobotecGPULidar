//
// Created by Robotec.AI on 10/12/2023.
//
#include <unordered_map>
#include <graph/Node.hpp>

__declspec(dllexport) std::unordered_map<APIObject<Node>*, std::shared_ptr<Node>> APIObject<Node>::instances;
