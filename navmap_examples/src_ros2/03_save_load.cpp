// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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


// Guardar/cargar NavMap en JSON muy simple (demo) — geometría + una capa U8
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include "navmap_core/NavMap.hpp"

using json = nlohmann::json;

static void save_json(const navmap::NavMap & nm, const std::string & path)
{
  json j;
  j["x"] = nm.positions.x; j["y"] = nm.positions.y; j["z"] = nm.positions.z;
  j["tris"] = json::array();
  for(const auto & c: nm.navcels) {
    j["tris"].push_back({c.v[0], c.v[1], c.v[2]});
  }
  // Solo capa "occupancy" si existe
  auto occ_any = nm.layers.get("occupancy");
  if(occ_any) {
    auto occ = std::dynamic_pointer_cast<navmap::LayerView<uint8_t>>(occ_any);
    if(occ) {j["occupancy"] = occ->data();}
  }
  std::ofstream ofs(path); ofs << j.dump(2);
}

static bool load_json(navmap::NavMap & nm, const std::string & path)
{
  std::ifstream ifs(path); if(!ifs) {return false;}
  json j; ifs >> j;
  nm.positions.x = j["x"].get<std::vector<float>>();
  nm.positions.y = j["y"].get<std::vector<float>>();
  nm.positions.z = j["z"].get<std::vector<float>>();
  nm.navcels.resize(j["tris"].size());
  for(size_t i = 0; i < nm.navcels.size(); ++i) {
    auto t = j["tris"][i];
    nm.navcels[i].v[0] = t[0]; nm.navcels[i].v[1] = t[1]; nm.navcels[i].v[2] = t[2];
  }
  nm.surfaces.clear();
  auto s = nm.create_surface("map");
  for(size_t i = 0; i < nm.navcels.size(); ++i) {
    nm.add_navcel_to_surface(s, (navmap::NavCelId)i);
  }
  nm.rebuild_geometry_accels();
  if(j.contains("occupancy")) {
    auto occ = nm.add_layer<uint8_t>("occupancy", "occ", "%", 0);
    auto & v = occ->mutable_data();
    v = j["occupancy"].get<std::vector<uint8_t>>();
  }
  return true;
}

class SaveLoadNode : public rclcpp::Node {
public:
  SaveLoadNode()
  : Node("navmap_save_load")
  {
    // Construye una malla simple y guarda a /tmp/navmap.json, luego vuelve a cargar
    navmap::NavMap nm;
    auto v0 = nm.add_vertex({0, 0, 0});
    auto v1 = nm.add_vertex({1, 0, 0});
    auto v2 = nm.add_vertex({0, 1, 0});
    auto c0 = nm.add_navcel(v0, v1, v2);
    auto s = nm.create_surface("map");
    nm.add_navcel_to_surface(s, c0);
    auto occ = nm.add_layer<uint8_t>("occupancy", "occ", "%", 0);
    nm.layer_set<uint8_t>("occupancy", c0, 254);
    nm.rebuild_geometry_accels();

    save_json(nm, "/tmp/navmap.json");

    navmap::NavMap re;
    (void)load_json(re, "/tmp/navmap.json");
    RCLCPP_INFO(get_logger(), "Loaded back: vertices=%zu tris=%zu",
                re.positions.size(), re.navcels.size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveLoadNode>());
  rclcpp::shutdown();
  return 0;
}
