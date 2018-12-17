#include <memory>   // std::make_unique
#include <string>   // std::string
#include <iostream> // std::{cout,cerr}

#include <boost/program_options.hpp> // @function parseConfig

#include "FullSystem/FullSystem.h"


struct Config {
  enum Preset { preset0 = 0, preset1, preset2, preset3 };
  enum Modes { mode0 = 0, mode1, mode2 };
  std::string m_sImagesPath;
  std::string m_sCalibPath;
  int m_iPreset = Preset::preset0;
  int m_iMode = Modes::mode0;

  static bool parseConfig(int argc, char* argv[]);
};
static Config *g_pConfig = nullptr;

bool Config::parseConfig(int argc, char* argv[]) {
    namespace po = boost::program_options;

    try {
      po::options_description desc{"Options"};
      desc.add_options()
        ("help,h", "Help screen")
        ("files,f", po::value<std::string>(&g_pConfig->m_sImagesPath)->required(), "")
        ("calib,c", po::value<std::string>(&g_pConfig->m_sCalibPath)->required(), "")
        ("preset,p", po::value<int>(&g_pConfig->m_iPreset), "")
        ("mode,m", po::value<int>(&g_pConfig->m_iMode), "");

      po::variables_map vm;
      store(parse_command_line(argc, argv, desc), vm);
      notify(vm);

      if (vm.count("help")) {
        std::cout << desc << '\n';
        return false;
      }
    } catch (const po::error &ex) {
      std::cerr << ex.what() << '\n';
      return false;
    }

    return true;
}

int main(int argc, char* argv[]) {
  Config config;
  g_pConfig = &config;

  if(!Config::parseConfig(argc, argv)) {
    return 1;
  }

  auto pFullSystem = std::make_unique<dso::FullSystem>();

  return 0;
}

