#include<map>

struct InterfaceKitSettings{
  int serial_number;
  bool ratiometric;
  std::map<int, int> analogDataRates;
  std::map<int, int> analogChangeTriggers;
};
